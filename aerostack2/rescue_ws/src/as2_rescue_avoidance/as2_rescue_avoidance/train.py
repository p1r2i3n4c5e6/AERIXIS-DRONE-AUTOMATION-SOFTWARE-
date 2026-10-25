"""Train the avoidance brain with PPO (laptop: CPU workers simulate, PyTorch learns, GPU if present).

  python3 -m as2_rescue_avoidance.train --iterations 300 --out models/brain
  python3 -m as2_rescue_avoidance.train --rules rtk,gnss --out models/brain   (one brain for 3 m and 10 m)

All moving drones share one network and learn from their own mesh view only (decentralised: in the
air nobody sees the whole picture). The ORCA shield is in the loop during training exactly as in
flight, so the network learns to work with it. Orbiting "blocker" drones fly plain ORCA.

Reward per drone, summed over each 0.5 s decision:
  + progress along its own route, capped at mission speed (1.0 per decision), + 5 for finishing it
    (run 1 without the cap learned to fly 1.2x the operator's mission speed all the time)
  - 0.05 per step and 10 % above mission speed (faster only when it pays, e.g. to clear a pass)
  - time: 0.4 per decision until its mission is done (run 2 without it learned to slow down and
    lose time: discounting alone is too weak a reason to finish)
  - closeness: 0.1 per step and metre under 1.2 x the separation, + 2 per step under it; - 50 and
    the episode ends on a collision
  - shield: how much ORCA had to change the network's velocity (/ mission speed, 0.1 per step)
  - 0.01 per step and metre away from the planned line (--line-weight; run 3 had 0.002 and flew
    wider than ORCA: the brain should not add width to the shield's detour)
Seeds: training 10 000 000+, validation 500 000+, final test 100 000+ (evaluate.py) never overlap.
"""
import argparse
import json
import math
from multiprocessing import get_context
import os
import random
import time

import numpy as np

from . import brain, links, scenarios, sim

FAMILY_WEIGHTS = {'head_on': 0.2, 'crossing': 0.2, 'overtaking': 0.2, 'blocker': 0.15, 'random': 0.25}
LINKS = tuple(links.PROFILES)
STEPS_PER_DECISION = int(round(brain.DECISION_S / sim.DT))
VALIDATION_SEEDS = range(500_000, 500_000 + 30)     # x 5 families x 3 links = 450 episodes


# ------------------------------------------------------------------ rollout (worker processes)
def _gaussian_logp(raw, mean, log_std):
    std = np.exp(log_std)
    return float(-0.5 * np.sum(((raw - mean) / std) ** 2 + 2 * log_std + math.log(2 * math.pi)))


def rollout(job):
    """Fly one training episode with a stochastic policy; returns per-drone trajectories + stats."""
    weights, log_std, family, link, seed, rules, line_weight = job
    rules = sim.RULES[rules]
    net = brain.Network(weights)
    scenario = scenarios.make(family, seed)
    episode = sim.Episode(scenario, links.PROFILES[link], seed, rules=rules)
    near, separation = 1.2 * rules.separation, rules.separation
    rng = np.random.default_rng(seed)
    learners = [i for i, d in enumerate(episode.drones) if not d.loop and len(d.route) > 1]
    traj = {i: dict(obs=[], act=[], logp=[], rew=[], terminal=False, last_obs=None) for i in learners}
    held, active = {}, set(learners)
    last_remaining = {}
    step = 0
    collided = False
    while not episode.finished and active:
        views = episode.views()
        commands = []
        for v in views:
            i = v['id']
            if i in active:
                # Reward for the step that just happened (progress, closeness, line, shield).
                if i in last_remaining:
                    progress = min(last_remaining[i] - v['remaining'], v['speed'] * sim.DT)
                    r = progress / max(0.3, v['speed'] * brain.DECISION_S)
                    r -= line_weight * abs(v['cross_track'])
                    r -= 0.5 * max(0.0, math.hypot(*v['vel']) / max(0.3, v['speed']) - 1.05)
                    traj[i]['rew'][-1] += r
                last_remaining[i] = v['remaining']
                if v['done']:
                    traj[i]['rew'][-1] += 5.0
                    traj[i]['terminal'] = True
                    active.discard(i)
                    commands.append(v['pref'])
                    continue
                if step % STEPS_PER_DECISION == 0:
                    if traj[i]['rew']:
                        traj[i]['rew'][-1] -= 0.4
                    obs = brain.observe(v)
                    mean = net(obs)
                    raw = mean + np.exp(log_std) * rng.standard_normal(2)
                    traj[i]['obs'].append(obs); traj[i]['act'].append(raw.astype(np.float32))
                    traj[i]['logp'].append(_gaussian_logp(raw, mean, log_std)); traj[i]['rew'].append(0.0)
                    held[i] = raw
                desired = brain.desired_velocity(v, held[i])
                executed = brain.shield(v, desired)
                traj[i]['rew'][-1] -= 0.1 * math.dist(executed, desired) / max(0.3, v['speed'])
                commands.append(executed)
            elif v['loop'] or i not in traj:
                commands.append(sim.policy_orca(v))
            else:
                commands.append(v['pref'])
        episode.step(commands)
        step += 1
        for (a, b), dist in episode.pair_dist.items():
            for i in (a, b):
                if i in active and traj[i]['rew']:
                    traj[i]['rew'][-1] -= 0.1 * max(0.0, near - dist) + (2.0 if dist < separation else 0.0)
            if dist < rules.collision:
                collided = True
                for i in (a, b):
                    if i in active and traj[i]['rew']:
                        traj[i]['rew'][-1] -= 50.0
                        traj[i]['terminal'] = True
        if collided:
            break
    if not collided:
        final = {v['id']: v for v in episode.views()}
        for i in active:
            if traj[i]['obs'] and not traj[i]['terminal']:
                traj[i]['last_obs'] = brain.observe(final[i])      # truncated: bootstrap
    out = [t for t in traj.values() if t['obs']]
    result = episode.result()
    return out, dict(family=family, link=link, collisions=result['collisions'], completed=result['completed'],
                     violation=result['violation_s'] > 0, min_distance=result['min_distance'],
                     delay=result['time'] / max(1.0, result['ideal_time']))


def validate_episode(job):
    weights, family, link, seed, rules = job
    policy = sim.policy_orca if weights is None else brain.LearnedPolicy(weights)
    r = sim.run_episode(scenarios.make(family, seed), policy, links.PROFILES[link], seed, rules=sim.RULES[rules])
    return dict(family=family, link=link, **r)


def score(rows):
    n = len(rows)
    done = [r for r in rows if r['completed']]
    return dict(episodes=n, collisions=sum(r['collisions'] > 0 for r in rows),
                below_separation=sum(r['violation_s'] > 0 for r in rows), completed=len(done),
                min_distance=min(r['min_distance'] for r in rows),
                delay=round(sum(r['time'] / r['ideal_time'] for r in done) / max(1, len(done)), 3),
                detour=round(sum(r['max_path_error'] for r in rows) / n, 2))


def better(a, b):
    """Safety first (collisions, then episodes below the separation), then completed missions, then
    time, then width (mean detour)."""
    if b is None:
        return True
    ka = (a['collisions'], a['below_separation'], -a['completed'], a['delay'], a['detour'])
    kb = (b['collisions'], b['below_separation'], -b['completed'], b['delay'], b['detour'])
    return ka < kb


# ------------------------------------------------------------------ learner (main process)
def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--iterations', type=int, default=300)
    parser.add_argument('--episodes', type=int, default=96, help='training episodes per iteration')
    parser.add_argument('--workers', type=int, default=max(1, (os.cpu_count() or 2) - 2))
    parser.add_argument('--validate-every', type=int, default=10)
    parser.add_argument('--lr', type=float, default=3e-4)
    parser.add_argument('--out', default='models/brain')
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--rules', default='gnss',
                        help='separation rules to train on, comma separated (first = 2/3 of the episodes): '
                             'gnss (10 m), rtk (3 m, RTK on every drone)')
    parser.add_argument('--line-weight', type=float, default=0.01,
                        help='penalty per step and metre off the planned line (run 3: 0.002)')
    parser.add_argument('--init', default='', help='start the actor from these weights (.npz, same size)')
    args = parser.parse_args(argv)
    rules = args.rules.split(',')
    if not rules or any(r not in sim.RULES for r in rules):
        parser.error('--rules: choose from %s' % ', '.join(sorted(sim.RULES)))
    rule_weights = [2.0] + [1.0 / max(1, len(rules) - 1)] * (len(rules) - 1)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)

    # Workers first (fork before CUDA exists in this process).
    pool = get_context('fork').Pool(args.workers)
    import torch
    from torch import nn
    torch.manual_seed(args.seed)
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    def mlp(sizes, out_tanh):
        layers = []
        for k in range(len(sizes) - 1):
            layers.append(nn.Linear(sizes[k], sizes[k + 1]))
            if k < len(sizes) - 2 or out_tanh:
                layers.append(nn.Tanh())
        return nn.Sequential(*layers)

    actor = mlp([brain.OBS_SIZE, 64, 64, 2], True).to(device)
    with torch.no_grad():                       # start at "plain ORCA": output ~0
        actor[-2].weight.mul_(0.01); actor[-2].bias.zero_()
        if args.init:
            init = brain.load(args.init)
            for k, m in enumerate(m for m in actor if isinstance(m, nn.Linear)):
                m.weight.copy_(torch.as_tensor(init['w%d' % k].T)); m.bias.copy_(torch.as_tensor(init['b%d' % k]))
    critic = mlp([brain.OBS_SIZE, 128, 128, 1], False).to(device)
    log_std = nn.Parameter(torch.full((2,), -0.7, device=device))
    optimiser = torch.optim.Adam(list(actor.parameters()) + list(critic.parameters()) + [log_std], lr=args.lr)

    def export():
        linear = [m for m in actor if isinstance(m, nn.Linear)]
        return {**{'w%d' % k: m.weight.detach().cpu().numpy().T.copy() for k, m in enumerate(linear)},
                **{'b%d' % k: m.bias.detach().cpu().numpy().copy() for k, m in enumerate(linear)}}

    val_jobs = [(f, l, s, r) for r in rules for f in scenarios.FAMILIES for l in LINKS for s in VALIDATION_SEEDS]
    baseline = score(pool.map(validate_episode, [(None, *j) for j in val_jobs], chunksize=8))
    print('ORCA on validation seeds:', json.dumps(baseline), flush=True)
    history, best = [], None
    rng = random.Random(args.seed)
    seed_counter = 10_000_000 + args.seed * 1_000_000
    start = time.time()
    gamma, lam, clip = 0.99, 0.95, 0.2
    for iteration in range(1, args.iterations + 1):
        weights = export()
        std = log_std.detach().cpu().numpy().astype(np.float64)
        jobs = []
        for _ in range(args.episodes):
            family = rng.choices(list(FAMILY_WEIGHTS), weights=list(FAMILY_WEIGHTS.values()))[0]
            seed_counter += 1
            rule = rng.choices(rules, weights=rule_weights)[0]
            jobs.append((weights, std, family, rng.choice(LINKS), seed_counter, rule, args.line_weight))
        results = pool.map(rollout, jobs, chunksize=2)
        trajectories = [t for out, _ in results for t in out]
        stats = [s for _, s in results]

        # GAE per trajectory (critic on the GPU).
        obs_all, act_all, logp_all, adv_all, ret_all = [], [], [], [], []
        with torch.no_grad():
            for t in trajectories:
                obs = torch.as_tensor(np.stack(t['obs']), device=device)
                values = critic(obs).squeeze(-1).cpu().numpy()
                last = 0.0
                if t['last_obs'] is not None:
                    last = float(critic(torch.as_tensor(t['last_obs'][None], device=device)).item())
                rewards = np.asarray(t['rew'], np.float32)
                adv = np.zeros_like(rewards)
                gae = 0.0
                for k in reversed(range(len(rewards))):
                    nxt = values[k + 1] if k + 1 < len(rewards) else (0.0 if t['terminal'] else last)
                    delta = rewards[k] + gamma * nxt - values[k]
                    gae = delta + gamma * lam * gae
                    adv[k] = gae
                obs_all.append(t['obs']); act_all.append(t['act']); logp_all.append(t['logp'])
                adv_all.append(adv); ret_all.append(adv + values)
        obs = torch.as_tensor(np.concatenate([np.stack(o) for o in obs_all]), device=device)
        act = torch.as_tensor(np.concatenate([np.stack(a) for a in act_all]), device=device)
        old_logp = torch.as_tensor(np.concatenate([np.asarray(l, np.float32) for l in logp_all]), device=device)
        adv = torch.as_tensor(np.concatenate(adv_all), device=device)
        ret = torch.as_tensor(np.concatenate(ret_all), device=device)
        adv = (adv - adv.mean()) / (adv.std() + 1e-8)

        n = obs.shape[0]
        for _ in range(6):
            order = torch.randperm(n, device=device)
            for start_k in range(0, n, 4096):
                idx = order[start_k:start_k + 4096]
                mean = actor(obs[idx])
                std_t = torch.exp(log_std)
                logp = (-0.5 * (((act[idx] - mean) / std_t) ** 2 + 2 * log_std + math.log(2 * math.pi))).sum(-1)
                ratio = torch.exp(logp - old_logp[idx])
                policy_loss = -torch.min(ratio * adv[idx], torch.clamp(ratio, 1 - clip, 1 + clip) * adv[idx]).mean()
                value_loss = ((critic(obs[idx]).squeeze(-1) - ret[idx]) ** 2).mean()
                entropy = log_std.sum()
                loss = policy_loss + 0.5 * value_loss - 1e-3 * entropy
                optimiser.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(list(actor.parameters()) + list(critic.parameters()), 0.5)
                optimiser.step()
            with torch.no_grad():
                log_std.clamp_(-3.0, 0.0)

        row = dict(iteration=iteration, seconds=round(time.time() - start), transitions=n,
                   mean_return=round(float(np.mean([sum(t['rew']) for t in trajectories])), 2),
                   collisions=sum(s['collisions'] > 0 for s in stats), below_separation=sum(s['violation'] for s in stats),
                   completed=sum(s['completed'] for s in stats), episodes=len(stats),
                   delay=round(float(np.mean([s['delay'] for s in stats if s['completed']] or [0])), 3),
                   std=[round(x, 3) for x in torch.exp(log_std).tolist()])
        if iteration % args.validate_every == 0 or iteration == args.iterations:
            weights = export()
            val = score(pool.map(validate_episode, [(weights, *j) for j in val_jobs], chunksize=8))
            row['validation'] = val
            if better(val, best):
                best = val
                np.savez(args.out + '.npz', **weights)
                row['saved'] = True
        history.append(row)
        print(json.dumps(row), flush=True)
        with open(args.out + '.history.json', 'w') as f:
            json.dump(dict(baseline_orca=baseline, best=best, history=history, args=vars(args)), f)
    pool.close()


if __name__ == '__main__':
    main()
