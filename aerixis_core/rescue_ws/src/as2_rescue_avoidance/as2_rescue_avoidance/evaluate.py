"""Evaluate avoidance policies over many encounters and write a report.

  avoidance_eval --seeds 300 --out results/stage1          (all policies, links and families)
  avoidance_eval --policies orca --links lora --families head_on --seeds 50
  avoidance_eval --policies orca,learned --model models/brain.npz --seed-start 100000 --out results/stage2
  avoidance_eval --rules rtk --policies stop_gate,orca,learned --model models/brain.npz --seed-start 100000

Every (policy, link, family, seed) is one episode; the same seeds are used for every policy, so the
policies meet exactly the same encounters. Runs on all CPU cores.
"""
import argparse
import json
from multiprocessing import Pool
import os
import statistics
import time

from . import links, scenarios, sim

_WEIGHTS = {}


def _episode(job):
    policy, link, family, seed, model, rules = job
    if policy == 'learned':
        from . import brain
        if model not in _WEIGHTS:
            _WEIGHTS[model] = brain.Network(brain.load(model))
        fly = brain.LearnedPolicy(_WEIGHTS[model])    # one per episode: it remembers decisions
    else:
        fly = sim.POLICIES[policy]
    result = sim.run_episode(scenarios.make(family, seed), fly, links.PROFILES[link], seed, rules=sim.RULES[rules])
    return dict(policy=policy, link=link, family=family, seed=seed, **result)


def _percentile(values, q):
    values = sorted(values)
    return values[min(len(values) - 1, int(q * len(values)))] if values else None


def summarise(rows):
    groups = {}
    for row in rows:
        groups.setdefault((row['policy'], row['link'], row['family']), []).append(row)
    table = []
    for (policy, link, family), items in sorted(groups.items()):
        n = len(items)
        delay = [r['time'] / r['ideal_time'] for r in items if r['completed'] and r['ideal_time'] > 0]
        passes = sum(r['passes'] for r in items)
        table.append(dict(
            policy=policy, link=link, family=family, episodes=n,
            collision_episodes=sum(r['collisions'] > 0 for r in items),
            violation_episodes=sum(r['violation_s'] > 0 for r in items),
            completed=sum(r['completed'] for r in items),
            aborted_missions=sum(r['aborted'] for r in items),
            min_distance=min(r['min_distance'] for r in items),
            p1_min_distance=_percentile([r['min_distance'] for r in items], 0.01),
            median_min_distance=statistics.median(r['min_distance'] for r in items),
            mean_path_error=round(statistics.fmean(r['max_path_error'] for r in items), 1),
            max_path_error=max(r['max_path_error'] for r in items),
            mean_delay=round(statistics.fmean(delay), 2) if delay else None,
            right_pass_share=round(sum(r['right_passes'] for r in items) / passes, 2) if passes else None))
    return table


def markdown(table, seeds, seconds, rules=sim.GNSS):
    lines = ['| policy | link | family | collisions | <SEP m | completed | aborted | closest (m) | 1% (m) | '
             'detour mean/max (m) | time x ideal | right passes |',
             '|---|---|---|---|---|---|---|---|---|---|---|---|']
    lines[0] = lines[0].replace('SEP', '%g' % rules.separation)
    for t in table:
        lines.append('| {policy} | {link} | {family} | {collision_episodes}/{episodes} | {violation_episodes}/{episodes} | '
                     '{completed}/{episodes} | {aborted_missions} | {min_distance} | {p1_min_distance} | '
                     '{mean_path_error}/{max_path_error} | {delay} | {right} |'.format(
                         delay=t['mean_delay'] if t['mean_delay'] is not None else '-',
                         right='{:.0%}'.format(t['right_pass_share']) if t['right_pass_share'] is not None else '-', **t))
    head = ('Rules: {r.name} (separation {r.separation:g} m, GNSS sigma {r.gnss_sigma:g} m). Seeds per family: {}. '
            'Episodes: {}. Wall time: {:.0f} s. "<{r.separation:g} m": episodes where the true distance between '
            'two drones went below the fleet separation; "collisions": centres below {r.collision:g} m. '
            '"detour": largest distance from the planned line (the width a pass costs).'
            .format(seeds, sum(t['episodes'] for t in table), seconds, r=rules))
    return head + '\n\n' + '\n'.join(lines) + '\n'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--policies', default=','.join(sim.POLICIES))
    parser.add_argument('--links', default=','.join(links.PROFILES))
    parser.add_argument('--families', default=','.join(scenarios.FAMILIES))
    parser.add_argument('--seeds', type=int, default=300)
    parser.add_argument('--seed-start', type=int, default=0, help='first seed (stage 2 tests on 100000+)')
    parser.add_argument('--model', default='', help='brain weights (.npz) for the "learned" policy')
    parser.add_argument('--workers', type=int, default=max(1, (os.cpu_count() or 2) - 2))
    parser.add_argument('--rules', choices=sorted(sim.RULES), default='gnss', help='gnss (10 m) or rtk (3 m)')
    parser.add_argument('--out', default='', help='write <out>.json (all episodes + table) and <out>.md')
    args = parser.parse_args(argv)
    if 'learned' in args.policies.split(',') and not args.model:
        parser.error('--model is required for the learned policy')
    jobs = [(p, l, f, s, args.model, args.rules) for p in args.policies.split(',') for l in args.links.split(',')
            for f in args.families.split(',') for s in range(args.seed_start, args.seed_start + args.seeds)]
    start = time.time()
    with Pool(args.workers) as pool:
        rows = pool.map(_episode, jobs, chunksize=20)
    seconds = time.time() - start
    table = summarise(rows)
    report = markdown(table, args.seeds, seconds, sim.RULES[args.rules])
    print(report)
    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        with open(args.out + '.json', 'w') as f:
            json.dump(dict(table=table, episodes=rows), f)
        with open(args.out + '.md', 'w') as f:
            f.write(report)


if __name__ == '__main__':
    main()
