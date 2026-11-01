#!/usr/bin/env python3
"""Optional layout preview; requires matplotlib. Does not change the arena."""
import json
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/rescue_arena_matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle

root = Path(__file__).resolve().parent
data = json.loads((root/'ground_truth/layout.json').read_text())
cfg = data['config']
fig, ax = plt.subplots(figsize=(12, 12), facecolor='#101f2a')
ax.set_facecolor('#e7eadb')
half = cfg['size_m']/2
for t in data['trees']:
    ax.add_patch(Circle((t['x'], t['y']), t['radius'], facecolor='#46754a', edgecolor='#305d36', lw=.5, alpha=.88))
ax.add_patch(Circle((0, 0), cfg['launch_clearance_radius_m'], facecolor='#dde7ee', edgecolor='#407993', lw=1.3, linestyle='--'))
ax.add_patch(Circle((0, 0), 7, facecolor='#334b5a'))
ax.text(0, 0, 'H', ha='center', va='center', color='#ffe084', weight='bold', fontsize=13)
ax.text(0, -23, 'LAUNCH', ha='center', fontsize=8, color='#25465b', weight='bold')
for h in data['humans']:
    ax.scatter(h['x'], h['y'], color='#d84924', edgecolor='white', s=40, linewidth=.7, zorder=3)
    ax.annotate(h['name'][-3:], (h['x'], h['y']), xytext=(4, 5), textcoords='offset points', fontsize=7.5, color='#692b1a', weight='bold')
ax.add_patch(Rectangle((-half, -half), 2*half, 2*half, fill=False, edgecolor='#284c43', lw=2))
ax.set(xlim=(-half-5, half+5), ylim=(-half-5, half+5), aspect='equal', xlabel='EAST  /  metres', ylabel='NORTH  /  metres')
ax.set_xticks(range(-150, 151, 50)); ax.set_yticks(range(-150, 151, 50))
ax.grid(color='#4b6b51', alpha=.15)
ax.tick_params(colors='#d0dce3')
ax.xaxis.label.set_color('#d0dce3'); ax.yaxis.label.set_color('#d0dce3')
for spine in ax.spines.values():
    spine.set_visible(False)
fig.suptitle('AERIXIS  /  RESCUE ARENA', x=.125, ha='left', y=.975, color='white', fontsize=23, weight='bold')
fig.text(.125, .928, f'300 × 300 m  ·  {len(data["humans"])} human targets  ·  {len(data["trees"])} trees', color='#a9c6d4', fontsize=13)
fig.text(.125, .055, f'Orange: people and target IDs     Green: tree canopy footprint\n'
         f'Closest people: {data["actual_min_human_spacing_m"]:.2f} m     Clear launch radius: 18 m     ENU frame', color='#a9c6d4', fontsize=10, linespacing=1.8)
fig.subplots_adjust(top=.9, bottom=.13, left=.12, right=.94)
fig.savefig(root/'arena_overview.png', dpi=160, facecolor=fig.get_facecolor())
print(root/'arena_overview.png')
