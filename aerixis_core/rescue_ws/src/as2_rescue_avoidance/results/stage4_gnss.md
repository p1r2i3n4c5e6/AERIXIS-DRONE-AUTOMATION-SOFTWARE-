Rules: gnss (separation 10 m, GNSS sigma 0.8 m). Seeds per family: 200. Episodes: 6000. Wall time: 100 s. "<10 m": episodes where the true distance between two drones went below the fleet separation; "collisions": centres below 2 m. "detour": largest distance from the planned line (the width a pass costs).

| policy | link | family | collisions | <10 m | completed | aborted | closest (m) | 1% (m) | detour mean/max (m) | time x ideal | right passes |
|---|---|---|---|---|---|---|---|---|---|---|---|
| learned | base_station | blocker | 0/200 | 1/200 | 200/200 | 0 | 9.61 | 10.86 | 4.8/22.65 | 1.03 | 72% |
| learned | base_station | crossing | 0/200 | 0/200 | 200/200 | 0 | 10.64 | 10.98 | 8.1/15.27 | 1.02 | 89% |
| learned | base_station | head_on | 0/200 | 0/200 | 200/200 | 0 | 10.53 | 11.06 | 7.8/11.15 | 0.99 | 100% |
| learned | base_station | overtaking | 0/200 | 1/200 | 200/200 | 0 | 9.43 | 10.21 | 9.5/15.54 | 1.12 | 81% |
| learned | base_station | random | 0/200 | 0/200 | 200/200 | 0 | 10.12 | 10.17 | 9.0/37.08 | 1.05 | 57% |
| learned | edge | blocker | 0/200 | 0/200 | 200/200 | 0 | 11.26 | 11.75 | 7.5/21.96 | 1.1 | 75% |
| learned | edge | crossing | 0/200 | 0/200 | 200/200 | 0 | 11.32 | 12.63 | 9.9/26.82 | 1.07 | 81% |
| learned | edge | head_on | 0/200 | 0/200 | 200/200 | 0 | 12.43 | 12.71 | 11.1/15.03 | 1.03 | 100% |
| learned | edge | overtaking | 0/200 | 0/200 | 200/200 | 0 | 10.78 | 11.27 | 13.3/21.36 | 1.27 | 80% |
| learned | edge | random | 0/200 | 0/200 | 200/200 | 0 | 11.23 | 11.92 | 11.4/27.96 | 1.12 | 56% |
| learned | lora | blocker | 0/200 | 0/200 | 200/200 | 0 | 11.3 | 11.67 | 8.4/24.9 | 1.12 | 75% |
| learned | lora | crossing | 0/200 | 0/200 | 200/200 | 0 | 11.54 | 13.42 | 10.7/23.54 | 1.09 | 86% |
| learned | lora | head_on | 0/200 | 0/200 | 200/200 | 0 | 13.07 | 13.75 | 12.0/19.22 | 1.05 | 100% |
| learned | lora | overtaking | 0/200 | 0/200 | 200/200 | 0 | 11.77 | 11.97 | 14.2/23.07 | 1.31 | 87% |
| learned | lora | random | 0/200 | 0/200 | 200/200 | 0 | 11.83 | 12.15 | 12.6/36.1 | 1.15 | 55% |
| orca | base_station | blocker | 0/200 | 1/200 | 200/200 | 0 | 9.58 | 10.99 | 3.9/17.46 | 1.05 | 73% |
| orca | base_station | crossing | 0/200 | 0/200 | 200/200 | 0 | 10.32 | 10.87 | 7.9/15.47 | 1.06 | 78% |
| orca | base_station | head_on | 0/200 | 0/200 | 200/200 | 0 | 10.49 | 11.02 | 7.9/12.36 | 1.02 | 100% |
| orca | base_station | overtaking | 0/200 | 2/200 | 199/200 | 0 | 9.39 | 10.21 | 13.0/34.51 | 1.24 | 80% |
| orca | base_station | random | 0/200 | 1/200 | 200/200 | 0 | 9.95 | 10.36 | 9.3/37.01 | 1.09 | 60% |
| orca | edge | blocker | 0/200 | 0/200 | 200/200 | 0 | 11.86 | 12.06 | 7.2/21.41 | 1.11 | 77% |
| orca | edge | crossing | 0/200 | 0/200 | 200/200 | 0 | 11.45 | 12.66 | 9.5/28.0 | 1.09 | 76% |
| orca | edge | head_on | 0/200 | 0/200 | 200/200 | 0 | 12.35 | 12.68 | 11.9/16.62 | 1.04 | 100% |
| orca | edge | overtaking | 0/200 | 0/200 | 196/200 | 0 | 10.82 | 11.25 | 14.5/38.13 | 1.49 | 79% |
| orca | edge | random | 0/200 | 0/200 | 200/200 | 0 | 11.04 | 11.49 | 11.2/30.16 | 1.13 | 59% |
| orca | lora | blocker | 0/200 | 0/200 | 200/200 | 0 | 11.83 | 12.27 | 7.8/25.96 | 1.12 | 77% |
| orca | lora | crossing | 0/200 | 0/200 | 200/200 | 0 | 12.14 | 13.62 | 11.2/32.65 | 1.1 | 80% |
| orca | lora | head_on | 0/200 | 0/200 | 200/200 | 0 | 12.87 | 13.91 | 13.1/20.54 | 1.06 | 100% |
| orca | lora | overtaking | 0/200 | 0/200 | 174/200 | 0 | 10.98 | 11.44 | 15.6/40.3 | 1.5 | 80% |
| orca | lora | random | 0/200 | 0/200 | 199/200 | 0 | 11.16 | 11.93 | 13.7/42.27 | 1.19 | 62% |
