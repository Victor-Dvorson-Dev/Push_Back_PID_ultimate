# Autonomous Run Performance Summary (2026-03-13)

Source: console output from the last run of `src/main_Sophia.py`.

## Movement Results

```text
#  Target(in,deg)  LinErr(in)  HeadErr(deg)  Result    Timeout   Cycles  Time(s)
1  -24.50,   0.0   -0.61       +1.2          TIMEOUT   Yes       201     1.00
2    0.00, -80.0   -0.25       +2.3          OK        No         97     0.48
3  -14.00, -80.0   -0.69       +2.2          TIMEOUT   Yes       171     0.85
4   11.00, -80.0   +0.28       +1.6          OK        No         98     0.49
5    0.00, 101.0   -0.08       +1.4          OK        No        140     0.70
6  -36.50, 101.0   -0.82       +1.4          TIMEOUT   Yes       251     1.25
7    0.00, 145.0   -0.10       +2.2          OK        No         88     0.44
8   -8.50, 145.0   -0.67       -1.1          TIMEOUT   Yes       176     0.88
9   30.00, 145.0   +3.11       +4.4          TIMEOUT   Yes       201     1.00
10  -6.00, 145.0   +0.17       +2.8          OK        No         88     0.44
11   0.00, 237.0   +0.08       +1.1          OK        No        102     0.51
12 -10.00, 237.0   -0.91       +1.5          TIMEOUT   Yes       121     0.60
13   0.00, 139.0   -0.18       -0.9          OK        No        105     0.53
14  21.00, 149.0   +4.07       -47.1         TIME_UP   Deadline  189     0.94
```

## Totals and Summary

- Completed with `OK`: **7 / 14**
- Ended by `TIMEOUT`: **6 / 14**
- Ended by `TIME_UP` (global deadline): **1 / 14**
- Total movement-loop time (sum of segment times): **10.11 s**
- Final brain time at stop: **~14.8 s**
- Deadline setting currently in code: **14.9 s**

## Distance Error Summary

- Mean absolute distance error: **0.92 in**
- Median absolute distance error: **0.48 in**
- Best distance error: **0.08 in**
- Worst distance error: **4.07 in**
- Mean absolute distance error excluding moves #9 and #14: **~0.40 in**

## Notes

- Most `TIMEOUT` segments were close to target in both distance and heading.
- Largest misses occurred late in the run under time pressure.
