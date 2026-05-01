# Friction-sweep ablation — handoff to Kevin

This document specifies exactly what the paper expects so the entries in
Table III ("TBD" markers) can be filled in. The study directly answers the
prof's review note ("Did you try different friction coefficients?").

## What the paper claims (Section VII-D)

> All main results use ODE Coulomb friction $\mu = 0.6$ between marble and
> plate (Gazebo default, consistent with steel-on-acrylic). To verify
> robustness, we sweep $\mu \in \{0.3, 0.5, 0.6, 0.8, 1.0\}$ on Scenario 1
> with all other parameters fixed and run 10 paired trials per coefficient
> for both pure dLQR and the 200k Hybrid policy.

## Protocol

| Item | Value |
|------|-------|
| Scenario | 1 only (0.12 m spawn, Lissajous TCP, no extra jitter) |
| Trials per μ | 10, paired (same seeds for LQR and Hybrid) |
| Seeds | 0..9 |
| Episode length | 500 control steps (~16.7 s @ 30 Hz) |
| Termination | marble leaves \|x\|+\|y\| > 0.30 m |
| Hybrid policy | TD3 v2 checkpoint at 200,000 steps |
| All other parameters | as specified in `config/lqr_params.yaml` and the paper |

## What to change in Gazebo

The marble friction lives in the marble SDF (or in the plate SDF, depending on
where you defined it). Look for:

```xml
<surface>
  <friction>
    <ode>
      <mu>0.6</mu>
      <mu2>0.6</mu2>
    </ode>
  </friction>
</surface>
```

Change `mu` and `mu2` together. Keep `restitution = 0.0` and contact
stiffness unchanged. The plate side of the contact pair should stay at the
default; modify only the marble's surface block to keep the change localized.

## Metrics to record per μ (and per controller)

For both pure LQR and Hybrid 200k:

| Metric | Aggregation |
|--------|-------------|
| Win-rate | fraction of 10 trials reaching H=500 steps |
| RMSE | per-trial RMSE of marble distance from centre, averaged over surviving trials |
| Max peak error | maximum of per-trial max distance, across all 10 trials |
| Mean total reward | mean of episode-summed reward (using the same reward function 8 in the paper) |

## Where the numbers go in the paper

The friction table is `\label{tab:friction}` in
`Hybrid_Optimal_Residual_RL_Ball_Plate.tex` around line 678. Each red
**TBD** marker is one number to replace. The expected layout is:

```latex
$0.3$ & <LQR_win>  & <Hybrid_win>  & <delta>  & <Hybrid_RMSE / LQR_RMSE> \\
```

The $\Delta_\text{win}$ column is `Hybrid_win - LQR_win` in percentage
points.

## How to update the paper

1. Replace each `\todoK` cell with the actual number.
2. If you want to keep the prof's exact wording, also append a sentence to the
   paragraph above the table summarizing the trend (e.g., "The win-rate gap
   $\Delta_\text{win}$ peaks at $\mu=0.6$ and attenuates at the extremes,
   confirming that the residual is tuned to the nominal contact regime").
3. Rebuild with: `pdflatex; bibtex; pdflatex; pdflatex`.

## Suggested narrative if the result is positive

If $\Delta_\text{win} > 0$ at $\mu=0.6$ and tapers off at extremes, the
paper claims the operational envelope generalizes across the realistic
contact range. If the gap inverts at some $\mu$, that becomes a fourth
operational-envelope finding — also a publishable result.

## Suggested narrative if the result is negative

If the Hybrid policy actively hurts at multiple friction values, this is a
genuine sensitivity finding and should be reported as such. The paper has a
section ("Where residual RL hurts") that already supports this framing.

## Time estimate

5 friction values × 2 controllers × 10 trials × ~20 s per trial ≈ 35 min of
pure simulation time + ~30 min for plotting and table editing.

— Phillipp
