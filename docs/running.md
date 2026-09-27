# Running and restoring the experiments

This guide distinguishes the standalone helper example from research experiments requiring external solvers, missing artifacts, or code repair.

## Standalone example

From the repository root, run:

```bash
matlab -batch "example_reference"
```

The example uses base MATLAB and deterministic synthetic inputs; it requires no toolbox, dataset, solver, or random seed. For installations without the [batch startup option](https://www.mathworks.com/help/matlab/matlab_env/commonly-used-startup-options.html), invoke `example_reference` in MATLAB's Command Window.

| Check | Derived expectation | Tolerance |
|---|---|---|
| Point `[23,4]` about origin `[20,0]` | Radius `5`, angle `atan2(4,3)` ≈ `0.927295218002` | Absolute error `<1e-12` |
| Coordinate round trip | Reconstructed `[23,4]` | Euclidean error `<1e-12` |
| RSL with `a=b=0`, `d=4` | Normalized length `4` | Absolute error `<1e-12` |
| Pack `[1,2,3;4,5,6]` | Row `[1,2,3,4,5,6]`; column `[1;4;2;5;3;6]` | Exact equality |

Expected final message: `Reference example passed.` The RSL helper also prints its `p`, `t`, and `L` intermediates because its assignments have no semicolons.

These are analytical helper fixtures, not measured paper results. MATLAB, Octave, GPOPS-II, and SNOPT execution have not been verified in this environment.

## Experiment dependencies

| Entry point | Dependencies or input | Intended output |
|---|---|---|
| [mission_plan.m](../mission_plan.m) | Included coordinate and RSL helpers | Waypoint, boundary points, remaining path length and travel time |
| [main_gpops_simple.m](../main_gpops_simple.m) | GPOPS-II, SNOPT, simple continuous/endpoint callbacks | `output` workspace structure and trajectory plot |
| [main_gpops_multitarget.m](../main_gpops_multitarget.m) | GPOPS-II, SNOPT, multi continuous/endpoint callbacks | `result.mat` containing `output`, trajectory plot |
| [realtime_dispatch.m](../realtime_dispatch.m) | Missing `opt_sol_dispatch.mat`, `control_grad.m`, `satur.m` | Trajectory comparison and information costs |
| [realtime_dispatch_multitarget.m](../realtime_dispatch_multitarget.m) | Missing `result_ms2_opt.mat` and `result_ms2_opt_err.mat`, each containing `output` | Information-cost histories, entry-angle sweep, trajectory comparison |

The GPOPS drivers select `snopt`, `sparseCD`, `RPMintegration`, and an `hp1` mesh. Configure the solver using the [GPOPS-II user guide](https://www.gpops2.com/resources/gpops2UsersGuide.pdf); no solver binaries or licenses are included.

The default multitarget driver writes `result.mat`, not either filename loaded by the dispatch comparison. Its callback models one fixed sensor, while the comparison script models three fixed sensors; simply renaming its output does not restore the missing experiments.

## Input contracts

- `opt_sol_dispatch.mat` must provide `x_res`, `y_res`, `xt_res`, and `yt_res` trajectories, with at least 100 indexable samples and an entry into the configured circle; the original generation procedure is absent.
- The multitarget MAT files must provide `output.result.solution.phase(1:2).time`, corresponding `.state` arrays, and `output.result.setup.auxdata.multisensor`.
- The comparison reads the first two state columns as sensor position; source scenario settings, target trajectories, noise, and sample counts must agree with the restored files.
- GPOPS solver nodes are reused with uniform `dt` approximations in the comparison. Retain that fact when checking numerical equivalence; a resampling change requires separate validation.
- Default target errors in `main_gpops_multitarget.m` are zero. Commented `randn` experiments have no recorded seed; no Monte Carlo dataset or expected aggregate results are supplied.

## Known execution and interpretation issues

- `mobileSensorSimpleContinuous.m` calls `norm(dx,dy)`: MATLAB treats the second argument as a norm order, not a second coordinate ([MathWorks reference](https://www.mathworks.com/help/matlab/ref/norm.html)). This does not calculate the intended circular distance and may error.
- `mobileSensorMultiEndpoint.m` declares its function as `mobileSensorSimpleEndpoint`; the filename and declaration disagree.
- The multitarget setup exposes `L` but hard-codes three targets in derivatives and the terminal determinant product. Changing `L` alone is unsupported.
- The entry-angle sweep evaluates 15 angles at increments of one hundredth of its preset interval; it does not search the complete interval or implement the paper's online gradient-descent iteration.
- `main_gpops_simple_backup.m` contains different state guesses, a 15-element event bound against a 16-element callback event, and phase-2 plotting times read from phase 1. It is an earlier experiment, not the recommended driver.
- [Algorithm differences](algorithm.md#implementation-differences) cover gradient weighting, derivatives, objective scaling, and information-state dimensions. Establish the intended scientific behavior before repairing them.

Scripts containing `clear`, `clear all`, or `close all` overwrite the current experiment workspace or figures; run them in a fresh session. The `graph_GD*` and `graph_SGD` scripts plot preset timing expressions; their constants are not newly measured benchmarks.
