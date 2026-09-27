# mobile-sensor

## Overview

Canonical repository: [rhymesg/mobile-sensor](https://github.com/rhymesg/mobile-sensor).

MATLAB research code for informative path planning: dispatching a mobile sensor into a circular operational area and selecting sensing trajectories using Fisher information and D-optimality.

The source collection relates to the [Information Fusion paper](#citation) on two-phase sensor dispatch, with waypoint geometry, information-gradient calculations, and GPOPS-II optimal-control experiments. Start with the [algorithm reference](docs/algorithm.md) for equations, source entry points, and translation guidance for Python or C++ implementations.

For reusable MATLAB measurement models, Fisher information, Cramér–Rao bounds, and log-determinant information costs, see [information-based-tracking](https://github.com/rhymesg/information-based-tracking). Its [research applications](https://github.com/rhymesg/information-based-tracking/blob/main/docs/papers.md#informative-mobile-sensor-dispatch) map these calculations to the paper; the toolkit does not include this repository's full dispatch planner.

*Information Fusion* ranked **#1 in Signal Processing by citations per document (2 years) in 2018**, according to [SCImago Journal & Country Rank](https://www.scimagojr.com/journalrank.php?category=1711&year=2018&order=cpd&ord=desc&type=j).

## Installation

Open this directory as MATLAB's current folder. The geometry example uses base MATLAB, with no external data or solver.

The optimization scripts require a separately installed [GPOPS-II distribution with SNOPT configured](https://www.gpops2.com/resources/gpops2UsersGuide.pdf). Exact original software versions are unavailable; MATLAB and Octave execution have not been verified.

Full dispatch experiments require missing MAT files and helper functions and contain implementation inconsistencies. Read [running and restoration notes](docs/running.md) before attempting those scripts.

## Usage

Run the deterministic geometry and matrix-packing example from the repository root:

```bash
matlab -batch "example_reference"
```

Expected values are radius `5`, angle `0.927295218002` radians, and normalized RSL length `4`. The example checks those values and both matrix-packing modes, then prints `Reference example passed.`

Call the geometry functions directly in MATLAB:

```matlab
[radius, angle] = Carte2Polar([20, 0], [23, 4]);
point = Polar2Carte([20, 0], radius, angle);
normalized_length = Dubins_RSL_length(0, 0, 4);
```

This example exercises reusable geometry helpers; it does not reproduce the paper's trajectory or timing results. [Running notes](docs/running.md) explain the original experiment entry points and expected artifacts.

## Development

The example contains numerical assertions for basic helper contracts. There is no automated validation of the complete planner; the original [test.m](test.m) is an exploratory Riccati calculation without assertions.

For a useful issue report, provide the source revision or archive identifier, MATLAB and solver versions, script name, parameter changes, required input files, and observed output. Report problems through [GitHub Issues](https://github.com/rhymesg/mobile-sensor/issues).

## Find a method

| Task | Entry points | Reference |
|---|---|---|
| Choose a waypoint and estimate remaining travel time | [mission_plan.m](mission_plan.m), [Dubins_RSL_length.m](Dubins_RSL_length.m) | Paper Section 3.3 |
| Inspect an information-rate gradient | [control_grad2d.m](control_grad2d.m) | Paper Section 3.1; [implementation differences](docs/algorithm.md#implementation-differences) |
| Inspect a two-phase optimal-control formulation | [main_gpops_multitarget.m](main_gpops_multitarget.m), [mobileSensorMultiContinuous.m](mobileSensorMultiContinuous.m), [mobileSensorMultiEndpoint.m](mobileSensorMultiEndpoint.m) | Paper Sections 2.3 and 4 |
| Inspect dispatch comparisons and entry-angle sweeps | [realtime_dispatch_multitarget.m](realtime_dispatch_multitarget.m) | [Experiment dependencies](docs/running.md#experiment-dependencies) |
| Translate coordinates and matrix storage | [Carte2Polar.m](Carte2Polar.m), [Polar2Carte.m](Polar2Carte.m), [mat2vec.m](mat2vec.m), [vec2mat.m](vec2mat.m) | [Cross-language contracts](docs/algorithm.md#cross-language-contracts) |

## Citation

Please cite the paper when using these research methods:

> Youngjoo Kim, Wooyoung Jung, and Hyochoong Bang. “Real-time path planning to dispatch a mobile sensor into an operational area.” *Information Fusion*, 45, 27–37, 2019. [doi:10.1016/j.inffus.2018.01.010](https://doi.org/10.1016/j.inffus.2018.01.010).

The article appeared online in January 2018 and in the January 2019 volume. [CITATION.cff](CITATION.cff) supplies the preferred paper citation; [source provenance](docs/source-provenance.md) identifies the supplied materials and the scope of this collection.

## License

No software license was included with the source archive, and none has been selected for this repository. Reuse and redistribution permissions need clarification from the rights holder; the citation request does not grant those permissions.
