# Informative sensor dispatch

This reference connects the [paper cited in the README](../README.md#citation) to the MATLAB source and documents contracts needed for reuse or translation.

## Published method

- A mobile sensor starts outside a circular sensing region; its measurements become available after entry.
- Phase 1 selects an entry point and a waypoint, while optimizing the entry angle during travel.
- Phase 2 steers inside the region using the spatial gradient of an information-rate cost.
- The paper compares this strategy with nonlinear-programming trajectories, accounting for planning delay and uncertainty in predicted target states.

The paper's information dynamics and D-optimal cost are

$$\dot J=-JF^T-FJ-JQJ+\mathcal I H^T R^{-1}H,\qquad c=-\tfrac12\ln\det J.$$

Here $J$ is the Fisher information matrix, $F$ the linearized target dynamics, $Q$ process-noise covariance, $H$ the measurement Jacobian, $R$ measurement-noise covariance, and $\mathcal I$ indicates measurement availability (paper Eqs. 6–7).

For position-dependent measurement gain $\Psi=H^T R^{-1}H$, the gradient component is $\partial\dot c/\partial x=-\tfrac12\operatorname{tr}(J^{-1}\partial\Psi/\partial x)$, with an analogous expression for $y$ (Eqs. 14–15).

## Paper-to-source map

| Paper component | Source | Implemented scope |
|---|---|---|
| Information dynamics, Eqs. 6–7 | [realtime_dispatch_multitarget.m](../realtime_dispatch_multitarget.m) | Four-state information propagation and cost histories |
| Two-phase optimal control, Section 2.3 | [main_gpops_multitarget.m](../main_gpops_multitarget.m), [continuous](../mobileSensorMultiContinuous.m), [endpoint](../mobileSensorMultiEndpoint.m) | Three-target GPOPS-II setup with three-by-three information matrices |
| Information-rate gradient, Section 3.1 | [control_grad2d.m](../control_grad2d.m) | Two scalar derivatives, with differences listed below |
| Entry-angle optimization, Section 3.2 | [realtime_dispatch_multitarget.m](../realtime_dispatch_multitarget.m) | Fixed entry-angle sweep; no complete online update from Eq. 23 |
| Waypoint construction, Section 3.3, Eqs. 19, 30–32 | [mission_plan.m](../mission_plan.m), [Dubins_RSL_length.m](../Dubins_RSL_length.m) | Preset boundary angles and normalized right-straight-left path length |

The separate [gradient_descent.m](../gradient_descent.m) minimizes a scalar quadratic; it does not optimize the sensor entry angle.

## Geometry procedure

For speed $v$, maximum heading rate $\dot\psi_{max}$, region radius $R_C$, and selected angular half-width $\theta_h$:

1. Set turning radius $\rho=v/\dot\psi_{max}$ and waypoint radius $R_w=R_C+4\rho$.
2. Set waypoint angle to the midpoint of the selected boundary-angle interval.
3. Compute $D=\sqrt{(R_w-R_C\cos\theta_h)^2+(R_C\sin\theta_h)^2}$ and normalized distance $d=D/\rho$.
4. Compute $a=\operatorname{atan2}(R_C\sin\theta_h,R_w-R_C\cos\theta_h)$ and $b=a+\theta_h$.
5. Evaluate `Dubins_RSL_length(a,b,d)`, multiply by $\rho$, and divide by $v$ for travel time.

This follows [mission_plan.m](../mission_plan.m). The RSL helper implements one path family under the paper's geometry restrictions, not a general shortest-Dubins-path solver; infeasible inputs can make its square root complex.

## Cross-language contracts

| Interface | Input and output |
|---|---|
| `Carte2Polar(X0,X1)` | Two real `1-by-2` points; returns nonnegative distance and `atan2(dy,dx)` angle in radians |
| `Polar2Carte(X0,R,the)` | Origin, scalar radius, scalar angle; returns a `1-by-2` point |
| `Dubins_RSL_length(a,b,d)` | Scalar radians and distance normalized by turning radius; returns normalized length and prints intermediate values |
| `mat2vec(A,'row')` | Concatenates rows into a row vector |
| `mat2vec(A,'col')` | Concatenates columns into a column vector |
| `vec2mat(v,r,c,mode)` | Inverse packing for exactly `r*c` entries; both helpers fall back to row mode for other mode strings |
| `control_grad2d(p_sen,p_tar,R,J)` | Three-coordinate positions, `3-by-3` positive-definite measurement covariance matrix, information matrix with an invertible leading `2-by-2` block; returns separate scalar outputs `[ux,uy]` |

- Geometry uses metres, seconds, radians, and counterclockwise headings from the positive x-axis; positions use an east/north/up convention.
- Multitarget GPOPS states are `[sensor_x,sensor_y,heading,target_xy_pairs,row_packed_J1,J2,J3]`; with three targets each row has 36 elements.
- The single-target setup instead uses `[target_x,target_y,sensor_x,sensor_y,heading,row_packed_J]`, with 14 elements.
- Matrix multiplication, elementwise operations, and solves must retain their MATLAB meanings; `J\dPsi` solves a linear system. Use explicit packing order when translating to Python or C++.
- Preserve update order when comparing with the dispatch script: update information before evaluating the gradient, then advance position and heading with forward Euler steps.
- Use [example_reference.m](../example_reference.m) as deterministic geometry and non-square matrix-packing fixtures. These fixtures do not validate a translated planner.

## Implementation differences

- The GPOPS callbacks accumulate measurement information only; they omit the full dissipation terms and use `3-by-3` matrices, whereas the dispatch comparison uses `4-by-4` position/velocity information matrices.
- Endpoint objectives use `log2`; the paper and dispatch comparison use natural logarithms. The single-target endpoint also omits the paper's one-half factor.
- `control_grad2d` now uses inverse measurement covariance via linear solves and the corrected derivative `dHdy(1,2)=2*x*y/(x^2+y^2)^2`. The GPOPS callbacks use the corrected elevation derivative `rho/(rho^2+u^2)`.
- The helper evaluates a leading position block of `J`, not the full four-state inverse. Zero horizontal separation and singular information blocks are unguarded.
- The dispatch caller now accumulates both gradient outputs separately. Its heading expression remains `-atan2(grad_sum(2),grad_sum(1))`; this is not generally the heading of the negative gradient. Resolve the intended steering convention before using the full planner.

[Running notes](running.md) specify inputs and solver dependencies for the original experiments.

[Regression checks](../tests/integration/numerics/README.md) exercise callback information, sensing gates, and gradient finite differences without GPOPS-II. Use the fixtures to compare information calculations and gradients during adaptation.
