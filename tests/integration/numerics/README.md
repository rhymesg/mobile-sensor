# Numerical regression checks

Synthetic callback inputs check sensing-region gates and all three target information blocks. A centered finite difference independently checks the two gradient outputs.

From the repository root, with base MATLAB:

```bash
matlab -batch "addpath('tests/integration/numerics'); verify_numerics"
```

These checks require no external data or plotting. They have been syntax checked, but have not been executed in MATLAB or Octave. They do not validate the complete research experiment.
