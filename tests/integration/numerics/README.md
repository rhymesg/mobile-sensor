# Numerical regression checks

Synthetic callback inputs check sensing-region gates and all three target information blocks. A centered finite difference independently checks the two gradient outputs.

From the repository root, with base MATLAB:

```bash
matlab -batch "addpath('tests/integration/numerics'); verify_numerics"
```

These checks require no external data or plotting and cover the numerical routines described above.
