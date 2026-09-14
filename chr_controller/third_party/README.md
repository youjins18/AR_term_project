# Generated dynamics boundary

`dynamics_gen_chr.py` is an optional SymPy reference generator for a nominal
reduced-order 3R model. It is not used by the safe default build because the
actual coupled Palletrone-arm dynamics and mount parameters are not yet identified.

The build creates and installs `libchr_dynamics.so` from the validated kinematic
library. A future generated rigid-body or learning model should preserve that
library boundary, or add a policy adapter that consumes `ChrState` and produces a
bounded `ChrReference`. Generated binaries must be rebuilt locally rather than
committed as architecture-dependent `.so` files.
