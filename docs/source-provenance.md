# Source provenance

This document identifies the research materials associated with mobile-sensor and the scope of the distributed source collection.

## Materials

- The supplied `mobile_sensor-20260927T100536Z-1-001.zip` archive contains 21 MATLAB files and two historical SNOPT logs.
- Archive SHA-256: `fa8c4cc6478a2fc9dfa3872c1784c32a3bca5d81570f37f3e2900e44685e9b58`.
- The MATLAB files retain the archive's flat layout. Numerical corrections in `control_grad2d`, both continuous callbacks, `mobileSensorMultiEndpoint`, and `realtime_dispatch_multitarget` are described in the [implementation reference](algorithm.md#implementation-differences) and covered by [regression checks](../tests/integration/numerics/README.md).
- `example_reference.m` is an added deterministic geometry and packing example.
- The supplied article is “Real-time path planning to dispatch a mobile sensor into an operational area”; its canonical bibliography is the [README citation](../README.md#citation).
- Paper metadata agrees with the [KAIST publication record](https://pure.kaist.ac.kr/en/publications/real-time-path-planning-to-dispatch-a-mobile-sensor-into-an-opera/): the volume is dated 2019, although the article was available online in 2018.

The source filenames and implemented formulations connect this collection to the paper, but the archive has no authorship statement or version history. Until authorship is confirmed, this repository does not label the collection an official or complete reference implementation; software attribution in [CITATION.cff](../CITATION.cff) is collective.

## Distribution boundary

The paper, source archive, and original logs remain local reference materials under ignored `ref/`. They are not republished or relicensed by this project.

See [running notes](running.md) for solver and input contracts, and [license status](../README.md#license) for reuse terms.
