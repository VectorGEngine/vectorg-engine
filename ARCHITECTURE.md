# Repository architecture

VectorG Engine shares one implementation across its 2D, 3D, `f32`, and `f64` variants.

- `crates/`: package manifests for the engine variants, testbeds, URDF loader, and mesh loader.
- `src/`: shared engine implementation.
- `src_testbed/`: shared testbed implementation used by the examples and benchmarks.
- `examples2d/`: 2D example scenes. Run with `cargo run --release --bin all_examples2`.
- `examples3d/`: 3D and vehicle-dynamics example scenes. Run with `cargo run --release --bin all_examples3`.
- `benchmarks2d/`: 2D stress tests.
- `benchmarks3d/`: 3D and vehicle-dynamics stress tests.

Cargo features in each package select the dimension and scalar type while keeping the simulation implementation shared.
