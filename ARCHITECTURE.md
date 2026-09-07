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

Raycast vehicles integrate chassis gravity in `update_vehicle`, before applying
suspension and tire impulses. Pass the world gravity vector and the same timestep
used by the following physics step. Automatic chassis gravity is temporarily
disabled to avoid applying it twice; `finish_vehicle_update` restores its original
scale after stepping (`World.step` does this in the JavaScript bindings). Reset or
removal must call `cancel_vehicle_update` before discarding the controller.

This keeps the general rigid-body solver unchanged. Vehicle gravity uses one
impulse per vehicle tick; other external forces and collision responses continue
to use the world's normal substeps. Complete the update/step/finish sequence before
changing the chassis gravity scale or taking a physics snapshot.
