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

Wheel geometry keeps the fixed suspension mount separate from the tire center.
In chassis coordinates, the steering pivot is `mount + direction * length`,
and the tire center is `pivot + steering_rotation * center_offset`. The single
suspension ray starts at `mount + steering_rotation * center_offset` and follows
the suspension direction. Steering rotates the neutral offset and axle around
`steering_axis`; it does not rotate the suspension direction or include wheel
roll. With a zero offset, the ray stays on the original suspension line.

The game derives suspension direction and rest length from the authored Mount-to-Joint
displacement. The existing wheel Up Axis (`spin.upLocalAxis`) transformed by
Joint's orientation defines the kingpin; Spin's orientation defines the tire
alignment and rolling axle. Caster rotates the whole Mount/Joint/Spin assembly around
the wheel center; toe rotates the neutral offset and axle around the kingpin;
camber adjusts the axle at the tire center. Driving, replay, force feedback,
and capability calculations share these frames.

The ray supplies a local road plane. Tire contact is solved using a circular
wheel cross-section perpendicular to the steered axle. Its support radius along
the plane normal and the suspension/normal angle determine spring length.
The resulting tire support point supplies tire and suspension contact forces;
the ray hit itself is not generally that contact point for tilted travel.
Spring compression and damper velocity are measured along suspension travel;
their combined force is projected into the supporting road-normal reaction.
Ray reach covers full droop and the radius projection for supported incidence
angles (normal opposite travel, cosine at least 0.1). Hits beyond reachable
travel or at near-parallel incidence do not create wheel contact. This remains
a single-ray approximation; it does not resolve tire width or suspension-arm arcs.

Downforce uses one shared configuration with a positive curve exponent, a maximum
center-of-mass force, and optional chassis-local points with their own maximum
forces. The controller derives theoretical top speed from maximum engine RPM,
the last forward gear ratio, final drive, and the first wheel's current radius.
The shared scale is `min(abs(speed) / top_speed, 1)^exponent`; missing wheels or
invalid top-speed inputs produce no downforce. Point forces replace the
center-of-mass force when points are present. An exponent of one is linear and
two is quadratic; both reach and cap at maximum force at theoretical top speed.
