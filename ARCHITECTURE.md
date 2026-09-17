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

Tire contacts share a scratch contact-space impulse response for the chassis and
any common ground bodies. Relaxed synchronous block iterations replace each
wheel's accumulated tangent/brake impulses, so no wheel gets priority from its
insertion order. Effective inverse mass/inertia and the force application
Jacobians match the real bodies, including locked axes and wheel roll influence.
Only the final summed impulses are applied to each real body.
The iteration uses a diagonal body-response majorizer and an energy line search:
pure rolling demand does not seed canceling lateral forces that waste tire grip,
and weakly coupled wheel-inertia corrections can take a full step.

Drive torque and grip/assist history are immutable during a tick's previews.
ABS/TC and grip-recovery predictions are rechecked against the other contacts'
latest impulses; a bounded outer solve updates their limits. A contact velocity
residual controls convergence (including the wheel's angular response), not raw
impulse differences. If an iteration cap is reached, the solver retains its
feasible accumulated result for the accepted limits and records the remaining
residual in its scratch diagnostics. Wheel state and feedback commit once;
solver iterations never integrate torque or recovery time repeatedly.
Assist torque searches alternate bracketed interpolation with bisection,
preserving the original torque-fraction resolution and slip targets. Scalar
isotropic friction projections use their exact radial solution; wheels whose
angular momentum exceeds both available impulse budgets skip the impossible
held-wheel preview. These reduce preview cost without removing the coupled
contact or assist convergence checks.

Wheel geometry keeps the fixed suspension mount separate from the tire center.
In chassis coordinates, the steering pivot is `mount + direction * length`,
and the tire center is `pivot + steering_rotation * center_offset`. The cylinder
suspension sweep is centered on `mount + steering_rotation * center_offset` and follows
the suspension direction. Steering rotates the neutral offset and axle around
`steering_axis`; it does not rotate the suspension direction or include wheel
roll. With a zero offset, the tire stays on the original suspension line.

The game derives suspension direction and rest length from the authored Mount-to-Joint
displacement. The existing wheel Up Axis (`spin.upLocalAxis`) transformed by
Joint's orientation defines the kingpin; Spin's orientation defines the tire
alignment and rolling axle. Caster rotates the whole Mount/Joint/Spin assembly around
the wheel center; toe rotates the neutral offset and axle around the kingpin;
camber adjusts the axle at the tire center. Driving, replay, force feedback,
and capability calculations share these frames.

The suspension query sweeps a cylinder (authored radius and full axle width)
from maximum compression to full droop. It visits only shapes/triangles whose
bounds overlap that travel volume, then selects the earliest supporting hit.
Mesh and compound parts are considered separately so a rejected steep contact
cannot hide supporting ground in the same collider. Contact normals must oppose
travel with cosine at least 0.1; initial overlaps request penetration geometry.
Contacts inside planar faces use exact cylinder support distances and face normals;
triangle edge/corner contacts use a convex shape cast on a clipped polygon around
the wheel's entire travel volume. Clipping and recentering use f64 intermediates;
the localized query uses the engine's scalar type and does not change track assets.
An exact triangle-plane bound prevents premature hits. Convex witnesses must lie
on the triangle and within the cylinder at the reported travel; invalid casts are
recomputed by bounded conservative advancement. Exact face contacts win near-equal
ties so a shared coplanar edge cannot disturb flat support. This preserves the
circular approach to real kerbs without large-triangle GJK compression spikes.
The sweep witness and cylinder support determine the contact, including camber.
A flat contact patch uses its central support when that point lies on the surface.
There is one suspension/tire load per wheel, regardless of candidate count.

Spring compression and ground-relative damper velocity are measured along
suspension travel; their combined force is projected into the road-normal reaction.
The game supplies a finite force envelope from full-stroke spring force plus
damping at the speed with equivalent spring energy. Its floor can support the
whole chassis weight and configured maximum downforce on one contact. Applied
suspension, anti-roll transfer, and tire grip all respect this same force limit.
The scene query covers suspension travel, not forward motion between ticks;
narrow obstacles can still be skipped at sufficiently high speed.

Downforce uses one shared configuration with a positive curve exponent, a maximum
center-of-mass force, and optional chassis-local points with their own maximum
forces. The controller derives theoretical top speed from maximum engine RPM,
the last forward gear ratio, final drive, and the first wheel's current radius.
The shared scale is `min(abs(speed) / top_speed, 1)^exponent`; missing wheels or
invalid top-speed inputs produce no downforce. Point forces replace the
center-of-mass force when points are present. An exponent of one is linear and
two is quadratic; both reach and cap at maximum force at theoretical top speed.
The summed active downforce also generates longitudinal drag at the center of
mass, multiplied by the car's nonnegative `drag_per_downforce` ratio (default
0.2). This drag opposes forward or reverse motion, uses the same curve and cap,
and is additional to body drag and rolling resistance. Points replace the
center-of-mass fallback in both the load and drag calculation.

Axle differentials live in the same tire/brake contact solve. Driven front and
rear pairs use independent acceleration/coast clutch settings; positive
mechanical power selects acceleration (also in reverse), disconnected drive
selects coast, and a torque deadband holds the mode through momentary cuts.
Intermediate locking is a passive bounded angular impulse, equal and opposite
at the two wheels. Its torque capacity is `p/(1-p) * (50 Nm + |axle torque|/2)`;
`p=0` is open and `p=1` instead uses an exact shared rotational degree of freedom.
Thus full lock gives identical angular speeds under all contact/brake conditions,
including unequal wheel radii, without post-solve speed edits or fictitious grip.
A percentage controls clutch strength, not a fixed percentage of speed difference.
The angular response and tire/brake impulses converge together; the full-lock
constraint remains exact even if the bounded contact solve reaches its cap.

AWD center balance is the rear share of incoming torque, not a center speed lock.
The same weights define shaft speed, equivalent inertia, torque distribution,
and road-load feedback. Internal axle reactions are excluded from engine load.
TC chooses a common cut of the incoming torque; axle reaction stays internal.
ABS/TC previews use the coupled rotational response and allow the unavoidable
corner scrub of a locked or slipping clutch. Individual tire slip still drives
the existing friction, grip recovery and skid calculations. Airborne wheels
participate in axle/brake rotation but provide no road grip or assist sensing.
