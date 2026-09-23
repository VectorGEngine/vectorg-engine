//! The suspension is cast from the chassis pose at the start of a step, and the
//! solver then moves the chassis. A renderer placing the tire after the step, from
//! that same length, is therefore working from a pose the chassis has already left.
//! On a landing the error is large enough to bury the tire in the road, so the end
//! of the step measures every wheel again before anything reads it.
use vectorg_engine::control::*;
use vectorg_engine::prelude::*;

const RADIUS: Real = 0.335;
const REST: Real = 0.30;
const MOUNTS: [(Real, Real, WheelAxle); 4] = [
    (-0.8, -1.3, WheelAxle::Rear),
    (0.8, -1.3, WheelAxle::Rear),
    (-0.8, 1.3, WheelAxle::Front),
    (0.8, 1.3, WheelAxle::Front),
];

struct Rig {
    bodies: RigidBodySet,
    colliders: ColliderSet,
    vehicle: DynamicRayCastVehicleController,
    chassis: RigidBodyHandle,
    pipeline: PhysicsPipeline,
    islands: IslandManager,
    broad: BroadPhaseMultiSap,
    narrow: NarrowPhase,
    joints: ImpulseJointSet,
    multis: MultibodyJointSet,
    ccd: CCDSolver,
    queries: QueryPipeline,
    params: IntegrationParameters,
}

impl Rig {
    /// A car dropped nose up onto flat trimesh ground, as a jump lands.
    fn new(pitch_deg: Real, fall: Real) -> Self {
        let mut bodies = RigidBodySet::new();
        let mut colliders = ColliderSet::new();
        let verts = vec![
            Point::new(-60.0, 0.0, -60.0),
            Point::new(60.0, 0.0, -60.0),
            Point::new(60.0, 0.0, 60.0),
            Point::new(-60.0, 0.0, 60.0),
        ];
        colliders.insert(ColliderBuilder::trimesh(verts, vec![[0u32, 1, 2], [0, 2, 3]]).unwrap());
        let chassis = bodies.insert(
            RigidBodyBuilder::dynamic()
                .position(Isometry::from_parts(
                    Translation::from(Vector::new(0.0, RADIUS + REST + fall, 0.0)),
                    Rotation::new(Vector::x() * pitch_deg.to_radians()),
                ))
                .can_sleep(false),
        );
        colliders.insert_with_parent(
            ColliderBuilder::cuboid(0.8, 0.15, 1.5).mass(1059.0),
            chassis,
            &mut bodies,
        );
        bodies[chassis].recompute_mass_properties_from_colliders(&colliders);
        let mut config = VehicleControllerConfig::default();
        config.transmission.automatic = false;
        let mut vehicle = DynamicRayCastVehicleController::new(chassis, config);
        vehicle.index_forward_axis = 2;
        for (x, z, axle) in MOUNTS {
            let wheel = vehicle.add_wheel(
                Point::new(x, 0.0, z),
                -Vector::y(),
                Vector::x(),
                REST,
                RADIUS,
                0.2,
                &WheelTuning {
                    suspension_stiffness: 90.0,
                    suspension_compression: 3.0,
                    suspension_damping: 3.0,
                    friction_slip: 1.0,
                    max_suspension_force: 37_635.0,
                    ..Default::default()
                },
                WheelRole::new(axle, axle == WheelAxle::Rear, false),
            );
            wheel.suspension_bump_travel = REST;
        }
        let mut queries = QueryPipeline::new();
        queries.update(&colliders);
        Self {
            bodies,
            colliders,
            vehicle,
            chassis,
            pipeline: PhysicsPipeline::new(),
            islands: IslandManager::new(),
            broad: BroadPhaseMultiSap::new(),
            narrow: NarrowPhase::new(),
            joints: ImpulseJointSet::new(),
            multis: MultibodyJointSet::new(),
            ccd: CCDSolver::new(),
            queries,
            params: IntegrationParameters {
                dt: 1.0 / 60.0,
                num_solver_iterations: std::num::NonZeroUsize::new(4).unwrap(),
                ..Default::default()
            },
        }
    }

    /// One frame as the game runs it: cast, step, then hand the wheels to the renderer.
    fn frame(&mut self, refresh: bool) {
        let filter = QueryFilter::default().exclude_rigid_body(self.chassis);
        self.vehicle.update_vehicle(
            self.params.dt,
            &(-Vector::y() * 9.81),
            &mut self.bodies,
            &self.colliders,
            &self.queries,
            filter,
        );
        self.pipeline.step(
            &(-Vector::y() * 9.81),
            &self.params,
            &mut self.islands,
            &mut self.broad,
            &mut self.narrow,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.joints,
            &mut self.multis,
            &mut self.ccd,
            None,
            &(),
            &(),
        );
        self.queries.update(&self.colliders);
        if refresh {
            let filter = QueryFilter::default().exclude_rigid_body(self.chassis);
            self.vehicle.finish_vehicle_update(
                &mut self.bodies,
                &self.colliders,
                &self.queries,
                filter,
            );
        }
    }

    /// Where the renderer puts the tire: the suspension length the controller is
    /// holding, rebuilt against the chassis pose as it now stands.
    fn rendered_wheel_height(&self, wheel_id: usize) -> Real {
        let pose = *self.bodies[self.chassis].position();
        let (x, z, _) = MOUNTS[wheel_id];
        let mount = pose * Point::new(x, 0.0, z);
        let direction = pose * -Vector::y();
        let length = self.vehicle.wheels()[wheel_id]
            .raycast_info()
            .suspension_length;
        (mount + direction * length).y
    }

    fn in_contact(&self, wheel_id: usize) -> bool {
        self.vehicle.wheels()[wheel_id].raycast_info().is_in_contact
    }
}

/// Deepest the rendered tire goes below the road over a landing, and how many frames
/// it actually spent on the ground, so a quiet result cannot pass for a clean one.
fn worst_rendered_sink(pitch_deg: Real, fall: Real, refresh: bool) -> (Real, usize) {
    let mut rig = Rig::new(pitch_deg, fall);
    let (mut worst, mut contact_frames) = (0.0, 0);
    for _ in 0..240 {
        rig.frame(refresh);
        if rig.in_contact(0) {
            contact_frames += 1;
            let sink = RADIUS - rig.rendered_wheel_height(0);
            if sink > worst {
                worst = sink;
            }
        }
    }
    (worst, contact_frames)
}

#[test]
fn a_stale_suspension_length_buries_the_rendered_tire() {
    // The bug itself, so the fix below is measured against something real rather than
    // against nothing. Every case lands: the contact count proves it.
    for (pitch, fall, least) in [(0.0, 0.3, 0.02), (10.0, 1.0, 0.05), (30.0, 2.5, 0.08)] {
        let (sink, frames) = worst_rendered_sink(pitch, fall, false);
        assert!(frames > 50, "the car must land: {frames} frames of contact");
        assert!(
            sink > least,
            "pitch {pitch} fall {fall}: expected a stale length to bury the tire deeper \
             than {least} m, got {sink}"
        );
    }
}

#[test]
fn refreshing_contacts_keeps_the_rendered_tire_on_the_road() {
    // Re-measuring after the step puts the length and the pose back in the same
    // instant, which is all a renderer ever needed.
    for pitch in [0.0, 10.0, 20.0, 30.0, 45.0] {
        for fall in [0.3, 1.0, 2.5] {
            let (sink, frames) = worst_rendered_sink(pitch, fall, true);
            assert!(frames > 50, "the car must land: {frames} frames of contact");
            assert!(
                sink <= 1.0e-4,
                "pitch {pitch} fall {fall}: tire still {sink} m into the road after refreshing"
            );
        }
    }
}

#[test]
fn refreshing_contacts_does_not_change_the_simulation() {
    // It applies no force and touches no velocity, so the car must follow exactly the
    // same path whether or not a renderer asks for fresh wheels.
    for (pitch, fall) in [(0.0, 0.3), (30.0, 2.5)] {
        let mut plain = Rig::new(pitch, fall);
        let mut refreshed = Rig::new(pitch, fall);
        for step in 0..240 {
            plain.frame(false);
            refreshed.frame(true);
            let a = *plain.bodies[plain.chassis].position();
            let b = *refreshed.bodies[refreshed.chassis].position();
            assert!(
                (a.translation.vector - b.translation.vector).norm() < 1.0e-6,
                "step {step}: chassis position drifted"
            );
            assert!(
                (a.rotation.coords - b.rotation.coords).norm() < 1.0e-6,
                "step {step}: chassis rotation drifted"
            );
            let va = *plain.bodies[plain.chassis].linvel();
            let vb = *refreshed.bodies[refreshed.chassis].linvel();
            assert!((va - vb).norm() < 1.0e-6, "step {step}: velocity drifted");
        }
    }
}
