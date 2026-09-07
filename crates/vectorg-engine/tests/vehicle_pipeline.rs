use vectorg_engine::control::*;
use vectorg_engine::prelude::*;

struct Scene {
    bodies: RigidBodySet,
    colliders: ColliderSet,
    vehicle: DynamicRayCastVehicleController,
    pipeline: PhysicsPipeline,
    islands: IslandManager,
    broad: BroadPhaseMultiSap,
    narrow: NarrowPhase,
    joints: ImpulseJointSet,
    multis: MultibodyJointSet,
    ccd: CCDSolver,
    queries: QueryPipeline,
    params: IntegrationParameters,
    normal: Vector<f32>,
}

impl Scene {
    fn new(hz: u32, iterations: usize, slope: f32, heading: f32) -> Self {
        let tilt = Rotation::new(Vector::x() * slope.to_radians());
        let rotation = tilt * Rotation::new(Vector::y() * heading.to_radians());
        let normal = tilt * Vector::y();
        let mut bodies = RigidBodySet::new();
        let mut colliders = ColliderSet::new();
        colliders.insert(ColliderBuilder::halfspace(UnitVector::new_normalize(
            normal,
        )));
        let chassis = bodies.insert(
            RigidBodyBuilder::dynamic()
                .position(Isometry::from_parts(
                    Translation::from(normal * 0.65),
                    rotation,
                ))
                .can_sleep(false),
        );
        colliders.insert_with_parent(
            ColliderBuilder::cuboid(0.8, 0.15, 1.5).mass(1200.0),
            chassis,
            &mut bodies,
        );
        bodies[chassis].recompute_mass_properties_from_colliders(&colliders);
        let mut config = VehicleControllerConfig::default();
        config.transmission.automatic = false;
        config.transmission.auto_reverse = false;
        config.dynamics.esc_strength = 0.0;
        let mut vehicle = DynamicRayCastVehicleController::new(chassis, config);
        vehicle.index_forward_axis = 2;
        vehicle.set_input(VehicleInput {
            brake: 1.0,
            clutch: 1.0,
            ..Default::default()
        });
        for (x, z, axle) in [
            (-0.8, 1.3, WheelAxle::Front),
            (0.8, 1.3, WheelAxle::Front),
            (-0.8, -1.3, WheelAxle::Rear),
            (0.8, -1.3, WheelAxle::Rear),
        ] {
            let wheel = vehicle.add_wheel(
                Point::new(x, 0.0, z),
                -Vector::y(),
                Vector::x(),
                0.4,
                0.35,
                &WheelTuning {
                    suspension_stiffness: 30.0,
                    suspension_compression: 3.0,
                    suspension_damping: 3.0,
                    friction_slip: 1.0,
                    ..Default::default()
                },
                WheelRole::new(axle, axle == WheelAxle::Rear, false),
            );
            wheel.max_brake_force = 180_000.0;
            wheel.anti_lock_brake = 0.0;
        }
        let mut queries = QueryPipeline::new();
        queries.update(&colliders);
        Self {
            bodies,
            colliders,
            vehicle,
            queries,
            normal,
            pipeline: PhysicsPipeline::new(),
            islands: IslandManager::new(),
            broad: BroadPhaseMultiSap::new(),
            narrow: NarrowPhase::new(),
            joints: ImpulseJointSet::new(),
            multis: MultibodyJointSet::new(),
            ccd: CCDSolver::new(),
            params: IntegrationParameters {
                dt: 1.0 / hz as f32,
                num_solver_iterations: std::num::NonZeroUsize::new(iterations).unwrap(),
                ..Default::default()
            },
        }
    }

    fn tick(&mut self) {
        self.vehicle.update_vehicle(
            self.params.dt,
            &(-Vector::y() * 9.81),
            &mut self.bodies,
            &self.colliders,
            &self.queries,
            QueryFilter::default().exclude_rigid_body(self.vehicle.chassis),
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
            Some(&mut self.queries),
            &(),
            &(),
        );
        self.vehicle.finish_vehicle_update(&mut self.bodies);
    }

    fn position(&self) -> Vector<f32> {
        *self.bodies[self.vehicle.chassis].translation()
    }
    fn tangent(&self, v: Vector<f32>) -> Vector<f32> {
        v - self.normal * v.dot(&self.normal)
    }
}

#[test]
fn brakes_hold_hills_through_force_and_position_integration() {
    for hz in [30, 60, 120] {
        for iterations in [1, 4, 8] {
            for heading in [0.0, 45.0, 90.0, 180.0] {
                let mut scene = Scene::new(hz, iterations, 10.0, heading);
                for _ in 0..hz * 5 {
                    scene.tick();
                }
                let start = scene.position();
                for _ in 0..hz * 10 {
                    scene.tick();
                }
                let drift = scene.tangent(scene.position() - start).norm();
                // Allow f32 suspension/rotation roundoff at very small substeps.
                assert!(drift < 0.002, "hz={hz} iterations={iterations} heading={heading} drift={drift} delta={:?} velocity={:?}", scene.position()-start, (scene.bodies[scene.vehicle.chassis].linvel(), scene.bodies[scene.vehicle.chassis].angvel()));
                if hz == 60 && iterations == 4 {
                    assert!(drift < 0.0005, "game timestep drift={drift}");
                }
                assert!(scene
                    .vehicle
                    .wheels()
                    .iter()
                    .all(|w| w.delta_rotation == 0.0));
            }
        }
    }
}

#[test]
fn hill_holding_releases_with_brakes_and_respects_available_grip() {
    for release_brakes in [false, true] {
        let mut scene = Scene::new(60, 4, 15.0, 0.0);
        for _ in 0..300 {
            scene.tick();
        }
        let start = scene.position();
        if release_brakes {
            scene.vehicle.set_input(VehicleInput {
                clutch: 1.0,
                ..Default::default()
            });
        } else {
            for wheel in scene.vehicle.wheels_mut() {
                wheel.friction_slip = 0.01;
            }
        }
        for _ in 0..120 {
            scene.tick();
        }
        assert!(scene.tangent(scene.position() - start).norm() > 0.5);
    }
}

#[test]
fn airborne_vehicle_receives_gravity_once() {
    let mut scene = Scene::new(60, 4, 0.0, 0.0);
    scene.bodies[scene.vehicle.chassis].set_translation(Vector::y() * 100.0, true);
    for _ in 0..60 {
        scene.tick();
    }
    let velocity = scene.bodies[scene.vehicle.chassis].linvel().y;
    assert!((velocity + 9.81).abs() < 0.15, "velocity={velocity}");
    assert!(scene.position().y < 96.0 && scene.position().y > 94.0);
}

#[test]
fn vehicle_preserves_custom_gravity_scale_and_zero_timestep() {
    for scale in [0.0, 0.3, 1.0, 2.0, -1.0] {
        let mut scene = Scene::new(60, 4, 0.0, 0.0);
        let handle = scene.vehicle.chassis;
        scene.bodies[handle].set_translation(Vector::y() * 100.0, true);
        scene.bodies[handle].set_gravity_scale(scale, false);
        scene.vehicle.update_vehicle(
            0.0,
            &(-Vector::y() * 9.81),
            &mut scene.bodies,
            &scene.colliders,
            &scene.queries,
            QueryFilter::default(),
        );
        assert_eq!(scene.bodies[handle].gravity_scale(), scale);
        assert_eq!(*scene.bodies[handle].linvel(), Vector::zeros());
        scene.tick();
        assert_eq!(scene.bodies[handle].gravity_scale(), scale);
        let expected = -9.81 * scale / 60.0 / (1.0 + scene.bodies[handle].linear_damping() / 60.0);
        assert!((scene.bodies[handle].linvel().y - expected).abs() < 0.00001);
        scene.vehicle.finish_vehicle_update(&mut scene.bodies);
        assert_eq!(scene.bodies[handle].gravity_scale(), scale);
    }
}

#[test]
fn suspension_and_tires_exchange_momentum_with_dynamic_ground() {
    let mut scene = Scene::new(60, 4, 0.0, 0.0);
    scene.params.min_island_size = 1;
    let ground = scene
        .colliders
        .iter()
        .find(|(_, c)| c.parent().is_none())
        .unwrap()
        .0;
    scene
        .colliders
        .remove(ground, &mut scene.islands, &mut scene.bodies, true);
    let platform = scene.bodies.insert(
        RigidBodyBuilder::dynamic()
            .gravity_scale(0.0)
            .can_sleep(false),
    );
    scene.colliders.insert_with_parent(
        ColliderBuilder::cuboid(50.0, 0.1, 50.0)
            .translation(-Vector::y() * 0.1)
            .mass(10000.0),
        platform,
        &mut scene.bodies,
    );
    scene.queries.update(&scene.colliders);
    scene.bodies[platform].sleep();
    scene.bodies[scene.vehicle.chassis].set_linvel(Vector::x(), true);
    scene.tick();
    let car = &scene.bodies[scene.vehicle.chassis];
    let ground = &scene.bodies[platform];
    let momentum = car.linvel() * car.mass() + ground.linvel() * ground.mass();
    assert!((momentum.x - 1200.0).abs() < 1.0, "momentum={momentum:?}");
    assert!(
        (momentum.y + 1200.0 * 9.81 / 60.0).abs() < 1.0,
        "momentum={momentum:?}"
    );
    assert!(ground.linvel().x > 0.0 && ground.linvel().y < 0.0);
}

#[test]
fn locked_wheels_follow_kinematic_ground() {
    let mut scene = Scene::new(60, 4, 0.0, 0.0);
    let ground = scene
        .colliders
        .iter()
        .find(|(_, c)| c.parent().is_none())
        .unwrap()
        .0;
    scene
        .colliders
        .remove(ground, &mut scene.islands, &mut scene.bodies, true);
    let platform = scene
        .bodies
        .insert(RigidBodyBuilder::kinematic_velocity_based().linvel(Vector::x()));
    scene.colliders.insert_with_parent(
        ColliderBuilder::cuboid(50.0, 0.1, 50.0).translation(-Vector::y() * 0.1),
        platform,
        &mut scene.bodies,
    );
    scene.queries.update(&scene.colliders);
    scene.bodies[scene.vehicle.chassis].set_linvel(Vector::x(), true);
    for _ in 0..300 {
        scene.tick();
    }
    let start = scene.position().x;
    for _ in 0..120 {
        scene.tick();
    }
    assert!((scene.position().x - start - 2.0).abs() < 0.01);
    assert!((scene.bodies[scene.vehicle.chassis].linvel().x - 1.0).abs() < 0.005);
}

#[test]
fn cancellation_restores_automatic_gravity() {
    let mut scene = Scene::new(60, 4, 0.0, 0.0);
    scene.vehicle.update_vehicle(
        scene.params.dt,
        &(-Vector::y() * 9.81),
        &mut scene.bodies,
        &scene.colliders,
        &scene.queries,
        QueryFilter::default().exclude_rigid_body(scene.vehicle.chassis),
    );
    assert_eq!(scene.bodies[scene.vehicle.chassis].gravity_scale(), 0.0);
    let prepared_velocity = scene.bodies[scene.vehicle.chassis].linvel().y;
    scene.vehicle.cancel_vehicle_update(&mut scene.bodies);
    assert_eq!(scene.bodies[scene.vehicle.chassis].gravity_scale(), 1.0);
    assert!(
        (scene.bodies[scene.vehicle.chassis].linvel().y - prepared_velocity - 9.81 / 60.0).abs()
            < 0.00001
    );
    scene.pipeline.step(
        &(-Vector::y() * 9.81),
        &scene.params,
        &mut scene.islands,
        &mut scene.broad,
        &mut scene.narrow,
        &mut scene.bodies,
        &mut scene.colliders,
        &mut scene.joints,
        &mut scene.multis,
        &mut scene.ccd,
        None,
        &(),
        &(),
    );
    let expected =
        prepared_velocity / (1.0 + scene.bodies[scene.vehicle.chassis].linear_damping() / 60.0);
    assert!((scene.bodies[scene.vehicle.chassis].linvel().y - expected).abs() < 0.00001);
}

#[test]
fn powertrain_accelerates_and_service_brakes_stop_across_substep_counts() {
    for iterations in [1, 4, 8] {
        let mut scene = Scene::new(60, iterations, 0.0, 0.0);
        for _ in 0..120 {
            scene.tick();
        }
        scene.vehicle.set_gear(1);
        scene.vehicle.set_input(VehicleInput {
            throttle: 1.0,
            ..Default::default()
        });
        for _ in 0..300 {
            scene.tick();
        }
        let speed = scene.bodies[scene.vehicle.chassis].linvel().norm();
        assert!(
            speed > 2.0,
            "iterations={iterations} acceleration speed={speed}"
        );
        assert!(scene
            .vehicle
            .wheels()
            .iter()
            .any(|w| w.delta_rotation.abs() > 0.01));
        scene.vehicle.set_input(VehicleInput {
            brake: 1.0,
            clutch: 1.0,
            ..Default::default()
        });
        for _ in 0..300 {
            scene.tick();
        }
        let speed = scene.bodies[scene.vehicle.chassis].linvel().norm();
        assert!(
            speed < 0.01,
            "iterations={iterations} stopped speed={speed}"
        );
        assert!(scene
            .vehicle
            .wheels()
            .iter()
            .all(|w| w.delta_rotation == 0.0));
    }
}
