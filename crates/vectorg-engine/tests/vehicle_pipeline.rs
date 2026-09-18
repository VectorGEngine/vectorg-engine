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
        let mut config = VehicleControllerConfig::default();
        config.transmission.automatic = false;
        config.transmission.auto_reverse = false;
        config.dynamics.esc_strength = 0.0;
        Self::with_config(hz, iterations, slope, heading, config)
    }

    fn with_config(
        hz: u32,
        iterations: usize,
        slope: f32,
        heading: f32,
        config: VehicleControllerConfig,
    ) -> Self {
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
                0.2,
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
fn autoclutch_launch_overshoots_releases_and_couples_below_trigger_in_the_vehicle_pipeline() {
    for hz in [30, 60, 120] {
        for (idle, max, inertia, peak) in
            [(1000.0, 8000.0, 0.2, 600.0), (4500.0, 15000.0, 0.06, 700.0)]
        {
            for (direction, slope, throttle) in
                [(1, 0.0, 1.0), (-1, 0.0, 0.5), (1, -8.0, 1.0), (1, 8.0, 1.0)]
            {
                let mut config = VehicleControllerConfig::default();
                config.engine.idle_rpm = idle;
                config.engine.max_rpm = max;
                config.engine.rev_limit_rpm = max;
                config.engine.inertia = inertia;
                config.engine.friction_torque = Some(70.0);
                config.engine.torque_curve = vec![(idle, peak), (max, peak * 0.8)];
                config.transmission.automatic = false;
                config.transmission.auto_clutch = true;
                config.transmission.auto_reverse = false;
                config.transmission.clutch_response = if idle > 4000.0 { 22.0 } else { 12.0 };
                config.transmission.shift_cooldown = 0.0;
                config.transmission.forward_ratios = vec![3.5];
                config.transmission.reverse_ratio = -3.5;
                config.transmission.final_drive_ratio = 4.0;
                config.dynamics.esc_strength = 0.0;
                let mut scene = Scene::with_config(hz, 4, slope, 0.0, config);
                // Isolate the driveline from wheelies in this simple box-body
                // fixture, which has neither an F1 chassis nor its downforce.
                scene.bodies[scene.vehicle.chassis].lock_rotations(true, true);
                for wheel in scene.vehicle.wheels_mut() {
                    // Enough grip to exercise clutch regulation independently
                    // of wheelspin; TC/low-grip cases are covered separately.
                    wheel.friction_slip = 4.0;
                }
                for _ in 0..hz {
                    scene.tick();
                }
                scene.vehicle.set_gear(direction);
                scene.tick();
                scene.vehicle.set_input(VehicleInput {
                    throttle,
                    ..Default::default()
                });
                let target = idle * 1.05 + (max * 0.6 - idle * 1.05) * ((throttle - 0.1) / 0.9);
                let mut reached = false;
                let mut released = false;
                let mut peak_rpm = idle;
                let mut coupled = false;
                for step in 0..hz * 8 {
                    scene.tick();
                    let state = scene.vehicle.state();
                    let road_rpm = state.vehicle_speed * direction as f32
                        / (std::f32::consts::TAU * 0.35)
                        * 14.0;
                    let road_rpm = road_rpm * 60.0;
                    let shaft_rpm = state.driven_wheel_speed.abs() / (std::f32::consts::TAU * 0.35)
                        * 14.0
                        * 60.0;
                    assert!(
                        state.engine_running,
                        "{hz} Hz idle {idle} slope {slope} step {step}"
                    );
                    if !released {
                        peak_rpm = peak_rpm.max(state.engine_rpm);
                    }
                    reached |= state.engine_rpm > target;
                    if reached && road_rpm < target * 0.1 {
                        assert!(state.engine_rpm > target * 0.8,
                            "RPM collapsed before moving: {hz} Hz idle={idle} road={road_rpm} engine={}",
                            state.engine_rpm);
                    }
                    released |=
                        reached && state.engine_rpm < target * 0.9 && road_rpm < target * 0.9;
                    if released
                        && shaft_rpm > idle
                        && shaft_rpm < target
                        && (state.engine_rpm - shaft_rpm).abs() < target * 0.08
                    {
                        coupled = true;
                    }
                }
                eprintln!("launch {hz}Hz idle={idle} dir={direction} slope={slope} throttle={throttle}: peak={peak_rpm} released={released} coupled={coupled} speed={}", scene.vehicle.state().vehicle_speed);
                assert!(reached && released, "launch must overshoot then release");
                assert!(peak_rpm < max * 0.97, "launch must not reach the limiter");
                assert!(
                    coupled,
                    "launch must synchronize below the original trigger"
                );
                assert!(scene.vehicle.state().vehicle_speed * direction as f32 > 5.0);
            }
        }
    }
}

#[test]
fn autoclutch_low_grip_launch_and_braking_preserve_traction_control() {
    for hz in [30, 60, 120] {
        for tc in [0.0, 1.0] {
            let mut config = VehicleControllerConfig::default();
            config.engine.idle_rpm = 4500.0;
            config.engine.max_rpm = 15000.0;
            config.engine.rev_limit_rpm = 15000.0;
            config.engine.inertia = 0.06;
            config.engine.friction_torque = Some(70.0);
            config.engine.torque_curve = vec![(4500.0, 650.0), (15000.0, 550.0)];
            config.transmission.automatic = false;
            config.transmission.auto_clutch = true;
            config.transmission.auto_reverse = false;
            config.transmission.forward_ratios = vec![3.5];
            config.transmission.final_drive_ratio = 4.0;
            config.dynamics.traction_control_strength = tc;
            config.dynamics.esc_strength = 0.0;
            let mut scene = Scene::with_config(hz, 4, 0.0, 0.0, config);
            scene.bodies[scene.vehicle.chassis].lock_rotations(true, true);
            for wheel in scene.vehicle.wheels_mut() {
                wheel.friction_slip = 0.6;
                wheel.traction_control = tc;
            }
            for _ in 0..hz {
                scene.tick();
            }
            scene.vehicle.set_gear(1);
            scene.tick();
            scene.vehicle.set_input(VehicleInput {
                throttle: 1.0,
                ..Default::default()
            });
            let mut peak_tc: f32 = 0.0;
            for _ in 0..hz * 5 {
                scene.tick();
                let state = scene.vehicle.state();
                assert!(state.engine_running && state.engine_rpm <= 15000.0);
                peak_tc = peak_tc.max(state.traction_control_activity);
            }
            assert!(
                scene.vehicle.state().vehicle_speed > 2.0,
                "{hz} Hz TC {tc}: car must move"
            );
            assert_eq!(peak_tc > 0.0, tc > 0.0);
            scene.vehicle.set_input(VehicleInput {
                brake: 1.0,
                ..Default::default()
            });
            for _ in 0..hz * 5 {
                scene.tick();
                assert!(scene.vehicle.state().engine_running);
            }
            assert!(scene.vehicle.state().vehicle_speed.abs() < 0.1);
            assert!((scene.vehicle.state().engine_rpm - 4500.0).abs() < 45.0);
        }
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
                    // At 45 degrees the chassis keeps a bounded sub-millimetre
                    // oscillation instead of a fixed rest pose, so the sampled
                    // drift reaches about 0.6 mm.
                    let limit = if heading == 45.0 { 0.00075 } else { 0.0005 };
                    assert!(
                        drift < limit,
                        "game timestep heading={heading} drift={drift}"
                    );
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
fn kerb_crossing_and_landing_settle_through_the_physics_pipeline() {
    for hz in [30, 60, 120] {
        for one_side in [false, true] {
            let mut scene = Scene::new(hz, 4, 0.0, 0.0);
            scene.colliders.insert(
                ColliderBuilder::cuboid(if one_side { 1.0 } else { 10.0 }, 0.05, 10.0)
                    .translation(Vector::new(if one_side { 1.0 } else { 0.0 }, 0.05, 12.0)),
            );
            scene.queries.update(&scene.colliders);
            for wheel in scene.vehicle.wheels_mut() {
                wheel.max_suspension_travel = 0.4;
                wheel.max_suspension_force = 20_000.0;
            }
            for _ in 0..hz * 2 {
                scene.tick();
            }
            scene.vehicle.set_input(VehicleInput {
                clutch: 1.0,
                ..Default::default()
            });
            let chassis = scene.vehicle.chassis;
            scene.bodies[chassis].set_linvel(Vector::new(0.0, 0.0, 5.0), true);
            let mut peak: f32 = 0.0;
            for _ in 0..hz * 2 {
                scene.tick();
                peak = peak.max(scene.bodies[chassis].linvel().y);
                assert!(scene.position().y < 1.2 && scene.position().y > 0.3);
            }
            assert!(scene.position().z > 4.0, "car must cross the kerb");
            assert!(peak < 2.0, "hz={hz} one_side={one_side} peak={peak}");
            scene.vehicle.set_input(VehicleInput {
                brake: 1.0,
                clutch: 1.0,
                ..Default::default()
            });
            for _ in 0..hz * 4 {
                scene.tick();
            }
            assert!(scene.bodies[chassis].linvel().y.abs() < 0.03);
            // A true airborne landing still receives support and settles.
            let mut position = scene.position();
            position.y += 0.8;
            scene.bodies[chassis].set_translation(position, true);
            for _ in 0..hz * 4 {
                scene.tick();
            }
            assert!(scene.bodies[chassis].linvel().norm() < 0.05);
            assert!(
                scene
                    .vehicle
                    .wheels()
                    .iter()
                    .filter(|w| w.raycast_info().is_in_contact)
                    .count()
                    >= 3
            );
        }
    }
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
