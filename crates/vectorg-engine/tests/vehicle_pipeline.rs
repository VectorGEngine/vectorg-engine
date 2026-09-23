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
        self.vehicle.finish_vehicle_update(
            &mut self.bodies,
            &self.colliders,
            &self.queries,
            QueryFilter::default(),
        );
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
        scene.vehicle.finish_vehicle_update(
            &mut scene.bodies,
            &scene.colliders,
            &scene.queries,
            QueryFilter::default(),
        );
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

fn full_tc_launch_speed(hz: u32, lock: f32, steering: f32, awd: bool) -> f32 {
    let mut config = VehicleControllerConfig::default();
    config.engine.idle_rpm = 1000.0;
    config.engine.max_rpm = 7000.0;
    config.engine.rev_limit_rpm = 7000.0;
    config.engine.torque_curve = vec![(1000.0, 400.0), (7000.0, 350.0)];
    config.transmission.automatic = false;
    config.transmission.auto_clutch = true;
    config.transmission.auto_reverse = false;
    config.transmission.forward_ratios = vec![3.5];
    config.transmission.final_drive_ratio = 4.0;
    config.dynamics.traction_control_strength = 1.0;
    config.dynamics.esc_strength = 0.0;
    config.differential = VehicleDifferentialConfig {
        front_accel_lock: lock,
        front_decel_lock: lock,
        rear_accel_lock: lock,
        rear_decel_lock: lock,
        center_balance: 0.5,
        center_lock: 0.0,
    };
    let mut scene = Scene::with_config(hz, 4, 0.0, 0.0, config);
    for wheel in scene.vehicle.wheels_mut() {
        let front = wheel.role.axle == WheelAxle::Front;
        wheel.role = WheelRole::new(wheel.role.axle, awd || front, front);
    }
    for _ in 0..hz {
        scene.tick();
    }
    scene.vehicle.set_gear(1);
    scene.tick();
    scene.vehicle.set_input(VehicleInput {
        throttle: 1.0,
        steering,
        ..Default::default()
    });
    for _ in 0..hz * 3 {
        scene.tick();
    }
    scene
        .tangent(*scene.bodies[scene.vehicle.chassis].linvel())
        .norm()
}

#[test]
fn full_tc_launches_at_full_steering_lock_for_every_differential_lock() {
    // A coupled axle cannot roll through a tight turn. Scrub forced onto the
    // inner wheel is not wheelspin, so no lock may stall a full-TC launch.
    // Straight-line launches must not depend on the lock.
    for hz in [30, 60, 120] {
        for awd in [false, true] {
            for steering in [1.0, 0.0] {
                let open = full_tc_launch_speed(hz, 0.0, steering, awd);
                for lock in [0.25, 0.5, 0.75, 0.9, 0.99, 0.999, 1.0] {
                    let speed = full_tc_launch_speed(hz, lock, steering, awd);
                    if steering == 0.0 {
                        assert!(
                            (speed - open).abs() < 0.01,
                            "{hz} Hz awd={awd} lock={lock}: straight {speed} != open {open}"
                        );
                    } else {
                        assert!(
                            speed > open * 0.85,
                            "{hz} Hz awd={awd} lock={lock}: full-lock launch {speed} vs open {open}"
                        );
                    }
                }
            }
        }
    }
}

#[derive(Clone, Copy)]
struct CenterCase {
    hz: u32,
    slope: f32,
    center: f32,
    balance: f32,
    axle_lock: f32,
    tc: f32,
    abs: f32,
    rear_grip: f32,
}

impl Default for CenterCase {
    fn default() -> Self {
        Self {
            hz: 60,
            slope: 0.0,
            center: 0.0,
            balance: 0.5,
            axle_lock: 0.3,
            tc: 0.0,
            abs: 0.0,
            rear_grip: 1.0,
        }
    }
}

fn awd_scene(case: CenterCase) -> Scene {
    let mut config = VehicleControllerConfig::default();
    config.engine.idle_rpm = 1000.0;
    config.engine.max_rpm = 7000.0;
    config.engine.rev_limit_rpm = 7000.0;
    config.engine.torque_curve = vec![(1000.0, 400.0), (7000.0, 350.0)];
    config.transmission.automatic = false;
    config.transmission.auto_clutch = true;
    config.transmission.auto_reverse = false;
    config.transmission.forward_ratios = vec![3.5];
    config.transmission.final_drive_ratio = 4.0;
    config.dynamics.traction_control_strength = case.tc;
    config.dynamics.abs_strength = case.abs;
    config.dynamics.esc_strength = 0.0;
    config.differential = VehicleDifferentialConfig {
        front_accel_lock: case.axle_lock,
        front_decel_lock: case.axle_lock,
        rear_accel_lock: case.axle_lock,
        rear_decel_lock: case.axle_lock,
        center_balance: case.balance,
        center_lock: case.center,
    };
    let mut scene = Scene::with_config(case.hz, 4, case.slope, 0.0, config);
    for wheel in scene.vehicle.wheels_mut() {
        let front = wheel.role.axle == WheelAxle::Front;
        wheel.role = WheelRole::new(wheel.role.axle, true, front);
        wheel.anti_lock_brake = case.abs;
        if !front {
            wheel.friction_slip *= case.rear_grip;
        }
    }
    scene
}

fn planar_speed(scene: &Scene) -> f32 {
    scene
        .tangent(*scene.bodies[scene.vehicle.chassis].linvel())
        .norm()
}

fn awd_launch_speed(case: CenterCase, steering: f32, seconds: u32) -> f32 {
    let mut scene = awd_scene(case);
    for _ in 0..case.hz {
        scene.tick();
    }
    scene.vehicle.set_gear(1);
    scene.tick();
    scene.vehicle.set_input(VehicleInput {
        throttle: 1.0,
        steering,
        ..Default::default()
    });
    for _ in 0..case.hz * seconds {
        scene.tick();
    }
    planar_speed(&scene)
}

fn awd_accelerate_to(scene: &mut Scene, hz: u32, speed: f32) {
    for _ in 0..hz {
        scene.tick();
    }
    scene.vehicle.set_gear(1);
    scene.vehicle.set_input(VehicleInput {
        throttle: 1.0,
        ..Default::default()
    });
    while planar_speed(scene) < speed {
        scene.tick();
    }
}

#[test]
fn center_lock_lets_the_gripping_axle_pull_when_the_other_spins() {
    for hz in [30, 60, 120] {
        for tc in [0.0, 1.0] {
            let mut previous = 0.0;
            for center in [0.0, 0.25, 0.5, 0.75, 1.0] {
                let speed = awd_launch_speed(
                    CenterCase {
                        hz,
                        center,
                        tc,
                        rear_grip: 0.05,
                        ..Default::default()
                    },
                    0.0,
                    2,
                );
                assert!(
                    speed >= previous * 0.98,
                    "{hz} Hz tc={tc} center={center}: {speed} after {previous}"
                );
                if center >= 0.5 {
                    // An open center lets the spinning rear starve the front.
                    assert!(speed > 5.0, "{hz} Hz tc={tc} center={center}: {speed}");
                }
                previous = speed;
            }
        }
    }
}

#[test]
fn center_lock_launches_with_full_tc_at_full_steering_lock() {
    // Front and rear cannot roll at one speed through a tight turn. The shaft
    // forces that scrub, so TC must not treat it as wheelspin at any lock.
    for hz in [30, 60, 120] {
        for balance in [0.3, 0.5, 0.7] {
            let case = CenterCase {
                hz,
                balance,
                tc: 1.0,
                ..Default::default()
            };
            let open = awd_launch_speed(case, 1.0, 3);
            for center in [0.25, 0.5, 0.75, 1.0] {
                let speed = awd_launch_speed(CenterCase { center, ..case }, 1.0, 3);
                assert!(
                    speed > open * 0.9,
                    "{hz} Hz balance={balance} center={center}: {speed} vs open {open}"
                );
            }
        }
    }
}

#[test]
fn center_lock_handbrake_disconnects_rear_drive() {
    for hz in [30, 60, 120] {
        let mut previous_force = f32::MAX;
        for center in [0.0, 0.5, 1.0] {
            let mut scene = awd_scene(CenterCase {
                hz,
                center,
                ..Default::default()
            });
            awd_accelerate_to(&mut scene, hz, 10.0);
            scene.vehicle.set_input(VehicleInput {
                throttle: 0.5,
                handbrake: 1.0,
                steering: 0.5,
                ..Default::default()
            });
            let mut rear_force: f32 = 0.0;
            for _ in 0..hz / 2 {
                scene.tick();
                let w = scene.vehicle.wheels();
                assert!(w[2].delta_rotation == 0.0 && w[3].delta_rotation == 0.0);
                assert!(w[0].delta_rotation != 0.0 && w[1].delta_rotation != 0.0);
                rear_force = rear_force.max(w[2].engine_force.abs() + w[3].engine_force.abs());
            }
            assert!(rear_force < previous_force, "{hz} Hz center={center}");
            if center == 1.0 {
                assert_eq!(rear_force, 0.0);
            }
            previous_force = rear_force;
        }
    }
}

#[test]
fn center_lock_holds_a_parked_car_on_a_hill() {
    for hz in [30, 60, 120] {
        for center in [0.5, 1.0] {
            for slope in [-15.0, 15.0] {
                let mut scene = awd_scene(CenterCase {
                    hz,
                    center,
                    slope,
                    ..Default::default()
                });
                scene.vehicle.set_input(VehicleInput {
                    brake: 1.0,
                    clutch: 1.0,
                    ..Default::default()
                });
                for _ in 0..hz * 2 {
                    scene.tick();
                }
                let start = scene.position();
                for _ in 0..hz * 3 {
                    scene.tick();
                    assert!(scene
                        .vehicle
                        .wheels()
                        .iter()
                        .all(|w| w.delta_rotation == 0.0));
                }
                let drift = (scene.position() - start).norm();
                assert!(
                    drift < 1e-3,
                    "{hz} Hz center={center} slope={slope}: {drift}"
                );
            }
        }
    }
}

#[test]
fn center_lock_does_not_lengthen_abs_braking() {
    for hz in [30, 60, 120] {
        for steering in [0.0, 0.3] {
            let distance = |center| {
                let mut scene = awd_scene(CenterCase {
                    hz,
                    center,
                    abs: 1.0,
                    ..Default::default()
                });
                awd_accelerate_to(&mut scene, hz, 12.0);
                let start = scene.position();
                scene.vehicle.set_input(VehicleInput {
                    brake: 1.0,
                    clutch: 1.0,
                    steering,
                    ..Default::default()
                });
                while planar_speed(&scene) > 0.2 {
                    scene.tick();
                }
                (scene.position() - start).norm()
            };
            let open = distance(0.0);
            for center in [0.5, 1.0] {
                let locked = distance(center);
                assert!(
                    locked < open * 1.02,
                    "{hz} Hz steering={steering} center={center}: {locked} vs open {open}"
                );
            }
        }
    }
}
