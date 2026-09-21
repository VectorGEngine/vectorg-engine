use super::tests::{four_wheel_test_vehicle, set_test_drive};
use super::*;

fn locks(controller: &mut DynamicRayCastVehicleController, value: Real) {
    controller.powertrain.config.differential = super::super::VehicleDifferentialConfig {
        front_accel_lock: value,
        front_decel_lock: value,
        rear_accel_lock: value,
        rear_decel_lock: value,
        center_balance: 0.5,
        center_lock: 0.0,
    };
}

#[test]
fn differential_free_axle_conserves_momentum_and_dissipates_relative_spin() {
    for hz in [30, 60, 120] {
        let mut last_gap = Real::MAX;
        for lock in [0.0, 0.25, 0.5, 0.75, 0.99, 1.0] {
            let (mut c, mut bodies, colliders) = four_wheel_test_vehicle(0.0, 0.0);
            locks(&mut c, lock);
            for w in &mut c.wheels {
                w.raycast_info.ground_object = None;
            }
            c.wheels[2].angular_velocity = -20.0;
            c.wheels[3].angular_velocity = 70.0;
            c.wheels[3].radius = 0.5;
            let inertia = [wheel_angular_inertia(0.35), wheel_angular_inertia(0.5)];
            let momentum = -20.0 * inertia[0] + 70.0 * inertia[1];
            let energy = (400.0 * inertia[0] + 4900.0 * inertia[1]) * 0.5;
            c.update_friction(&mut bodies, &colliders, 1.0 / hz as Real);
            let [a, b] = [c.wheels[2].angular_velocity, c.wheels[3].angular_velocity];
            assert!((a * inertia[0] + b * inertia[1] - momentum).abs() < 0.0001);
            assert!((a * a * inertia[0] + b * b * inertia[1]) * 0.5 <= energy + 0.001);
            assert!((a - b).abs() <= last_gap + 0.0001);
            last_gap = (a - b).abs();
            if lock == 0.0 {
                assert_eq!([a, b], [-20.0, 70.0]);
            }
            if lock == 1.0 {
                assert_eq!(a, b);
            }
        }
    }
}

#[test]
fn differential_full_lock_survives_brakes_assists_and_lost_contacts() {
    for hz in [30, 60, 120] {
        for direction in [-1.0, 1.0] {
            for assist in [0.0, 0.5, 1.0] {
                for airborne in [0, 1, 2] {
                    let (mut c, mut bodies, colliders) =
                        four_wheel_test_vehicle(20.0 * direction, assist);
                    locks(&mut c, 1.0);
                    for w in &mut c.wheels {
                        w.role.driven = true;
                        w.anti_lock_brake = assist;
                    }
                    c.wheels[3].radius = 0.45;
                    c.wheels[2].wheel_suspension_force = 200.0;
                    for &i in [2, 3].iter().take(airborne) {
                        c.wheels[i].raycast_info.ground_object = None;
                    }
                    for step in 0..24 {
                        set_test_drive(
                            &mut c,
                            if step < 12 {
                                1500.0 * direction
                            } else {
                                -100.0 * direction
                            },
                        );
                        for (i, w) in c.wheels.iter_mut().enumerate() {
                            w.target_rotation = 100.0 * direction;
                            w.brake = if step < 6 {
                                0.0
                            } else if i % 2 == 0 {
                                0.6
                            } else {
                                0.1
                            };
                            w.max_brake_force = 15000.0;
                        }
                        c.update_friction(&mut bodies, &colliders, 1.0 / hz as Real);
                        assert_eq!(c.wheels[0].angular_velocity, c.wheels[1].angular_velocity);
                        assert_eq!(c.wheels[2].angular_velocity, c.wheels[3].angular_velocity);
                        for (i, w) in c.wheels.iter().enumerate() {
                            assert!(w.angular_velocity.is_finite());
                            let base = c.contact_solver.contacts[i].base;
                            let limit = base.grip_impulse * c.contact_solver.actuations[i].grip;
                            let impulse = [w.forward_impulse, w.side_impulse];
                            assert!(envelope_norm(impulse, base.shape) <= limit + 0.001);
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn differential_modes_switch_and_clutch_disconnection_uses_coast() {
    for direction in [-1.0, 1.0] {
        let (mut c, mut bodies, colliders) = four_wheel_test_vehicle(20.0 * direction, 0.0);
        c.powertrain.config.differential.rear_accel_lock = 1.0;
        c.powertrain.config.differential.rear_decel_lock = 0.0;
        for w in &mut c.wheels {
            w.raycast_info.ground_object = None;
        }
        set_test_drive(&mut c, 200.0 * direction);
        c.update_friction(&mut bodies, &colliders, 0.01);
        assert!(c.differential_accelerating[1]);
        assert_eq!(c.wheels[2].angular_velocity, c.wheels[3].angular_velocity);
        set_test_drive(&mut c, -200.0 * direction);
        c.wheels[2].angular_velocity += direction * 20.0;
        c.update_friction(&mut bodies, &colliders, 0.01);
        assert!(!c.differential_accelerating[1]);
        assert!((c.wheels[2].angular_velocity - c.wheels[3].angular_velocity).abs() > 19.0);
        c.powertrain.config.differential.rear_decel_lock = 1.0;
        for w in &mut c.wheels {
            w.drivetrain_connected = false;
            w.wheel_coupling_torque = 0.0;
        }
        c.update_friction(&mut bodies, &colliders, 0.01);
        assert_eq!(c.wheels[2].angular_velocity, c.wheels[3].angular_velocity);
        c.reset();
        assert_eq!(c.differential_accelerating, [false, false]);
    }
}

#[test]
fn differential_awd_weights_and_shaft_speed_follow_center_balance() {
    for bias in [0.0, 0.25, 0.5, 0.75, 1.0] {
        let (mut c, _, _) = four_wheel_test_vehicle(0.0, 0.0);
        for (i, w) in c.wheels.iter_mut().enumerate() {
            w.role.driven = true;
            w.angular_velocity = [10.0, 30.0, 50.0, 70.0][i];
        }
        c.powertrain.config.differential.center_balance = bias;
        let weights = c.drive_weights();
        assert_eq!(
            weights,
            vec![
                bias * 0.5,
                bias * 0.5,
                (1.0 - bias) * 0.5,
                (1.0 - bias) * 0.5
            ]
        );
        let (speed, radius) = c.driven_wheel_speed_and_radius();
        assert!((speed / radius - (20.0 * bias + 60.0 * (1.0 - bias))).abs() < 0.00001);
        c.apply_powertrain_output(super::super::vehicle_powertrain::PowertrainOutput {
            drive_torque: 1000.0,
            engine_brake_torque: 0.0,
            wheel_coupling_torque: 1000.0,
            wheel_target_velocity: 100.0,
            drive_throttle: 1.0,
            drivetrain_connected: true,
            service_brake: 0.0,
        });
        for (w, weight) in c.wheels.iter().zip(weights) {
            assert_eq!(w.wheel_coupling_torque, 1000.0 * weight);
        }
        for w in &mut c.wheels {
            w.role.driven = w.role.axle == WheelAxle::Rear;
        }
        assert_eq!(c.drive_weights(), vec![0.0, 0.0, 0.5, 0.5]);
        for w in &mut c.wheels {
            w.role.driven = w.role.axle == WheelAxle::Front;
        }
        assert_eq!(c.drive_weights(), vec![0.5, 0.5, 0.0, 0.0]);
    }
}

#[test]
fn differential_locked_corner_keeps_useful_drive_with_full_tc() {
    for lock in [0.0, 0.5, 0.99, 1.0] {
        let (mut c, mut bodies, colliders) = four_wheel_test_vehicle(15.0, 1.0);
        locks(&mut c, lock);
        let mut forward_impulse = 0.0;
        for step in 0..120 {
            bodies[c.chassis].set_linvel(Vector::z() * 15.0, true);
            bodies[c.chassis].set_angvel(Vector::y() * 0.6, true);
            set_test_drive(&mut c, 800.0);
            c.update_friction(&mut bodies, &colliders, 1.0 / 60.0);
            if step >= 90 {
                forward_impulse += c.wheels[2].forward_impulse + c.wheels[3].forward_impulse;
            }
        }
        assert!(
            forward_impulse > 5.0,
            "lock={lock}: forward impulse {forward_impulse}"
        );
        if lock == 1.0 {
            assert_eq!(c.wheels[2].angular_velocity, c.wheels[3].angular_velocity);
        }
    }
}

#[test]
fn differential_response_is_independent_of_wheel_insertion_order() {
    for lock in [0.0, 0.5, 1.0] {
        let (mut a, mut ba, ca) = four_wheel_test_vehicle(12.0, 0.0);
        let (mut b, mut bb, cb) = four_wheel_test_vehicle(12.0, 0.0);
        for c in [&mut a, &mut b] {
            locks(c, lock);
            c.wheels[2].radius = 0.4;
            c.wheels[2].wheel_suspension_force *= 0.4;
            for w in &mut c.wheels {
                w.role.driven = true;
                w.brake = 0.1;
                w.max_brake_force = 2000.0;
            }
        }
        b.wheels.reverse();
        for step in 0..30 {
            set_test_drive(&mut a, 600.0);
            set_test_drive(&mut b, 600.0);
            a.update_friction(&mut ba, &ca, 1.0 / 60.0);
            b.update_friction(&mut bb, &cb, 1.0 / 60.0);
            assert!((ba[a.chassis].linvel() - bb[b.chassis].linvel()).norm() < 0.0002);
            for (left, right) in a.wheels.iter().zip(b.wheels.iter().rev()) {
                assert!(
                    (left.angular_velocity - right.angular_velocity).abs() < 0.001,
                    "lock={lock} step={step} left={} right={} residual={}/{}",
                    left.angular_velocity,
                    right.angular_velocity,
                    a.contact_solver.residual,
                    b.contact_solver.residual
                );
                assert!((left.forward_impulse - right.forward_impulse).abs() < 0.01);
            }
        }
    }
}

#[test]
fn differential_center_couples_axle_means_with_lost_contacts_and_unequal_radii() {
    for hz in [30, 60, 120] {
        for center in [0.5, 1.0] {
            for airborne in [[].as_slice(), &[0, 1], &[2, 3]] {
                for rear_radius in [0.35, 0.45] {
                    let (mut c, mut bodies, colliders) = four_wheel_test_vehicle(10.0, 1.0);
                    c.powertrain.config.differential.center_lock = center;
                    for w in &mut c.wheels {
                        w.role.driven = true;
                    }
                    for i in [2, 3] {
                        c.wheels[i].radius = rear_radius;
                    }
                    for &i in airborne {
                        c.wheels[i].raycast_info.ground_object = None;
                    }
                    for step in 0..30 {
                        set_test_drive(&mut c, if step < 20 { 1500.0 } else { -200.0 });
                        c.update_friction(&mut bodies, &colliders, 1.0 / hz as Real);
                        let w: Vec<Real> = c.wheels.iter().map(|w| w.angular_velocity).collect();
                        assert!(w.iter().all(|v| v.is_finite()), "{w:?}");
                        if center == 1.0 {
                            // A rigid center is a constraint, not a stiff clutch.
                            let mismatch = ((w[0] + w[1]) - (w[2] + w[3])).abs() * 0.5;
                            assert!(mismatch < 1e-3, "{hz} Hz: mean mismatch {mismatch}");
                        }
                        for (i, wheel) in c.wheels.iter().enumerate() {
                            let base = c.contact_solver.contacts[i].base;
                            let limit = base.grip_impulse * c.contact_solver.actuations[i].grip;
                            let impulse = [wheel.forward_impulse, wheel.side_impulse];
                            assert!(envelope_norm(impulse, base.shape) <= limit + 0.001);
                        }
                    }
                    assert!(c.contact_solver.residual <= CONTACT_SOLVER_TOLERANCE);
                }
            }
        }
    }
}

#[test]
fn differential_rigid_center_and_axles_turn_every_wheel_together() {
    for hz in [30, 60, 120] {
        let (mut c, mut bodies, colliders) = four_wheel_test_vehicle(10.0, 0.0);
        locks(&mut c, 1.0);
        c.powertrain.config.differential.center_lock = 1.0;
        for w in &mut c.wheels {
            w.role.driven = true;
        }
        c.wheels[0].friction_slip = 0.05;
        for step in 0..30 {
            set_test_drive(&mut c, if step < 20 { 1500.0 } else { -200.0 });
            c.update_friction(&mut bodies, &colliders, 1.0 / hz as Real);
            let w: Vec<Real> = c.wheels.iter().map(|w| w.angular_velocity).collect();
            assert!(w.iter().all(|&v| v == w[0]), "{hz} Hz: {w:?}");
        }
    }
}

#[test]
fn differential_open_center_adds_no_coupling_and_handbrake_releases_a_locked_one() {
    let (mut c, mut bodies, colliders) = four_wheel_test_vehicle(10.0, 0.0);
    for w in &mut c.wheels {
        w.role.driven = true;
    }
    set_test_drive(&mut c, 500.0);
    c.update_friction(&mut bodies, &colliders, 1.0 / 60.0);
    assert!(c.contact_solver.center.is_none());

    c.powertrain.config.differential.center_lock = 1.0;
    c.powertrain.config.differential.center_balance = 0.3;
    for (handbrake, lock, front) in [(0.0, 1.0, 0.3), (0.5, 0.5, 0.65), (1.0, 0.0, 1.0)] {
        c.set_input(VehicleInput {
            handbrake,
            ..VehicleInput::default()
        });
        assert_eq!(c.center_lock(), lock);
        // The disconnected rear share moves to the front.
        let weights = c.drive_weights();
        assert!(
            (weights[0] + weights[1] - front).abs() < 1e-6,
            "{weights:?}"
        );
        assert!((weights.iter().sum::<Real>() - 1.0).abs() < 1e-6);
    }
}

#[test]
fn differential_stalled_contact_solves_stop_early_and_stay_feasible() {
    // Opposing toe, unequal side grip and a limited-slip rear axle make saturated
    // tires fight. The coupled residual then only creeps down between windows.
    let (mut controller, mut bodies, colliders) = four_wheel_test_vehicle(56.0, 1.0);
    locks(&mut controller, 0.55);
    set_test_drive(&mut controller, 300.0);
    for wheel in &mut controller.wheels {
        let side = wheel.chassis_connection_point_cs.x.signum();
        let toe = if wheel.role.axle == WheelAxle::Front {
            0.0005
        } else {
            0.0015
        } * side;
        wheel.wheel_axle_ws = Vector::new(toe.cos(), 0.0, -toe.sin());
        wheel.friction_slip = 1.0 + 0.02 * side;
        wheel.raycast_info.contact_point_ws.y = -0.3;
    }
    let mut stalled = 0;
    for _ in 0..240 {
        controller.current_vehicle_speed = bodies[controller.chassis].linvel().z;
        controller.update_friction(&mut bodies, &colliders, 1.0 / 60.0);
        let solver = &controller.contact_solver;
        stalled += solver.stalled_passes;
        for (i, prepared) in solver.contacts.iter().enumerate() {
            let base = prepared.base;
            let limit = base.grip_impulse * solver.actuations[i].grip;
            assert!(envelope_norm(solver.impulses[i], base.shape) <= limit + 0.001);
            assert!(
                solver.brakes[i].abs() <= base.brake_budget * solver.actuations[i].brake + 0.001
            );
        }
        assert!(bodies[controller.chassis]
            .linvel()
            .iter()
            .all(|v| v.is_finite()));
    }
    assert!(
        stalled > 0,
        "the stalled scenario must exercise the early stop"
    );
}
