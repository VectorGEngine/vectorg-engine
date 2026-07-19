use vectorg_engine::control::{
    DynamicRayCastVehicleController, VehicleControllerConfig, WheelAxle, WheelRole, WheelTuning,
};
use vectorg_engine::prelude::*;
use vectorg_engine_testbed_3d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let impulse_joints = ImpulseJointSet::new();
    let multibody_joints = MultibodyJointSet::new();

    let ground = ColliderBuilder::cuboid(50.0, 0.1, 50.0)
        .translation(vector![0.0, -0.1, 0.0])
        .friction(1.0)
        .material_name("tarmac".to_string());
    colliders.insert(ground);

    let chassis = RigidBodyBuilder::dynamic()
        .translation(vector![0.0, 0.9, 0.0])
        .can_sleep(false);
    let chassis_handle = bodies.insert(chassis);
    let chassis_collider = ColliderBuilder::cuboid(0.9, 0.3, 2.0)
        .mass(1230.0)
        .friction(0.0);
    colliders.insert_with_parent(chassis_collider, chassis_handle, &mut bodies);

    let mut config = VehicleControllerConfig::default();
    config.engine.horsepower = 590.0;
    config.engine.idle_rpm = 1000.0;
    config.engine.max_rpm = 8000.0;
    config.engine.rev_limit_rpm = 7900.0;
    config.engine.inertia = 0.899_999_976_158_142_1;
    config.engine.friction_torque = Some(70.0);
    config.engine.engine_braking = 0.2;
    config.engine.drivetrain_efficiency = 0.9;
    config.engine.force_scale = 0.6;
    config.engine.torque_curve = vec![
        (1000.0, 422.292),
        (2000.0, 506.974),
        (3000.0, 565.453),
        (4000.0, 590.0),
        (5000.0, 586.53),
        (6000.0, 564.822),
        (7000.0, 523.597),
        (8000.0, 460.2),
    ];
    config.transmission.reverse_ratio = -3.569_999_933_242_798;
    config.transmission.forward_ratios = vec![
        4.079_999_923_706_055,
        2.700_000_047_683_716,
        1.899_999_976_158_142,
        1.399_999_976_158_142,
        1.059_999_942_779_541,
        0.850_000_023_841_857_9,
    ];
    config.transmission.final_drive_ratio = 5.0;
    config.transmission.clutch_response = 12.0;
    config.transmission.upshift_range_position = 0.9;
    config.transmission.downshift_range_position = 0.7;
    config.transmission.automatic = true;
    config.transmission.auto_reverse = true;
    config.turbo.enabled = true;
    config.turbo.max_boost = 1.350_000_023_841_858;
    config.turbo.spool_rate = 1.0;
    config.dynamics.brake_bias = 0.6;
    config.dynamics.abs_strength = 1.0;
    config.dynamics.traction_control_strength = 1.0;
    config.dynamics.esc_strength = 0.0;
    config.dynamics.drag_coefficient = 0.5;
    config.dynamics.frontal_area = 2.1;
    config.dynamics.rolling_resistance = 0.014;
    config.dynamics.downforce_coefficient = 0.8;
    config.steering.max_angle = 50.0f32.to_radians();
    config.steering.speed_sensitivity = 32.0;
    config.steering.minimum_speed_factor = 0.3;

    let mut vehicle = DynamicRayCastVehicleController::new(chassis_handle, config);
    vehicle.index_forward_axis = 2;
    vehicle.index_up_axis = 1;
    vehicle.add_tire_type("road", 1.0);
    vehicle.add_surface_to_tire_type("road", "tarmac", 1.15);

    let tuning = WheelTuning {
        suspension_stiffness: 80.0,
        suspension_compression: 2.0,
        suspension_damping: 2.599_999_904_632_568_4,
        max_suspension_travel: 0.25,
        side_friction_stiffness: 1.1,
        friction_slip: 1.2,
        max_suspension_force: 20_000.0,
        tire_type: "road".to_string(),
    };
    let wheel_radius = 0.300_000_011_920_928_96;
    let suspension_rest_length = 0.35;
    let wheel_positions = [
        (point![0.78, -0.28, 1.25], WheelAxle::Front),
        (point![-0.78, -0.28, 1.25], WheelAxle::Front),
        (point![0.78, -0.28, -1.25], WheelAxle::Rear),
        (point![-0.78, -0.28, -1.25], WheelAxle::Rear),
    ];

    for (position, axle) in wheel_positions {
        let front = axle == WheelAxle::Front;
        let wheel = vehicle.add_wheel(
            position,
            -Vector::y(),
            -Vector::x(),
            suspension_rest_length,
            wheel_radius,
            &tuning,
            WheelRole::new(axle, true, front),
        );
        wheel.max_brake_force = 5000.0;
        wheel.anti_roll = 0.400_000_005_960_464_5;
        wheel.fwd_factor = 1.600_000_023_841_858;
        wheel.contact_damping = 0.150_000_005_960_464_48;
        wheel.base_contact_damping = wheel.contact_damping;
    }

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.set_vehicle_controller(vehicle);
    testbed.look_at(point![8.0, 5.0, -12.0], point![0.0, 0.5, 0.0]);
}
