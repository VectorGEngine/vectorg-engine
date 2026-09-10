use crate::dynamics::{RigidBody, RigidBodyHandle, RigidBodySet};
use crate::geometry::{ColliderHandle, ColliderSet, Ray};
use crate::math::{Point, Real, Rotation, Vector, DIM};
use crate::pipeline::{QueryFilter, QueryPipeline};
use crate::utils::SimdDot;
use std::collections::HashMap;

use super::vehicle_powertrain::{
    VehicleControllerConfig, VehicleEngineState, VehicleInput, VehiclePowertrain,
    VehicleShiftOutcome, VehicleState, WheelAxle, WheelRole,
};

const COUNTER_STEER_ASSIST_MIN_SPEED: Real = 5.0;
const COUNTER_STEER_ASSIST_FULL_SPEED: Real = 10.0;
const COUNTER_STEER_ASSIST_MIN_CONTACTS: usize = 2;
const COUNTER_STEER_ASSIST_ENTER_ANGLE: Real = 0.104_719_76; // 6 degrees.
const COUNTER_STEER_ASSIST_EXIT_ANGLE: Real = 0.052_359_88; // 3 degrees.
const COUNTER_STEER_ASSIST_FULL_ANGLE: Real = 0.349_065_84; // 20 degrees.
const COUNTER_STEER_ASSIST_RESPONSE: Real = 8.0;
const COUNTER_STEER_ASSIST_RELEASE_RESPONSE: Real = 15.0;
const COUNTER_STEER_ASSIST_YAW_DAMPING: Real = 0.08;
const COUNTER_STEER_ASSIST_INPUT_DEADZONE: Real = 0.01;
const WHEEL_REFERENCE_RADIUS: Real = 0.35;
const WHEEL_EFFECTIVE_INERTIA: Real = 1.5;
const WHEEL_STOP_EPSILON: Real = 1.0e-4;
// Numerical allowance for a rolling constraint, in meters per second.
const ASSIST_SURFACE_SPEED_TOLERANCE: Real = 0.01;
// Maximum powered wheel-surface overspeed in m/s; TC strength reduces this gap.
const TRACTION_CONTROL_MAX_SPEED_GAP: Real = 10.0;
const ESC_SIDESLIP_YAW_GAIN: Real = 2.0;

const SLIDING_START_SPEED: Real = 4.0;
const SLIDING_FULL_SPEED: Real = 8.0;
const RECOVERY_THRESHOLD: Real = 0.94;
const RECOVERY_BASE_RATE: Real = 0.15;
const RECOVERY_RATE: Real = 0.3;

fn counter_steer_assist_speed_activation(forward_speed: Real) -> Real {
    let normalized = ((forward_speed - COUNTER_STEER_ASSIST_MIN_SPEED)
        / (COUNTER_STEER_ASSIST_FULL_SPEED - COUNTER_STEER_ASSIST_MIN_SPEED))
        .clamp(0.0, 1.0);
    normalized * normalized * (3.0 - 2.0 * normalized)
}

fn curved_steering_input(input: Real, road_wheel_curve: Real) -> Real {
    let normalized = input.clamp(-1.0, 1.0);
    (1.0 - road_wheel_curve) * normalized + road_wheel_curve * normalized.powi(3)
}

/// A character controller to simulate vehicles using ray-casting for the wheels.
pub struct DynamicRayCastVehicleController {
    wheels: Vec<Wheel>,
    forward_ws: Vec<Vector<Real>>,
    axle: Vec<Vector<Real>>,
    /// The current forward speed of the vehicle.
    pub current_vehicle_speed: Real,
    /// Electronic stability control strength (`0.0` = off, `1.0` = full strength).
    pub esc: Real,

    /// Handle of the vehicle’s chassis.
    pub chassis: RigidBodyHandle,
    /// The chassis’ local _up_ direction (`0 = x, 1 = y, 2 = z`)
    pub index_up_axis: usize,
    /// The chassis’ local _forward_ direction (`0 = x, 1 = y, 2 = z`)
    pub index_forward_axis: usize,
    /// Available tire types
    pub tire_types: HashMap<String, TireType>,
    powertrain: VehiclePowertrain,
    last_steering_compression: Real,
    counter_steer_assist_active: bool,
    counter_steer_assist_offset: Real,
    counter_steer_assist_direction: Real,

    timer: Real,
    // Gravity is integrated with the vehicle impulses, then restored after the
    // world step. Keeping ownership here avoids changing the general solver.
    pending_gravity_scale: Option<Real>,
    pending_gravity_impulse: Vector<Real>,
}

#[derive(Clone, Debug, PartialEq)]
/// Parameters affecting the physical behavior of a wheel.
pub struct WheelTuning {
    /// The suspension stiffness.
    ///
    /// Increase this value if the suspension appears to not push the vehicle strong enough.
    pub suspension_stiffness: Real,
    /// The suspension’s damping when it is being compressed.
    pub suspension_compression: Real,
    /// The suspension’s damping when it is being released.
    ///
    /// Increase this value if the suspension appears to overshoot.
    pub suspension_damping: Real,
    /// The maximum distance the suspension can travel before and after its resting length.
    pub max_suspension_travel: Real,
    /// Parameter controlling how much traction the tire has.
    ///
    /// The larger the value, the more instantaneous braking will happen (with the risk of
    /// causing the vehicle to flip if it’s too strong).
    pub friction_slip: Real,
    /// The maximum force applied by the suspension.
    pub max_suspension_force: Real,
    /// The type of tire for friction calculations
    pub tire_type: String,
}

impl Default for WheelTuning {
    fn default() -> Self {
        Self {
            suspension_stiffness: 5.88,
            suspension_compression: 0.83,
            suspension_damping: 0.88,
            max_suspension_travel: 5.0,
            friction_slip: 10.5,
            max_suspension_force: 6000.0,
            tire_type: "default".to_string(),
        }
    }
}

/// Objects used to initialize a wheel.
struct WheelDesc {
    /// The position of the wheel, relative to the chassis.
    pub chassis_connection_cs: Point<Real>,
    /// The direction of the wheel’s suspension, relative to the chassis.
    ///
    /// The ray-casting will happen following this direction to detect the ground.
    pub direction_cs: Vector<Real>,
    /// The wheel’s axle axis, relative to the chassis.
    pub axle_cs: Vector<Real>,
    /// The rest length of the wheel’s suspension spring.
    pub suspension_rest_length: Real,
    /// The maximum distance the suspension can travel before and after its resting length.
    pub max_suspension_travel: Real,
    /// The wheel’s radius.
    pub radius: Real,

    /// The suspension stiffness.
    ///
    /// Increase this value if the suspension appears to not push the vehicle strong enough.
    pub suspension_stiffness: Real,
    /// The suspension’s damping when it is being compressed.
    pub damping_compression: Real,
    /// The suspension’s damping when it is being released.
    ///
    /// Increase this value if the suspension appears to overshoot.
    pub damping_relaxation: Real,
    /// Parameter controlling how much traction the tire has.
    ///
    /// The larger the value, the more instantaneous braking will happen (with the risk of
    /// causing the vehicle to flip if it’s too strong).
    pub friction_slip: Real,
    /// The maximum force applied by the suspension.
    pub max_suspension_force: Real,
    /// The type of tire for friction calculations
    pub tire_type: String,
    /// The wheel's role in the vehicle drivetrain.
    pub role: WheelRole,
}

#[derive(Clone, Debug, PartialEq)]
/// A wheel attached to a vehicle.
pub struct Wheel {
    raycast_info: RayCastInfo,

    center: Point<Real>,
    wheel_direction_ws: Vector<Real>,
    wheel_axle_ws: Vector<Real>,

    /// The position of the wheel, relative to the chassis.
    pub chassis_connection_point_cs: Point<Real>,
    /// The direction of the wheel’s suspension, relative to the chassis.
    ///
    /// The ray-casting will happen following this direction to detect the ground.
    pub direction_cs: Vector<Real>,
    /// The wheel’s axle axis, relative to the chassis.
    pub axle_cs: Vector<Real>,
    /// The rest length of the wheel’s suspension spring.
    pub suspension_rest_length: Real,
    /// The maximum distance the suspension can travel before and after its resting length.
    pub max_suspension_travel: Real,
    /// The wheel’s radius.
    pub radius: Real,
    /// The suspension stiffness.
    ///
    /// Increase this value if the suspension appears to not push the vehicle strong enough.
    pub suspension_stiffness: Real,
    /// The suspension’s damping when it is being compressed.
    pub damping_compression: Real,
    /// The suspension’s damping when it is being released.
    ///
    /// Increase this value if the suspension appears to overshoot.
    pub damping_relaxation: Real,
    /// Parameter controlling how much traction the tire has.
    ///
    /// The larger the value, the more instantaneous braking will happen (with the risk of
    /// causing the vehicle to flip if it’s too strong).
    pub friction_slip: Real,
    /// The wheel’s current rotation on its axle.
    pub rotation: Real,
    /// The change in rotation since the last update.
    pub delta_rotation: Real,
    /// The target angular velocity of the wheel.
    pub target_rotation: Real,
    angular_velocity: Real,
    angular_load: Real,
    drive_torque_transfer: Real,
    wheel_coupling_torque: Real,
    drive_throttle: Real,
    drivetrain_connected: bool,
    traction_control_cut: Real,
    abs_release: Real,
    handbrake_overrides_abs: bool,
    /// Fraction of the lateral impulse application height moved toward the chassis center of mass.
    pub anti_roll: Real,
    /// The maximum force applied by the suspension.
    pub max_suspension_force: Real,

    /// The forward impulses applied by the wheel on the chassis.
    pub forward_impulse: Real,
    /// The side impulses applied by the wheel on the chassis.
    pub side_impulse: Real,
    /// The braking impulse applied by this wheel on the chassis.
    pub brake_impulse: Real,

    /// The steering angle for this wheel.
    pub steering: Real,
    /// The forward force applied by this wheel on the chassis.
    pub engine_force: Real,
    /// The maximum brakking multiplier applied to this wheel.
    pub brake: Real,
    /// The maximum braking force applied by this wheel, in newtons.
    pub max_brake_force: Real,
    /// The anti-lock braking system strength applied to this wheel.
    pub anti_lock_brake: Real,
    /// traction control system force applied to this wheel.
    pub is_anti_lock_brake: bool,
    /// traction control system force applied to this wheel.
    pub traction_control: Real,
    /// The impulse applied from tire to engine
    pub engine_force_feedback: Real,
    lock: bool,

    clipped_inv_contact_dot_suspension: Real,
    suspension_relative_velocity: Real,
    contact_forward_speed: Real,
    contact_side_speed: Real,
    /// The force applied by the suspension.
    pub wheel_suspension_force: Real,
    /// Weighted contact-demand ratio for feedback and effects, using the selected
    /// brake constraint. One means demand fits the grip budget; zero means no
    /// grip/contact. This reporting ratio does not scale the contact impulses.
    pub skid_info: Real,
    last_skid_info: Real,
    /// Held sliding multiplier; one means no grip loss awaiting recovery.
    sliding_grip: Real,
    /// The ground friction multiplier for this wheel.
    pub ground_friction: Real,
    /// The type of ground this wheel is currently on.
    pub ground_type: String,
    /// The suspension compression ratio, where 1.0 means the suspension is at its rest length.
    pub suspension_compression_rate: Real,
    /// The type of tire for friction calculations
    pub tire_type: String,
    /// The wheel's role in the vehicle drivetrain.
    pub role: WheelRole,
}

impl Wheel {
    fn new(info: WheelDesc) -> Self {
        Self {
            raycast_info: RayCastInfo::default(),
            suspension_rest_length: info.suspension_rest_length,
            max_suspension_travel: info.max_suspension_travel,
            radius: info.radius,
            suspension_stiffness: info.suspension_stiffness,
            damping_compression: info.damping_compression,
            damping_relaxation: info.damping_relaxation,
            chassis_connection_point_cs: info.chassis_connection_cs,
            direction_cs: info.direction_cs,
            axle_cs: info.axle_cs,
            wheel_direction_ws: info.direction_cs,
            wheel_axle_ws: info.axle_cs,
            center: Point::origin(),
            friction_slip: info.friction_slip,
            steering: 0.0,
            engine_force: 0.0,
            rotation: 0.0,
            delta_rotation: 0.0,
            target_rotation: 0.0,
            angular_velocity: 0.0,
            angular_load: 0.0,
            drive_torque_transfer: 0.0,
            wheel_coupling_torque: 0.0,
            drive_throttle: 0.0,
            drivetrain_connected: false,
            traction_control_cut: 0.0,
            abs_release: 0.0,
            handbrake_overrides_abs: false,
            brake: 0.0,
            max_brake_force: 1000.0,
            anti_lock_brake: 0.0,
            is_anti_lock_brake: false,
            traction_control: 0.0,
            engine_force_feedback: 0.0,
            anti_roll: 0.0,
            clipped_inv_contact_dot_suspension: 0.0,
            suspension_relative_velocity: 0.0,
            contact_forward_speed: 0.0,
            contact_side_speed: 0.0,
            wheel_suspension_force: 0.0,
            max_suspension_force: info.max_suspension_force,
            skid_info: 0.0,
            last_skid_info: 0.0,
            sliding_grip: 1.0,
            side_impulse: 0.0,
            brake_impulse: 0.0,
            forward_impulse: 0.0,
            lock: false,
            tire_type: info.tire_type,
            suspension_compression_rate: 0.0,
            ground_friction: 1.0,
            ground_type: String::new(),
            role: info.role,
        }
    }

    fn reset(&mut self) {
        self.raycast_info = RayCastInfo::default();
        self.center = Point::origin();
        self.wheel_direction_ws = self.direction_cs;
        self.wheel_axle_ws = self.axle_cs;
        self.rotation = 0.0;
        self.delta_rotation = 0.0;
        self.angular_velocity = 0.0;
        self.angular_load = 0.0;
        self.drive_torque_transfer = 0.0;
        self.target_rotation = 0.0;
        self.wheel_coupling_torque = 0.0;
        self.drive_throttle = 0.0;
        self.drivetrain_connected = false;
        self.traction_control_cut = 0.0;
        self.abs_release = 0.0;
        self.handbrake_overrides_abs = false;
        self.forward_impulse = 0.0;
        self.side_impulse = 0.0;
        self.brake_impulse = 0.0;
        self.steering = 0.0;
        self.engine_force = 0.0;
        self.brake = 0.0;
        self.is_anti_lock_brake = false;
        self.engine_force_feedback = 0.0;
        self.lock = false;
        self.clipped_inv_contact_dot_suspension = 0.0;
        self.suspension_relative_velocity = 0.0;
        self.contact_forward_speed = 0.0;
        self.contact_side_speed = 0.0;
        self.wheel_suspension_force = 0.0;
        self.skid_info = 0.0;
        self.last_skid_info = 0.0;
        self.sliding_grip = 1.0;
        self.ground_friction = 1.0;
        self.ground_type.clear();
        self.suspension_compression_rate = 0.0;
    }

    /// Information about suspension and the ground obtained from the ray-casting
    /// for this wheel.
    pub fn raycast_info(&self) -> &RayCastInfo {
        &self.raycast_info
    }

    /// The world-space center of the wheel.
    pub fn center(&self) -> Point<Real> {
        self.center
    }

    /// The world-space direction of the wheel’s suspension.
    pub fn suspension(&self) -> Vector<Real> {
        self.wheel_direction_ws
    }

    /// The world-space direction of the wheel’s axle.
    pub fn axle(&self) -> Vector<Real> {
        self.wheel_axle_ws
    }
}

/// Information about suspension and the ground obtained from the ray-casting
/// to simulate a wheel’s suspension.
#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub struct RayCastInfo {
    /// The (world-space) contact normal between the wheel and the floor.
    pub contact_normal_ws: Vector<Real>,
    /// The (world-space) point hit by the wheel’s ray-cast.
    pub contact_point_ws: Point<Real>,
    /// The suspension length for the wheel.
    pub suspension_length: Real,
    /// The (world-space) starting point of the ray-cast.
    pub hard_point_ws: Point<Real>,
    /// Is the wheel in contact with the ground?
    pub is_in_contact: bool,
    /// The collider hit by the ray-cast.
    pub ground_object: Option<ColliderHandle>,
}

#[derive(Clone)]
struct WheelContactState {
    is_grounded: bool,
    ground_object: Option<ColliderHandle>,
    forward_dir: Vector<Real>,
    forward_speed: Real,
    friction: TireFriction,
    kinetic_grip: Real,
    peak_friction_limit: Real,
}

impl Default for WheelContactState {
    fn default() -> Self {
        Self {
            is_grounded: false,
            ground_object: None,
            forward_dir: Vector::zeros(),
            forward_speed: 0.0,
            friction: TireFriction::new(0.0, 0.0),
            kinetic_grip: 1.0,
            peak_friction_limit: 0.0,
        }
    }
}

#[derive(Copy, Clone, Debug)]
struct EscIntervention {
    activity: Real,
    engine_cut: Real,
    brake_strength: Real,
    brake_axle: Option<WheelAxle>,
    brake_side: Real,
}

impl Default for EscIntervention {
    fn default() -> Self {
        Self {
            activity: 0.0,
            engine_cut: 0.0,
            brake_strength: 0.0,
            brake_axle: None,
            brake_side: 0.0,
        }
    }
}

fn aligned_wheel_forward(
    contact_normal: &Vector<Real>,
    side_dir: &Vector<Real>,
    chassis_forward: &Vector<Real>,
) -> Vector<Real> {
    let mut forward = contact_normal
        .cross(side_dir)
        .try_normalize(1.0e-5)
        .unwrap_or_else(Vector::zeros);

    if forward.dot(chassis_forward) < 0.0 {
        forward = -forward;
    }

    forward
}

#[cfg(feature = "dim3")]
fn steering_positive_side(
    contact_normal: &Vector<Real>,
    wheel_forward: &Vector<Real>,
) -> Vector<Real> {
    contact_normal
        .cross(wheel_forward)
        .try_normalize(1.0e-5)
        .unwrap_or_else(Vector::zeros)
}

#[cfg(feature = "dim2")]
fn steering_positive_side(
    _contact_normal: &Vector<Real>,
    wheel_forward: &Vector<Real>,
) -> Vector<Real> {
    Vector::new(wheel_forward.y, -wheel_forward.x)
}

fn wheel_angular_inertia(radius: Real) -> Real {
    let radius_scale = radius.max(0.01) / WHEEL_REFERENCE_RADIUS;
    (WHEEL_EFFECTIVE_INERTIA * radius_scale * radius_scale).max(Real::EPSILON)
}

// Solve rolling contact and the bounded brake torque together. A brake that
// holds the wheel locked removes its rotational freedom from the contact solve.
#[cfg(test)]
fn braked_contact_impulses(
    angular_velocity: Real,
    road_speed: Real,
    radius: Real,
    inertia: Real,
    contact_inverse_mass: Real,
    brake_angular_impulse: Real,
) -> (Real, Real) {
    let coupled_inverse_mass = contact_inverse_mass + radius * radius / inertia;
    let forward = (angular_velocity * radius - road_speed) / coupled_inverse_mass;
    if brake_angular_impulse <= 0.0 {
        return (forward, 0.0);
    }
    if contact_inverse_mass > Real::EPSILON {
        let locked_impulse = -road_speed / contact_inverse_mass;
        let holding_impulse = inertia * angular_velocity - radius * locked_impulse;
        if holding_impulse.abs() <= brake_angular_impulse {
            return (forward, locked_impulse - forward);
        }
    }
    // With insufficient holding torque, oppose the wheel's post-contact motion.
    let motion = angular_velocity * contact_inverse_mass + road_speed * radius / inertia;
    let direction = if motion == 0.0 { 0.0 } else { motion.signum() };
    let braking = -direction * brake_angular_impulse * radius / inertia / coupled_inverse_mass;
    (forward, braking)
}

fn apply_opposing_angular_impulse(
    angular_velocity: &mut Real,
    reference_angular_velocity: Real,
    inertia: Real,
    angular_impulse: Real,
) -> Real {
    let direction = if angular_velocity.abs() > WHEEL_STOP_EPSILON {
        angular_velocity.signum()
    } else {
        reference_angular_velocity.signum()
    };
    if direction == 0.0 || angular_impulse <= 0.0 {
        return angular_impulse.max(0.0);
    }

    let stopping_impulse = angular_velocity.abs() * inertia;
    let applied_impulse = angular_impulse.min(stopping_impulse);
    *angular_velocity -= direction * applied_impulse / inertia;
    if angular_velocity.abs() <= WHEEL_STOP_EPSILON {
        *angular_velocity = 0.0;
    }
    angular_impulse - applied_impulse
}

fn update_wheel_rotation(wheel: &mut Wheel, dt: Real) {
    if wheel.angular_velocity.abs() <= WHEEL_STOP_EPSILON {
        wheel.angular_velocity = 0.0;
    }
    wheel.rotation += wheel.angular_velocity * dt;
    wheel.delta_rotation = wheel.angular_velocity * dt;
}

fn anti_roll_bar_transfer(
    left_compression: Real,
    right_compression: Real,
    stiffness: Real,
    chassis_mass: Real,
    left_force: Real,
    right_force: Real,
    left_max_force: Real,
    right_max_force: Real,
) -> Real {
    let maximum_leftward_transfer = left_force.min((right_max_force - right_force).max(0.0));
    let maximum_rightward_transfer = right_force.min((left_max_force - left_force).max(0.0));
    ((left_compression - right_compression) * stiffness * chassis_mass)
        .clamp(-maximum_leftward_transfer, maximum_rightward_transfer)
}

fn assist_torque_fraction(strength: Real, slip_at_fraction: impl Fn(Real) -> Real) -> Real {
    let strength = strength.clamp(0.0, 1.0);
    if strength == 0.0 {
        return 1.0;
    }
    let error = |fraction| slip_at_fraction(fraction) - ASSIST_SURFACE_SPEED_TOLERANCE;
    let controlled_fraction = if error(1.0) <= 0.0 {
        1.0
    } else if error(0.0) > 0.0 {
        // Tire reaction may need multiple steps to recover existing slip.
        0.0
    } else {
        let mut allowed = 0.0;
        let mut rejected = 1.0;
        for _ in 0..24 {
            let candidate = (allowed + rejected) * 0.5;
            if error(candidate) <= 0.0 {
                allowed = candidate;
            } else {
                rejected = candidate;
            }
        }
        allowed
    };
    // Strength scales the required correction, not a slip target or response time.
    // No previous actuator value feeds this solve: weak assistance cannot accumulate
    // into full intervention, even when the wheel remains spinning or locked.
    1.0 - strength * (1.0 - controlled_fraction)
}

fn traction_control_torque_fraction(
    strength: Real,
    slip_at_fraction: impl Fn(Real) -> Real,
) -> Real {
    let strength = strength.clamp(0.0, 1.0);
    if strength == 0.0 {
        return 1.0;
    }
    let allowed_gap = TRACTION_CONTROL_MAX_SPEED_GAP * (1.0 - strength);
    // Reuse the full correction solve, shifting its target to the allowed gap.
    assist_torque_fraction(1.0, |fraction| slip_at_fraction(fraction) - allowed_gap)
}

fn anti_lock_brake_torque_fraction(
    strength: Real,
    direction: Real,
    speeds_at_fraction: impl Fn(Real) -> (Real, Real),
) -> Real {
    let strength = strength.clamp(0.0, 1.0);
    if strength == 0.0 {
        return 1.0;
    }
    // Strength sets permitted wheel underspeed relative to road speed, not a
    // brake-release cap. Every enabled level can fully release an existing lock.
    let allowed_slip_ratio = 1.0 - strength;
    assist_torque_fraction(1.0, |fraction| {
        let (wheel_speed, road_speed) = speeds_at_fraction(fraction);
        (road_speed - wheel_speed) * direction - road_speed.abs() * allowed_slip_ratio
    })
}

impl DynamicRayCastVehicleController {
    /// Creates a new vehicle represented by the given rigid-body.
    ///
    /// Wheels have to be attached afterwards calling [`Self::add_wheel`].
    pub fn new(chassis: RigidBodyHandle, config: VehicleControllerConfig) -> Self {
        let mut tire_types = HashMap::new();

        // Create default tire types
        tire_types.insert(
            "default".to_string(),
            TireType::new("default", TireFriction::new(1.0, 0.85)),
        );

        Self {
            wheels: vec![],
            forward_ws: vec![],
            axle: vec![],
            current_vehicle_speed: 0.0,
            esc: config.dynamics.esc_strength,
            chassis,
            index_up_axis: 1,
            index_forward_axis: 0,
            tire_types,
            powertrain: VehiclePowertrain::new(config),
            last_steering_compression: 0.0,
            counter_steer_assist_active: false,
            counter_steer_assist_offset: 0.0,
            counter_steer_assist_direction: 0.0,
            timer: 0.0,
            pending_gravity_scale: None,
            pending_gravity_impulse: Vector::zeros(),
        }
    }

    /// Sets the normalized driver inputs consumed by the next vehicle update.
    pub fn set_input(&mut self, input: VehicleInput) {
        self.powertrain.set_input(input);
    }

    /// Requests the existing starter sequence on the next update if the engine is stopped.
    pub fn start_engine(&mut self) {
        self.powertrain.start_engine();
    }

    /// The normalized driver inputs currently held by the controller.
    pub fn input(&self) -> VehicleInput {
        self.powertrain.input()
    }

    /// The current engine, transmission, and vehicle output state.
    pub fn state(&self) -> VehicleState {
        self.powertrain.state()
    }

    /// The current discrete engine lifecycle state.
    pub fn engine_state(&self) -> VehicleEngineState {
        self.powertrain.engine_state()
    }

    /// Restores all transient simulation state while preserving vehicle configuration and tuning.
    pub fn reset(&mut self) {
        self.powertrain.reset();
        self.current_vehicle_speed = 0.0;
        self.forward_ws.clear();
        self.axle.clear();
        self.last_steering_compression = 0.0;
        self.counter_steer_assist_active = false;
        self.counter_steer_assist_offset = 0.0;
        self.counter_steer_assist_direction = 0.0;
        self.timer = 0.0;
        for wheel in &mut self.wheels {
            wheel.reset();
        }
    }

    /// Requests the next higher gear.
    pub fn shift_up(&mut self) -> VehicleShiftOutcome {
        self.powertrain.shift_up()
    }

    /// Requests the next lower gear.
    pub fn shift_down(&mut self) -> VehicleShiftOutcome {
        self.powertrain.shift_down()
    }

    /// Selects a specific gear, where -1 is reverse and 0 is neutral.
    pub fn set_gear(&mut self, gear: i32) -> VehicleShiftOutcome {
        self.powertrain.set_gear(gear)
    }

    /// Enables or disables all steering assistance, including speed-sensitive
    /// steering range reduction and velocity-based counter-steering.
    pub fn set_steering_assist(&mut self, enabled: bool) {
        self.powertrain.config.steering.assist = enabled;

        if !enabled {
            self.counter_steer_assist_active = false;
            self.counter_steer_assist_offset = 0.0;
            self.counter_steer_assist_direction = 0.0;
        }
    }

    /// Sets the minimum steering range factor at high speed (`0.0` to `1.0`).
    pub fn set_minimum_speed_factor(&mut self, factor: Real) {
        if factor.is_finite() {
            self.powertrain.config.steering.minimum_speed_factor = factor.clamp(0.0, 1.0);
        }
    }

    /// Sets counter-steer assistance strength (`0.0` = none, `1.0` = full assistance).
    pub fn set_counter_steer_assist(&mut self, strength: Real) {
        self.powertrain.config.steering.counter_steer_assist = strength.clamp(0.0, 1.0);

        if self.powertrain.config.steering.counter_steer_assist <= Real::EPSILON {
            self.counter_steer_assist_active = false;
            self.counter_steer_assist_offset = 0.0;
            self.counter_steer_assist_direction = 0.0;
        }
    }

    /// Adds a new tire type to the controller
    pub fn add_tire_type(&mut self, tire_type: &str, peak: Real, sliding: Real) {
        self.tire_types.insert(
            tire_type.to_string(),
            TireType::new(tire_type, TireFriction::new(peak, sliding)),
        );
    }

    /// Adds a surface to an existing tire type
    pub fn remove_tire_type(&mut self, name: &str) {
        self.tire_types.remove(name);
    }

    /// Gets a reference to a tire type by name
    pub fn get_tire_type(&self, name: &str) -> Option<&TireType> {
        self.tire_types.get(name)
    }

    /// Gets a mutable reference to a tire type by name
    pub fn get_tire_type_mut(&mut self, name: &str) -> Option<&mut TireType> {
        self.tire_types.get_mut(name)
    }

    /// Sets the tire type for a specific wheel
    pub fn set_wheel_tire_type(&mut self, wheel_index: usize, tire_type_name: &str) {
        if let Some(wheel) = self.wheels.get_mut(wheel_index) {
            if self.tire_types.contains_key(tire_type_name) {
                wheel.tire_type = tire_type_name.to_string();
            }
        }
    }

    /// Gets all available tire type names
    pub fn get_tire_type_names(&self) -> Vec<&String> {
        self.tire_types.keys().collect()
    }

    fn side_axis(&self) -> usize {
        for axis in 0..DIM {
            if axis != self.index_forward_axis && axis != self.index_up_axis {
                return axis;
            }
        }

        for axis in 0..DIM {
            if axis != self.index_forward_axis {
                return axis;
            }
        }

        self.index_forward_axis
    }

    fn chassis_yaw_rate(&self, chassis: &RigidBody) -> Real {
        let up = chassis.position().rotation * Vector::ith(self.index_up_axis, 1.0);
        chassis.angvel().dot(&up)
    }

    fn esc_intervention(&self, chassis: &RigidBody) -> EscIntervention {
        let esc = self.esc.clamp(0.0, 1.0);
        let speed = self.current_vehicle_speed;

        if esc == 0.0 || speed.abs() <= 1.0 {
            return EscIntervention::default();
        }

        let mut steering = 0.0;
        let mut num_steered_wheels = 0;
        let mut min_forward = Real::MAX;
        let mut max_forward = -Real::MAX;

        for wheel in &self.wheels {
            if wheel.steering.abs() > Real::EPSILON {
                steering += wheel.steering;
                num_steered_wheels += 1;
            }

            let forward = wheel.chassis_connection_point_cs.coords[self.index_forward_axis];
            min_forward = min_forward.min(forward);
            max_forward = max_forward.max(forward);
        }

        if num_steered_wheels > 0 {
            steering /= num_steered_wheels as Real;
        }

        let wheelbase = (max_forward - min_forward).abs().max(1.0);
        let desired_yaw_rate = speed * steering.tan() / wheelbase;
        let actual_yaw_rate = self.chassis_yaw_rate(chassis);
        let rotation = chassis.position().rotation;
        let local_up = Vector::ith(self.index_up_axis, 1.0);
        let local_forward = Vector::ith(self.index_forward_axis, 1.0);
        let local_positive_yaw_side = local_up
            .cross(&local_forward)
            .try_normalize(1.0e-5)
            .unwrap_or_else(Vector::zeros);
        let positive_yaw_side = rotation * local_positive_yaw_side;
        let side_speed = positive_yaw_side.dot(chassis.linvel());
        let sideslip_angle = side_speed.atan2(speed.abs().max(1.0));
        let yaw_error = desired_yaw_rate - actual_yaw_rate;
        let correction_error = yaw_error + sideslip_angle * ESC_SIDESLIP_YAW_GAIN;
        let is_understeer = desired_yaw_rate.abs() > Real::EPSILON
            && desired_yaw_rate * actual_yaw_rate >= 0.0
            && yaw_error * desired_yaw_rate > 0.0
            && correction_error * yaw_error > 0.0;
        let control_error = if is_understeer {
            yaw_error
        } else {
            correction_error
        };
        let yaw_factor = ((control_error.abs() - 0.14) / 0.75).clamp(0.0, 1.0);
        let steering_factor = (steering.abs() / 0.55).clamp(0.0, 1.0);
        let speed_factor = ((speed.abs() - 2.0) / 10.0).clamp(0.0, 1.0);
        let mode_factor = if is_understeer { steering_factor } else { 1.0 };
        let strength = esc * yaw_factor * speed_factor * mode_factor;

        if strength == 0.0 {
            return EscIntervention::default();
        }

        let engine_cut = strength * 0.45;
        let brake_strength = strength * 0.45;
        let (brake_axle, brake_direction) = if is_understeer {
            // Understeer: create yaw with the inside rear wheel.
            (WheelAxle::Rear, desired_yaw_rate)
        } else {
            // Oversteer, counter-yaw, or a straight-line spin: stabilize with a front wheel.
            (WheelAxle::Front, correction_error)
        };
        let side_axis = self.side_axis();
        let side_orientation = local_positive_yaw_side[side_axis].signum();
        let brake_side = brake_direction.signum() * speed.signum() * side_orientation;

        EscIntervention {
            activity: strength,
            engine_cut,
            brake_strength,
            brake_axle: Some(brake_axle),
            brake_side,
        }
    }

    /// Adds a surface to an existing tire type
    pub fn add_surface_to_tire_type(
        &mut self,
        tire_type_name: &str,
        surface_name: &str,
        peak: Real,
        sliding: Real,
    ) {
        if let Some(tire_type) = self.tire_types.get_mut(tire_type_name) {
            tire_type.add_surface(surface_name, TireFriction::new(peak, sliding));
        }
    }

    //
    // basically most of the code is general for 2 or 4 wheel vehicles, but some of it needs to be reviewed
    //
    /// Adds a wheel to this vehicle.
    pub fn add_wheel(
        &mut self,
        chassis_connection_cs: Point<Real>,
        direction_cs: Vector<Real>,
        axle_cs: Vector<Real>,
        suspension_rest_length: Real,
        radius: Real,
        tuning: &WheelTuning,
        role: WheelRole,
    ) -> &mut Wheel {
        let ci = WheelDesc {
            chassis_connection_cs,
            direction_cs,
            axle_cs,
            suspension_rest_length,
            radius,
            suspension_stiffness: tuning.suspension_stiffness,
            damping_compression: tuning.suspension_compression,
            damping_relaxation: tuning.suspension_damping,
            friction_slip: tuning.friction_slip,
            max_suspension_travel: tuning.max_suspension_travel,
            max_suspension_force: tuning.max_suspension_force,
            tire_type: tuning.tire_type.clone(),
            role,
        };

        let wheel_id = self.wheels.len();
        let mut wheel = Wheel::new(ci);
        wheel.anti_lock_brake = self.powertrain.config.dynamics.abs_strength.clamp(0.0, 1.0);
        wheel.traction_control = self
            .powertrain
            .config
            .dynamics
            .traction_control_strength
            .clamp(0.0, 1.0);
        self.wheels.push(wheel);

        &mut self.wheels[wheel_id]
    }

    #[cfg(feature = "dim2")]
    fn update_wheel_transform(&mut self, chassis: &RigidBody, wheel_index: usize) {
        self.update_wheel_transforms_ws(chassis, wheel_index);
        let wheel = &mut self.wheels[wheel_index];
        wheel.center = (wheel.raycast_info.hard_point_ws
            + wheel.wheel_direction_ws * wheel.raycast_info.suspension_length)
            .coords;
    }

    #[cfg(feature = "dim3")]
    fn update_wheel_transform(&mut self, chassis: &RigidBody, wheel_index: usize) {
        self.update_wheel_transforms_ws(chassis, wheel_index);
        let wheel = &mut self.wheels[wheel_index];

        let steering_orn = Rotation::new(-wheel.wheel_direction_ws * wheel.steering);
        wheel.wheel_axle_ws = steering_orn * (chassis.position() * wheel.axle_cs);
        wheel.center = wheel.raycast_info.hard_point_ws
            + wheel.wheel_direction_ws * wheel.raycast_info.suspension_length;
    }

    fn update_wheel_transforms_ws(&mut self, chassis: &RigidBody, wheel_id: usize) {
        let wheel = &mut self.wheels[wheel_id];
        wheel.raycast_info.is_in_contact = false;

        let chassis_transform = chassis.position();

        wheel.raycast_info.hard_point_ws = chassis_transform * wheel.chassis_connection_point_cs;
        wheel.wheel_direction_ws = chassis_transform * wheel.direction_cs;
        wheel.wheel_axle_ws = chassis_transform * wheel.axle_cs;
    }

    #[profiling::function]
    fn ray_cast(
        &mut self,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        queries: &QueryPipeline,
        filter: QueryFilter,
        chassis: &RigidBody,
        wheel_id: usize,
    ) {
        let wheel = &mut self.wheels[wheel_id];
        let raylen = wheel.suspension_rest_length + wheel.radius;
        let rayvector = wheel.wheel_direction_ws * raylen;
        let source = wheel.raycast_info.hard_point_ws;
        wheel.raycast_info.contact_point_ws = source + rayvector;
        let ray = Ray::new(source, rayvector);
        let hit = queries.cast_ray_and_get_normal(bodies, colliders, &ray, 1.0, true, filter);

        wheel.raycast_info.ground_object = None;

        if let Some((collider_hit, mut hit)) = hit {
            if hit.time_of_impact == 0.0 {
                let collider = &colliders[collider_hit];
                let up_ray = Ray::new(source + rayvector, -rayvector);
                if let Some(hit2) =
                    collider
                        .shape
                        .cast_ray_and_get_normal(collider.position(), &up_ray, 1.0, false)
                {
                    hit.normal = -hit2.normal;
                }

                if hit.normal == Vector::zeros() {
                    // If the hit is still not defined, set the normal.
                    hit.normal = -wheel.wheel_direction_ws;
                }
            }

            wheel.raycast_info.contact_normal_ws = hit.normal;
            wheel.raycast_info.is_in_contact = true;
            wheel.raycast_info.ground_object = Some(collider_hit);

            let hit_distance = hit.time_of_impact * raylen;
            wheel.raycast_info.suspension_length = hit_distance - wheel.radius;

            // clamp on max suspension travel
            let min_suspension_length = wheel.suspension_rest_length - wheel.max_suspension_travel;
            let max_suspension_length = wheel.suspension_rest_length + wheel.max_suspension_travel;
            wheel.raycast_info.suspension_length = wheel
                .raycast_info
                .suspension_length
                .clamp(min_suspension_length, max_suspension_length);
            wheel.raycast_info.contact_point_ws = ray.point_at(hit.time_of_impact);

            let denominator = wheel
                .raycast_info
                .contact_normal_ws
                .dot(&wheel.wheel_direction_ws);
            let chassis_velocity_at_contact_point =
                chassis.velocity_at_point(&wheel.raycast_info.contact_point_ws);
            let proj_vel = wheel
                .raycast_info
                .contact_normal_ws
                .dot(&chassis_velocity_at_contact_point);

            if denominator >= -0.1 {
                wheel.suspension_relative_velocity = 0.0;
                wheel.clipped_inv_contact_dot_suspension = 1.0 / 0.1;
            } else {
                let inv = -1.0 / denominator;
                wheel.suspension_relative_velocity = proj_vel * inv;
                wheel.clipped_inv_contact_dot_suspension = inv;
            }
        } else {
            // No contact, put wheel info as in rest position
            wheel.raycast_info.suspension_length = wheel.suspension_rest_length;
            wheel.suspension_relative_velocity = 0.0;
            wheel.raycast_info.contact_normal_ws = -wheel.wheel_direction_ws;
            wheel.clipped_inv_contact_dot_suspension = 1.0;
        }
    }

    fn driven_wheel_speed_and_radius(&self) -> (Real, Real) {
        let mut driven_speed: Real = 0.0;
        let mut radius_sum: Real = 0.0;
        let mut driven_count: usize = 0;

        for wheel in &self.wheels {
            if wheel.role.driven {
                let speed = wheel.angular_velocity * wheel.radius;
                if speed.abs() > driven_speed.abs() {
                    driven_speed = speed;
                }
                radius_sum += wheel.radius;
                driven_count += 1;
            }
        }

        let average_radius = if driven_count == 0 {
            0.35
        } else {
            radius_sum / driven_count as Real
        };
        (driven_speed, average_radius)
    }

    fn update_powertrain(&mut self, dt: Real) -> super::vehicle_powertrain::PowertrainOutput {
        let (speed, radius) = self.driven_wheel_speed_and_radius();
        let driven_count = self.wheels.iter().filter(|w| w.role.driven).count();
        // Match the wheel used for the existing fastest-driven-wheel RPM signal.
        let wheel = self.wheels.iter().filter(|w| w.role.driven).reduce(|a, b| {
            if (b.angular_velocity * b.radius).abs() > (a.angular_velocity * a.radius).abs() {
                b
            } else {
                a
            }
        });
        let (inverse_inertia, load_acceleration, drive_fraction) = wheel
            .map(|w| {
                let scale = w.radius / radius / wheel_angular_inertia(w.radius);
                (
                    scale / driven_count as Real,
                    // The base coupling response uses an equal torque split.
                    // Account for the additive torque actually transferred to
                    // this wheel, so it is not mistaken for extra clutch load.
                    (w.angular_load - w.drive_torque_transfer) * scale,
                    if w.traction_control > 0.0 {
                        1.0 - w.traction_control_cut
                    } else {
                        1.0
                    },
                )
            })
            .unwrap_or((0.0, 0.0, 1.0));
        self.powertrain.update_with_wheel_dynamics(
            dt,
            self.current_vehicle_speed,
            speed,
            radius,
            inverse_inertia,
            load_acceleration,
            drive_fraction,
        )
    }

    fn update_steering(&mut self, chassis: &RigidBody, dt: Real) {
        let steering_config = &self.powertrain.config.steering;
        let input = self.powertrain.input();
        let assist_enabled = steering_config.assist;
        let speed_factor = if !assist_enabled || steering_config.speed_sensitivity <= Real::EPSILON
        {
            1.0
        } else {
            let normalized = (self.current_vehicle_speed.abs() / steering_config.speed_sensitivity)
                .clamp(0.0, 1.0);
            steering_config.minimum_speed_factor
                + (1.0 - normalized).powi(2) * (1.0 - steering_config.minimum_speed_factor)
        };
        let max_angle = steering_config.max_angle;
        let normalized_input = input.steering.clamp(-1.0, 1.0);
        let driver_steering_angle = normalized_input * max_angle * speed_factor;
        let curved_input =
            curved_steering_input(normalized_input, steering_config.road_wheel_curve);
        let player_angle = curved_input * max_angle * speed_factor;
        let counter_steer_strength = steering_config.counter_steer_assist.clamp(0.0, 1.0);
        let assist_speed_activation =
            counter_steer_assist_speed_activation(self.current_vehicle_speed);
        let mut target_assist_offset = None;
        let mut cancel_immediately = false;
        let grounded =
            self.powertrain.state().wheels_in_contact >= COUNTER_STEER_ASSIST_MIN_CONTACTS;
        let can_assist = assist_enabled
            && counter_steer_strength > Real::EPSILON
            && assist_speed_activation > 0.0
            && grounded;

        let mut drift_angle = None;
        if can_assist {
            let up = chassis.position().rotation * Vector::ith(self.index_up_axis, 1.0);
            let forward = chassis.position().rotation * Vector::ith(self.index_forward_axis, 1.0);
            let mut velocity = *chassis.linvel();
            velocity -= up * velocity.dot(&up);

            if let Some(velocity_dir) = velocity.try_normalize(Real::EPSILON) {
                drift_angle = Some(
                    up.dot(&velocity_dir.cross(&forward))
                        .atan2(velocity_dir.dot(&forward)),
                );
            }
        }

        if let Some(angle) = drift_angle {
            let absolute_angle = angle.abs();
            self.counter_steer_assist_active = if self.counter_steer_assist_active {
                absolute_angle > COUNTER_STEER_ASSIST_EXIT_ANGLE
            } else {
                absolute_angle > COUNTER_STEER_ASSIST_ENTER_ANGLE
            };

            if self.counter_steer_assist_active {
                let yaw_rate = self.chassis_yaw_rate(chassis);
                let correction_angle = (-angle - yaw_rate * COUNTER_STEER_ASSIST_YAW_DAMPING)
                    .clamp(-max_angle, max_angle);
                self.counter_steer_assist_direction = correction_angle.signum();
                let matching_input = input.steering.abs() > COUNTER_STEER_ASSIST_INPUT_DEADZONE
                    && input.steering * correction_angle > 0.0;

                if matching_input {
                    let normalized_angle = ((absolute_angle - COUNTER_STEER_ASSIST_ENTER_ANGLE)
                        / (COUNTER_STEER_ASSIST_FULL_ANGLE - COUNTER_STEER_ASSIST_ENTER_ANGLE))
                        .clamp(0.0, 1.0);
                    let activation = normalized_angle
                        * normalized_angle
                        * (3.0 - 2.0 * normalized_angle)
                        * assist_speed_activation;
                    target_assist_offset = Some(
                        (correction_angle - player_angle) * counter_steer_strength * activation,
                    );
                } else if input.steering.abs() > COUNTER_STEER_ASSIST_INPUT_DEADZONE {
                    cancel_immediately = true;
                }
            }
        } else {
            self.counter_steer_assist_active = false;
        }

        if input.steering.abs() > COUNTER_STEER_ASSIST_INPUT_DEADZONE
            && self.counter_steer_assist_direction != 0.0
            && input.steering * self.counter_steer_assist_direction < 0.0
        {
            cancel_immediately = true;
        }

        if cancel_immediately {
            self.counter_steer_assist_offset = 0.0;
            self.counter_steer_assist_direction = 0.0;
        } else {
            let target_offset = target_assist_offset.unwrap_or(0.0);
            let response_rate = if target_offset.abs() < self.counter_steer_assist_offset.abs() {
                COUNTER_STEER_ASSIST_RELEASE_RESPONSE
            } else {
                COUNTER_STEER_ASSIST_RESPONSE
            };
            let response = 1.0 - (-response_rate * dt.max(0.0)).exp();
            self.counter_steer_assist_offset +=
                (target_offset - self.counter_steer_assist_offset) * response;

            if target_assist_offset.is_none() && self.counter_steer_assist_offset.abs() <= 1.0e-4 {
                self.counter_steer_assist_offset = 0.0;
                self.counter_steer_assist_direction = 0.0;
            }
        }

        let mut center_angle = player_angle + self.counter_steer_assist_offset;

        center_angle = center_angle.clamp(-max_angle, max_angle);
        let state = self.powertrain.state_mut();
        state.driver_steering_angle = driver_steering_angle;
        state.steering_angle = center_angle;

        let side_axis = self.side_axis();
        let mut steered_forward_sum = 0.0;
        let mut steered_count = 0;
        let mut fixed_forward_sum = 0.0;
        let mut fixed_count = 0;
        let mut min_side = Real::MAX;
        let mut max_side = -Real::MAX;

        for wheel in &self.wheels {
            let forward = wheel.chassis_connection_point_cs.coords[self.index_forward_axis];
            if wheel.role.steered {
                steered_forward_sum += forward;
                steered_count += 1;
                let side = wheel.chassis_connection_point_cs.coords[side_axis];
                min_side = min_side.min(side);
                max_side = max_side.max(side);
            } else {
                fixed_forward_sum += forward;
                fixed_count += 1;
            }
        }

        if steered_count == 0 {
            return;
        }

        let wheelbase = if fixed_count == 0 {
            1.0
        } else {
            ((steered_forward_sum / steered_count as Real)
                - (fixed_forward_sum / fixed_count as Real))
                .abs()
                .max(0.1)
        };
        let track_width = if min_side < max_side {
            max_side - min_side
        } else {
            0.0
        };

        for wheel in &mut self.wheels {
            if !wheel.role.steered {
                wheel.steering = 0.0;
                continue;
            }

            if center_angle.abs() <= 1.0e-4 || track_width <= Real::EPSILON {
                wheel.steering = center_angle;
                continue;
            }

            let turn_radius = wheelbase / center_angle.abs().tan().max(1.0e-4);
            let side = wheel.chassis_connection_point_cs.coords[side_axis];
            let inner_wheel = side.signum() == center_angle.signum();
            let wheel_radius = if inner_wheel {
                (turn_radius - track_width * 0.5).max(0.05)
            } else {
                turn_radius + track_width * 0.5
            };
            wheel.steering = center_angle.signum() * (wheelbase / wheel_radius).atan();
        }
    }

    fn apply_powertrain_output(&mut self, output: super::vehicle_powertrain::PowertrainOutput) {
        let input = self.powertrain.input();
        let dynamics = &self.powertrain.config.dynamics;
        let driven_count = self.wheels.iter().filter(|wheel| wheel.role.driven).count();
        let driven_divisor = driven_count.max(1) as Real;
        let motion_sign = if self.current_vehicle_speed.abs() > 0.1 {
            self.current_vehicle_speed.signum()
        } else {
            0.0
        };

        for wheel in &mut self.wheels {
            if wheel.role.driven {
                let radius = wheel.radius.max(0.01);
                let drive_force = output.drive_torque / (radius * driven_divisor);
                let engine_brake_force =
                    output.engine_brake_torque / (radius * driven_divisor) * motion_sign;
                wheel.engine_force = drive_force - engine_brake_force;
                wheel.target_rotation = output.wheel_target_velocity;
                wheel.wheel_coupling_torque = output.wheel_coupling_torque / driven_divisor;
                wheel.drive_throttle = output.drive_throttle;
                wheel.drivetrain_connected = output.drivetrain_connected;
            } else {
                wheel.engine_force = 0.0;
                wheel.target_rotation = 0.0;
                wheel.wheel_coupling_torque = 0.0;
                wheel.drive_throttle = 0.0;
                wheel.drivetrain_connected = false;
            }

            let service_brake = match wheel.role.axle {
                WheelAxle::Front => output.service_brake * dynamics.brake_bias,
                WheelAxle::Rear => output.service_brake * (1.0 - dynamics.brake_bias),
            };
            wheel.brake = if wheel.role.axle == WheelAxle::Rear {
                service_brake.max(input.handbrake)
            } else {
                service_brake
            };
            wheel.handbrake_overrides_abs =
                input.handbrake > service_brake && wheel.role.axle == WheelAxle::Rear;
        }
    }

    fn apply_chassis_dynamics(&self, dt: Real, bodies: &mut RigidBodySet) {
        let dynamics = &self.powertrain.config.dynamics;
        let chassis = bodies
            .get_mut_internal_with_modification_tracking(self.chassis)
            .unwrap();
        let transform = *chassis.position();
        let forward = transform.rotation * Vector::ith(self.index_forward_axis, 1.0);
        let up = transform.rotation * Vector::ith(self.index_up_axis, 1.0);
        let speed = self.current_vehicle_speed;
        let speed_abs = speed.abs();
        let drag =
            0.5 * 1.225 * dynamics.drag_coefficient * dynamics.frontal_area * speed * speed_abs;
        let rolling = if speed_abs > 0.1 {
            chassis.mass() * 9.81 * dynamics.rolling_resistance * speed.signum()
        } else {
            0.0
        };
        chassis.apply_impulse(-forward * (drag + rolling) * dt, false);
        let downforce_scale = speed_abs * speed_abs * dt;
        if dynamics.downforce_points.is_empty() {
            chassis.apply_impulse(
                -up * dynamics.downforce_coefficient * downforce_scale,
                false,
            );
        } else {
            for point in &dynamics.downforce_points {
                let world_point = transform * Point::from(point.position);
                chassis.apply_impulse_at_point(
                    -up * point.coefficient * downforce_scale,
                    world_point,
                    false,
                );
            }
        }
        chassis.set_linear_damping(
            dynamics.base_linear_damping + dynamics.linear_damping_per_speed * speed_abs,
        );
        chassis.set_angular_damping(
            dynamics.base_angular_damping + dynamics.angular_damping_per_speed * speed_abs,
        );
    }

    fn update_output_state(&mut self, chassis: &RigidBody) {
        let wheels_in_contact = self
            .wheels
            .iter()
            .filter(|wheel| wheel.raycast_info.is_in_contact)
            .count();
        let abs_activity = self
            .wheels
            .iter()
            .filter(|wheel| wheel.is_anti_lock_brake)
            .count() as Real
            / self.wheels.len().max(1) as Real;
        let driven_count = self.wheels.iter().filter(|wheel| wheel.role.driven).count();
        let traction_control_activity = if driven_count == 0 {
            0.0
        } else {
            self.wheels
                .iter()
                .filter(|wheel| wheel.role.driven && wheel.raycast_info.is_in_contact)
                .map(|wheel| {
                    if wheel.wheel_coupling_torque * wheel.target_rotation.signum() > 0.0 {
                        wheel.traction_control_cut
                    } else {
                        0.0
                    }
                })
                .sum::<Real>()
                / driven_count as Real
        };
        let up = chassis.position().rotation * Vector::ith(self.index_up_axis, 1.0);
        let mut planar_velocity = *chassis.linvel();
        planar_velocity -= up * planar_velocity.dot(&up);
        let velocity_direction = planar_velocity.try_normalize(Real::EPSILON);
        let mut steering_count = 0;
        let mut skid_sum = 0.0;
        let mut compression_sum = 0.0;
        let mut ground_friction_sum = 0.0;
        let mut slip_feedback = 0.0;

        for wheel in self
            .wheels
            .iter()
            .filter(|wheel| wheel.role.steered && wheel.raycast_info.is_in_contact)
        {
            steering_count += 1;
            skid_sum += wheel.skid_info;
            compression_sum += wheel.suspension_compression_rate;
            ground_friction_sum += wheel.ground_friction;

            if let Some(velocity_direction) = velocity_direction {
                let normal = wheel.raycast_info.contact_normal_ws;
                let axle = wheel.wheel_axle_ws - normal * wheel.wheel_axle_ws.dot(&normal);
                if let Some(side) = axle.try_normalize(Real::EPSILON) {
                    if let Some(wheel_forward) = normal.cross(&side).try_normalize(Real::EPSILON) {
                        let angle = velocity_direction.angle(&wheel_forward)
                            * up.dot(&velocity_direction.cross(&wheel_forward)).signum();
                        slip_feedback += (-angle * 4.0).clamp(-1.0, 1.0);
                    }
                }
            }
        }

        let (force_feedback, steering_friction) = if steering_count == 0 {
            (0.0, 0.24)
        } else {
            let count = steering_count as Real;
            let average_skid = skid_sum / count;
            let average_ground_friction = ground_friction_sum / count;
            let speed_factor = (self.current_vehicle_speed.abs() * 0.1).clamp(0.0, 1.0);
            let bump = self.last_steering_compression - compression_sum;
            self.last_steering_compression = compression_sum;
            let abs_pulse = (self.timer * 35.0).sin() * abs_activity * 0.2;
            let feedback =
                ((slip_feedback / count) * average_skid * speed_factor + bump * 6.0 + abs_pulse)
                    .clamp(-1.0, 1.0);
            let wheel_speed_factor =
                1.0 - (self.powertrain.state().driven_wheel_speed.abs() / 3.0).min(1.0);
            let compression = (compression_sum * 4.0).min(1.0) * wheel_speed_factor;
            let friction = (0.24
                + (0.6 + average_skid * 0.4) * average_ground_friction * compression * 0.36)
                .min(1.0);
            (feedback, friction)
        };
        let state = self.powertrain.state_mut();
        state.wheels_in_contact = wheels_in_contact;
        state.abs_activity = abs_activity;
        state.traction_control_activity = traction_control_activity.clamp(0.0, 1.0);
        state.force_feedback = force_feedback;
        state.steering_friction = steering_friction;
    }

    /// Integrates gravity together with suspension and tire impulses for this tick.
    /// Step the world next, then call `finish_vehicle_update` to restore automatic
    /// chassis gravity. Other bodies continue using the world's normal integration.
    #[profiling::function]
    pub fn update_vehicle(
        &mut self,
        dt: Real,
        gravity: &Vector<Real>,
        bodies: &mut RigidBodySet,
        colliders: &ColliderSet,
        queries: &QueryPipeline,
        filter: QueryFilter,
    ) {
        self.finish_vehicle_update(bodies);
        if dt <= 0.0 {
            return;
        }
        self.timer += dt;
        let num_wheels = self.wheels.len();
        let chassis = &bodies[self.chassis];

        let forward_w = chassis.position() * Vector::ith(self.index_forward_axis, 1.0);
        self.current_vehicle_speed = forward_w.dot(chassis.linvel());
        let output = self.update_powertrain(dt);
        self.update_steering(chassis, dt);
        self.apply_powertrain_output(output);
        self.apply_chassis_dynamics(dt, bodies);
        let chassis = &bodies[self.chassis];

        for i in 0..num_wheels {
            self.update_wheel_transform(chassis, i);
        }

        //
        // simulate suspension
        //

        for wheel_id in 0..self.wheels.len() {
            self.ray_cast(bodies, colliders, queries, filter, chassis, wheel_id);
        }

        let chassis_mass = chassis.mass();
        self.update_suspension(chassis_mass);
        self.apply_anti_roll_bars(chassis_mass);

        let chassis = bodies
            .get_mut_internal_with_modification_tracking(self.chassis)
            .unwrap();

        let gravity_scale = chassis.gravity_scale();
        self.pending_gravity_scale = Some(gravity_scale);
        self.pending_gravity_impulse = gravity * (chassis.mass() * gravity_scale * dt);
        chassis.set_gravity_scale(0.0, false);
        chassis.apply_impulse(self.pending_gravity_impulse, false);

        for wheel in &mut self.wheels {
            if wheel.engine_force.abs() > 0.0 {
                chassis.wake_up(true);
            }

            // apply suspension force
            let suspension_force = wheel
                .wheel_suspension_force
                .min(wheel.max_suspension_force)
                .max(0.0);

            let impulse = wheel.raycast_info.contact_normal_ws * suspension_force * dt;
            chassis.apply_impulse_at_point(impulse, wheel.raycast_info.contact_point_ws, false);
        }

        for wheel in &self.wheels {
            if let Some(ground) = wheel
                .raycast_info
                .ground_object
                .and_then(|handle| colliders[handle].parent())
                .filter(|handle| *handle != self.chassis && bodies[*handle].is_dynamic())
            {
                let force = wheel
                    .wheel_suspension_force
                    .min(wheel.max_suspension_force)
                    .max(0.0);
                bodies
                    .get_mut_internal_with_modification_tracking(ground)
                    .unwrap()
                    .apply_impulse_at_point(
                        -wheel.raycast_info.contact_normal_ws * force * dt,
                        wheel.raycast_info.contact_point_ws,
                        true,
                    );
            }
        }

        self.update_friction(bodies, colliders, dt);

        for wheel in &mut self.wheels {
            update_wheel_rotation(wheel, dt);
        }
        let chassis = &bodies[self.chassis];
        self.update_output_state(chassis);
    }

    /// Completes the vehicle tick after stepping the world.
    pub fn finish_vehicle_update(&mut self, bodies: &mut RigidBodySet) {
        let Some(scale) = self.pending_gravity_scale.take() else {
            return;
        };
        if let Some(chassis) = bodies.get_mut(self.chassis) {
            chassis.set_gravity_scale(scale, false);
            self.current_vehicle_speed = (chassis.position()
                * Vector::ith(self.index_forward_axis, 1.0))
            .dot(chassis.linvel());
            self.update_output_state(chassis);
        }
    }

    /// Cancels gravity ownership before the world step when resetting or removing
    /// the controller. Retract the pre-applied gravity impulse so the world can
    /// integrate it normally, without adding it a second time.
    pub fn cancel_vehicle_update(&mut self, bodies: &mut RigidBodySet) {
        if let Some(scale) = self.pending_gravity_scale.take() {
            if let Some(chassis) = bodies.get_mut(self.chassis) {
                chassis.apply_impulse(-self.pending_gravity_impulse, false);
                chassis.set_gravity_scale(scale, false);
            }
        }
    }

    /// Reference to all the wheels attached to this vehicle.
    pub fn wheels(&self) -> &[Wheel] {
        &self.wheels
    }

    /// Mutable reference to all the wheels attached to this vehicle.
    pub fn wheels_mut(&mut self) -> &mut [Wheel] {
        &mut self.wheels
    }

    fn update_suspension(&mut self, chassis_mass: Real) {
        for w_it in 0..self.wheels.len() {
            let wheels = &mut self.wheels[w_it];
            wheels.suspension_compression_rate = 0.0;

            if wheels.raycast_info.is_in_contact {
                let mut force;
                //	Spring
                {
                    let rest_length = wheels.suspension_rest_length;
                    let current_length = wheels.raycast_info.suspension_length;
                    let length_diff = rest_length - current_length;
                    wheels.suspension_compression_rate = 1.0 - (current_length / rest_length);

                    force = wheels.suspension_stiffness
                        * length_diff
                        * wheels.clipped_inv_contact_dot_suspension;
                }

                // Damper
                {
                    let projected_rel_vel = wheels.suspension_relative_velocity;
                    {
                        let susp_damping = if projected_rel_vel < 0.0 {
                            wheels.damping_compression
                        } else {
                            wheels.damping_relaxation
                        };
                        force -= susp_damping * projected_rel_vel;
                    }
                }

                // RESULT
                wheels.wheel_suspension_force = (force * chassis_mass).max(0.0);
            } else {
                wheels.wheel_suspension_force = 0.0;
            }
        }
    }

    fn apply_anti_roll_bars(&mut self, chassis_mass: Real) {
        let side_axis = self.side_axis();

        for axle in [WheelAxle::Front, WheelAxle::Rear] {
            let stiffness = match axle {
                WheelAxle::Front => {
                    self.powertrain
                        .config
                        .dynamics
                        .front_anti_roll_bar_stiffness
                }
                WheelAxle::Rear => self.powertrain.config.dynamics.rear_anti_roll_bar_stiffness,
            };
            if stiffness <= 0.0 || chassis_mass <= 0.0 {
                continue;
            }

            let mut first = None;
            let mut second = None;
            let mut has_extra_wheel = false;
            for (wheel_id, wheel) in self
                .wheels
                .iter()
                .enumerate()
                .filter(|(_, wheel)| wheel.role.axle == axle)
            {
                let entry = (
                    wheel_id,
                    wheel.chassis_connection_point_cs.coords[side_axis],
                );
                if first.is_none() {
                    first = Some(entry);
                } else if second.is_none() {
                    second = Some(entry);
                } else {
                    has_extra_wheel = true;
                    break;
                }
            }
            let (Some(first), Some(second)) = (first, second) else {
                continue;
            };
            if has_extra_wheel {
                continue;
            }

            let ((left_id, left_side), (right_id, right_side)) = if first.1 <= second.1 {
                (first, second)
            } else {
                (second, first)
            };
            if right_side - left_side <= Real::EPSILON {
                continue;
            }

            let left = &self.wheels[left_id];
            let right = &self.wheels[right_id];
            if !left.raycast_info.is_in_contact || !right.raycast_info.is_in_contact {
                continue;
            }

            let left_compression =
                left.suspension_rest_length - left.raycast_info.suspension_length;
            let right_compression =
                right.suspension_rest_length - right.raycast_info.suspension_length;
            let transfer = anti_roll_bar_transfer(
                left_compression,
                right_compression,
                stiffness,
                chassis_mass,
                left.wheel_suspension_force,
                right.wheel_suspension_force,
                left.max_suspension_force,
                right.max_suspension_force,
            );

            self.wheels[left_id].wheel_suspension_force += transfer;
            self.wheels[right_id].wheel_suspension_force -= transfer;
        }
    }

    // Experimental rear-axle drive allocation: balance positive surface-speed
    // excess over each wheel's own road speed, not the wheel angular velocities.
    fn redistribute_rear_drive_torque(
        &self,
        contacts: &[WheelContactState],
        drive_torques: &mut [Real],
        dt: Real,
    ) {
        if dt <= Real::EPSILON {
            return;
        }
        let mut rear = self
            .wheels
            .iter()
            .enumerate()
            .filter(|(_, wheel)| wheel.role.driven && wheel.role.axle == WheelAxle::Rear);
        let (Some((a, first)), Some((b, second))) = (rear.next(), rear.next()) else {
            return;
        };
        if rear.next().is_some()
            || !contacts[a].is_grounded
            || !contacts[b].is_grounded
            || !first.drivetrain_connected
            || !second.drivetrain_connected
            || first.drive_throttle <= 0.0
            || second.drive_throttle <= 0.0
            || first.brake > 0.0
            || second.brake > 0.0
        {
            return;
        }
        let direction = first.target_rotation.signum();
        let first_torque = drive_torques[a] * direction;
        let second_torque = drive_torques[b] * direction;
        if direction == 0.0
            || second.target_rotation.signum() != direction
            || first_torque <= 0.0
            || second_torque <= 0.0
        {
            return;
        }
        let first_radius = first.radius.max(0.01);
        let second_radius = second.radius.max(0.01);
        let first_excess = ((first.angular_velocity * first_radius - contacts[a].forward_speed)
            * direction)
            .max(0.0);
        let second_excess = ((second.angular_velocity * second_radius - contacts[b].forward_speed)
            * direction)
            .max(0.0);
        let response = dt
            * (first_radius / wheel_angular_inertia(first_radius)
                + second_radius / wheel_angular_inertia(second_radius));
        // Equal and opposite torque changes correct the excess-spin difference.
        // The only bound is available drive torque: never create donor braking
        // or additional axle torque. Tire reaction and assists still solve below.
        let transfer =
            ((first_excess - second_excess) / response).clamp(-second_torque, first_torque);
        drive_torques[a] -= transfer * direction;
        drive_torques[b] += transfer * direction;
    }

    #[profiling::function]
    fn update_friction(&mut self, bodies: &mut RigidBodySet, colliders: &ColliderSet, dt: Real) {
        let num_wheels = self.wheels.len();
        let force_scale = self.powertrain.config.engine.force_scale;
        self.powertrain.state_mut().esc_activity = 0.0;
        if num_wheels == 0 {
            return;
        }

        self.forward_ws.resize(num_wheels, Default::default());
        self.axle.resize(num_wheels, Default::default());
        let mut contacts = vec![WheelContactState::default(); num_wheels];

        let (esc_intervention, esc_side_axis, chassis_forward) = {
            let chassis = &bodies[self.chassis];
            let intervention = self.esc_intervention(chassis);
            let rotation = chassis.position().rotation;
            let chassis_forward = rotation * Vector::ith(self.index_forward_axis, 1.0);
            (intervention, self.side_axis(), chassis_forward)
        };
        self.powertrain.state_mut().esc_activity = esc_intervention.activity;

        for wheel in &mut self.wheels {
            wheel.brake_impulse = 0.0;
            wheel.drive_torque_transfer = 0.0;
            wheel.side_impulse = 0.0;
            wheel.forward_impulse = 0.0;
            wheel.is_anti_lock_brake = false;
            wheel.ground_friction = 1.0;
            wheel.ground_type = String::new();
            wheel.lock = false;
            wheel.skid_info = 0.0;
            wheel.engine_force_feedback = 0.0;
            wheel.contact_forward_speed = 0.0;
            wheel.contact_side_speed = 0.0;
        }

        for wheel_id in 0..num_wheels {
            let wheel = &mut self.wheels[wheel_id];
            let Some(ground_object) = wheel.raycast_info.ground_object else {
                wheel.last_skid_info = wheel.skid_info;
                wheel.traction_control_cut = 0.0;
                continue;
            };

            let contact_normal = wheel.raycast_info.contact_normal_ws;
            let axle =
                wheel.wheel_axle_ws - contact_normal * wheel.wheel_axle_ws.dot(&contact_normal);
            let side_dir = axle.try_normalize(1.0e-5).unwrap_or_else(Vector::zeros);
            let forward_dir = aligned_wheel_forward(&contact_normal, &side_dir, &chassis_forward);
            let contact_velocity = relative_velocity_at_contact(
                bodies,
                colliders,
                self.chassis,
                Some(ground_object),
                &wheel.raycast_info.contact_point_ws,
            );
            let positive_side = steering_positive_side(&contact_normal, &forward_dir);

            self.axle[wheel_id] = side_dir;
            self.forward_ws[wheel_id] = forward_dir;
            wheel.contact_forward_speed = forward_dir.dot(&contact_velocity);
            wheel.contact_side_speed = positive_side.dot(&contact_velocity);

            wheel.ground_type = colliders[ground_object].material.name.clone();
            let tire = self.tire_types.get(&wheel.tire_type);
            let friction = tire
                .map(|t| t.get_friction(&wheel.ground_type))
                .unwrap_or_else(|| TireFriction::new(wheel.friction_slip, wheel.friction_slip));
            wheel.ground_friction = friction.peak;
            let kinetic_grip = friction.multiplier(
                wheel.angular_velocity * wheel.radius - wheel.contact_forward_speed,
                wheel.contact_side_speed,
            );
            let suspension_force = wheel
                .wheel_suspension_force
                .min(wheel.max_suspension_force)
                .max(0.0);
            let peak_friction_limit =
                suspension_force * dt * wheel.ground_friction * wheel.friction_slip;
            contacts[wheel_id] = WheelContactState {
                is_grounded: true,
                ground_object: Some(ground_object),
                forward_dir,
                forward_speed: wheel.contact_forward_speed,
                friction,
                kinetic_grip,
                peak_friction_limit,
            };
        }

        let mut drive_torques: Vec<_> = self
            .wheels
            .iter()
            .map(|wheel| {
                wheel.wheel_coupling_torque * force_scale * (1.0 - esc_intervention.engine_cut)
            })
            .collect();
        if esc_intervention.brake_strength == 0.0 {
            self.redistribute_rear_drive_torque(&contacts, &mut drive_torques, dt);
        }

        for wheel_id in 0..num_wheels {
            let contact = &mut contacts[wheel_id];
            let wheel = &mut self.wheels[wheel_id];
            if contact.is_grounded {
                // Earlier wheels have already applied their longitudinal impulse.
                // Do not let each locked wheel independently stop the whole chassis.
                contact.forward_speed = contact.forward_dir.dot(&relative_velocity_at_contact(
                    bodies,
                    colliders,
                    self.chassis,
                    contact.ground_object,
                    &wheel.raycast_info.contact_point_ws,
                ));
            }
            let radius = wheel.radius.max(0.01);
            let inertia = wheel_angular_inertia(radius);
            let drive_direction = wheel.target_rotation.signum();
            let raw_wheel_torque = drive_torques[wheel_id];
            let powered_acceleration = wheel.role.driven
                && wheel.drivetrain_connected
                && raw_wheel_torque * drive_direction > Real::EPSILON;
            let tc_requested = wheel.role.driven
                && wheel.drivetrain_connected
                && wheel.drive_throttle > 0.0
                && wheel.traction_control > 0.0;
            let esc_brake = if esc_intervention.brake_strength > 0.0 {
                let side = wheel.chassis_connection_point_cs.coords[esc_side_axis];
                let wheel_side = if side > 0.0 {
                    1.0
                } else if side < 0.0 {
                    -1.0
                } else {
                    0.0
                };

                if esc_intervention.brake_axle == Some(wheel.role.axle)
                    && wheel_side != 0.0
                    && wheel_side == esc_intervention.brake_side
                {
                    esc_intervention.brake_strength
                } else {
                    0.0
                }
            } else {
                0.0
            };

            let brake = (wheel.brake + esc_brake).clamp(0.0, 1.0);
            let requested_brake_impulse = wheel.max_brake_force * brake * dt;

            if !contact.is_grounded {
                wheel.sliding_grip = 1.0;
                wheel.traction_control_cut = 0.0;
                wheel.abs_release = 0.0;
                let drive_angular_impulse = raw_wheel_torque * dt;
                let mut angular_velocity = wheel.angular_velocity + drive_angular_impulse / inertia;
                let reference_angular_velocity = angular_velocity;
                apply_opposing_angular_impulse(
                    &mut angular_velocity,
                    reference_angular_velocity,
                    inertia,
                    requested_brake_impulse * radius,
                );
                wheel.angular_velocity = angular_velocity;
                wheel.angular_load = if dt > Real::EPSILON {
                    (reference_angular_velocity - angular_velocity) * inertia / dt
                } else {
                    0.0
                };
                wheel.lock =
                    brake > Real::EPSILON && wheel.angular_velocity.abs() <= WHEEL_STOP_EPSILON;
                wheel.last_skid_info = wheel.skid_info;
                continue;
            }

            let raw_drive_angular_impulse = raw_wheel_torque * dt;
            let point = wheel.raycast_info.contact_point_ws;
            let chassis = &bodies[self.chassis];
            let axes = [contact.forward_dir, self.axle[wheel_id]];
            // Preserve anti_roll as chassis-roll influence, rather than shifting
            // one steered axis's force point. Use the same generalized velocity
            // and impulse Jacobian in prediction and application.
            let angular_axes = axes.map(|axis| {
                let angular = (point - chassis.center_of_mass()).cross(&axis);
                angular - chassis_forward * (angular.dot(&chassis_forward) * wheel.anti_roll)
            });
            let ground_body = contact
                .ground_object
                .and_then(|handle| colliders[handle].parent())
                .filter(|handle| *handle != self.chassis);
            let ground_velocity = ground_body
                .map(|handle| bodies[handle].velocity_at_point(&point))
                .unwrap_or_else(Vector::zeros);
            let speed = [
                axes[0].dot(&(chassis.linvel() - ground_velocity))
                    + angular_axes[0].dot(chassis.angvel()),
                axes[1].dot(&(chassis.linvel() - ground_velocity))
                    + angular_axes[1].dot(chassis.angvel()),
            ];
            let mut response = tangent_impulse_response(chassis, axes, angular_axes);
            if let Some(handle) = ground_body {
                let ground = &bodies[handle];
                let ground_response = tangent_impulse_response(
                    ground,
                    axes,
                    axes.map(|axis| (point - ground.center_of_mass()).cross(&axis)),
                );
                for i in 0..2 {
                    for j in 0..2 {
                        response[i][j] += ground_response[i][j];
                    }
                }
            }
            // Use the exact same coupled result for assist previews and application.
            let contact_for = |drive_fraction: Real, brake_impulse: Real| CoupledContact {
                speed,
                response,
                omega: wheel.angular_velocity
                    + raw_drive_angular_impulse * drive_fraction / inertia,
                radius,
                inertia,
                grip_impulse: contact.peak_friction_limit,
                brake_budget: brake_impulse * radius,
            };
            let solve = |drive_fraction: Real, brake_impulse: Real| {
                solve_with_grip_recovery(
                    contact_for(drive_fraction, brake_impulse),
                    contact.kinetic_grip,
                    wheel.sliding_grip,
                    contact.friction.sliding_ratio(),
                    dt,
                )
            };
            let predict = |drive_fraction: Real, brake_impulse: Real| {
                let (result, _) = solve(drive_fraction, brake_impulse);
                (result.omega * radius, result.speed[0])
            };
            let drive_cut_for_brake = |brake_impulse: Real| {
                if powered_acceleration && tc_requested {
                    1.0 - traction_control_torque_fraction(wheel.traction_control, |fraction| {
                        let (wheel_speed, road_speed) = predict(fraction, brake_impulse);
                        (wheel_speed - road_speed) * drive_direction
                    })
                } else {
                    0.0
                }
            };
            let brake_fraction = if requested_brake_impulse > 0.0
                && !wheel.handbrake_overrides_abs
                && self.current_vehicle_speed.abs() > 1.0
                && contact.forward_speed.abs() > 1.0
            {
                anti_lock_brake_torque_fraction(
                    wheel.anti_lock_brake,
                    contact.forward_speed.signum(),
                    |fraction| {
                        let brake_impulse = requested_brake_impulse * fraction;
                        let cut = drive_cut_for_brake(brake_impulse);
                        predict(1.0 - cut, brake_impulse)
                    },
                )
            } else {
                1.0
            };
            let max_brake_impulse = requested_brake_impulse * brake_fraction;
            let cut = drive_cut_for_brake(max_brake_impulse);
            let (result, sliding_grip) = solve(1.0 - cut, max_brake_impulse);
            let (unbraked, _) = solve(1.0 - cut, 0.0);
            let mut final_contact = contact_for(1.0 - cut, max_brake_impulse);
            final_contact.grip_impulse *= sliding_grip;
            let drive_angular_velocity = final_contact.omega;
            wheel.skid_info = final_contact.skid_info(result);
            wheel.sliding_grip = sliding_grip;
            let base_drive_torque =
                wheel.wheel_coupling_torque * force_scale * (1.0 - esc_intervention.engine_cut);
            // Record only the transfer that reaches the wheel after TC. The
            // road/brake angular_load below remains the physical contact load.
            wheel.drive_torque_transfer = (raw_wheel_torque - base_drive_torque) * (1.0 - cut);
            wheel.is_anti_lock_brake = brake_fraction < 1.0;
            wheel.abs_release = 1.0 - brake_fraction;
            // Limiter/coast torque must not apply TC to negative torque, but a
            // brief interruption under held throttle must retain controller memory.
            if powered_acceleration || !tc_requested {
                wheel.traction_control_cut = cut;
            }

            wheel.last_skid_info = wheel.skid_info;
            wheel.forward_impulse = result.tangent[0];
            wheel.side_impulse = result.tangent[1];
            wheel.brake_impulse = result.tangent[0] - unbraked.tangent[0];
            wheel.engine_force_feedback = result.tangent[0];
            let mut final_angular_velocity = result.omega;
            if final_angular_velocity.abs() <= WHEEL_STOP_EPSILON {
                final_angular_velocity = 0.0;
            }
            wheel.angular_velocity = final_angular_velocity;
            wheel.angular_load = if dt > Real::EPSILON {
                (drive_angular_velocity - final_angular_velocity) * inertia / dt
            } else {
                0.0
            };
            wheel.lock = brake > Real::EPSILON && final_angular_velocity == 0.0;
            let chassis = bodies
                .get_mut_internal_with_modification_tracking(self.chassis)
                .unwrap();
            chassis.apply_impulse(
                axes[0] * result.tangent[0] + axes[1] * result.tangent[1],
                false,
            );
            chassis.apply_torque_impulse(
                angular_axes[0] * result.tangent[0] + angular_axes[1] * result.tangent[1],
                false,
            );
            if let Some(handle) = ground_body.filter(|handle| bodies[*handle].is_dynamic()) {
                bodies
                    .get_mut_internal_with_modification_tracking(handle)
                    .unwrap()
                    .apply_impulse_at_point(
                        -axes[0] * result.tangent[0] - axes[1] * result.tangent[1],
                        point,
                        false,
                    );
            }
        }
    }
}

// One maximum-dissipation solve for tangent contact and
// bounded brake torque. Impulses, not forces, are used throughout.
//
// Minimize the change in chassis + wheel kinetic energy over a friction disk
// and a signed brake-impulse interval. The brake's active constraints (released,
// holding, or at either torque bound) are part of that same convex problem.
// No wheel-lock flag, steering correction, slip blend, or previous actuator
// state selects a different friction rule.
//
// The caller supplies matching generalized velocities and impulse responses,
// including chassis-roll influence and dynamic ground reactions.
// The selected contact branch also supplies the demand for skid_info reporting.

// Lateral demand weighting for skid reporting. This preserves its feedback and
// effects scale without changing the contact impulses or exposing tire tuning.
const SKID_LATERAL_DEMAND_WEIGHT: Real = 0.2;

#[derive(Clone, Copy, Debug)]
struct CoupledContact {
    /// Tangent generalized velocity of the chassis relative to the ground.
    pub speed: [Real; 2],
    /// Symmetric contact inverse-mass matrix, including both bodies and cross
    /// terms. Application must use the same impulse Jacobians used to build it.
    pub response: [[Real; 2]; 2],
    /// Wheel velocity entering this solve, after any caller-applied drive impulse.
    pub omega: Real,
    pub radius: Real,
    pub inertia: Real,
    /// Impulse budget including suspension load, timestep and current grip envelope.
    pub grip_impulse: Real,
    pub brake_budget: Real,
}

#[derive(Clone, Copy, Debug)]
struct CoupledContactSolution {
    pub tangent: [Real; 2],
    /// Uncapped demand with the same brake constraint as the applied solution.
    pub requested_tangent: [Real; 2],
    #[cfg(test)]
    brake: Real,
    pub omega: Real,
    pub speed: [Real; 2],
}

fn dot(a: [Real; 2], b: [Real; 2]) -> Real {
    a[0] * b[0] + a[1] * b[1]
}

fn multiply(a: [[Real; 2]; 2], x: [Real; 2]) -> [Real; 2] {
    [dot(a[0], x), dot(a[1], x)]
}

fn shifted_solve(a: [[Real; 2]; 2], rhs: [Real; 2], shift: Real) -> [Real; 2] {
    let xx = a[0][0] + shift;
    let yy = a[1][1] + shift;
    let determinant = xx * yy - a[0][1] * a[1][0];
    if determinant <= 0.0 {
        return [Real::INFINITY; 2];
    }
    [
        (yy * rhs[0] - a[0][1] * rhs[1]) / determinant,
        (xx * rhs[1] - a[1][0] * rhs[0]) / determinant,
    ]
}

/// Solve the two-dimensional convex quadratic under a circular impulse limit.
/// On its boundary, (A + lambda I) j = rhs makes the resulting slip oppose j.
fn friction_disk(a: [[Real; 2]; 2], rhs: [Real; 2], limit: Real) -> [Real; 2] {
    if limit <= 0.0 || rhs == [0.0; 2] {
        return [0.0; 2];
    }
    let free = shifted_solve(a, rhs, 0.0);
    if free[0].hypot(free[1]) <= limit {
        return free;
    }
    let mut lower = 0.0;
    // For positive definite A, this shift guarantees a point inside the disk.
    let mut upper = rhs[0].hypot(rhs[1]) / limit;
    for _ in 0..32 {
        let shift = (lower + upper) * 0.5;
        if shift == lower || shift == upper {
            break;
        }
        let impulse = shifted_solve(a, rhs, shift);
        if impulse[0].hypot(impulse[1]) > limit {
            lower = shift;
        } else {
            upper = shift;
        }
    }
    shifted_solve(a, rhs, upper)
}

impl CoupledContact {
    fn skid_info(self, solution: CoupledContactSolution) -> Real {
        if self.grip_impulse <= 0.0 {
            return 0.0;
        }
        let required = solution.requested_tangent;
        let demand = required[0].hypot(required[1] * SKID_LATERAL_DEMAND_WEIGHT);
        if demand == 0.0 {
            1.0
        } else {
            (self.grip_impulse / demand).clamp(0.0, 1.0)
        }
    }

    fn finish(
        self,
        tangent: [Real; 2],
        brake: Real,
        response: [[Real; 2]; 2],
        rhs: [Real; 2],
    ) -> CoupledContactSolution {
        let delta = multiply(self.response, tangent);
        CoupledContactSolution {
            tangent,
            requested_tangent: if rhs == [0.0; 2] {
                [0.0; 2]
            } else {
                shifted_solve(response, rhs, 0.0)
            },
            #[cfg(test)]
            brake,
            omega: self.omega + (brake - tangent[0] * self.radius) / self.inertia,
            speed: [self.speed[0] + delta[0], self.speed[1] + delta[1]],
        }
    }

    pub fn solve(self) -> CoupledContactSolution {
        // With the wheel held, its brake impulse is an unknown inside the box;
        // solve chassis contact first, then check the actual holding requirement.
        let held = friction_disk(
            self.response,
            [-self.speed[0], -self.speed[1]],
            self.grip_impulse,
        );
        let holding_brake = held[0] * self.radius - self.inertia * self.omega;
        if holding_brake.abs() <= self.brake_budget {
            let mut result = self.finish(
                held,
                holding_brake,
                self.response,
                [-self.speed[0], -self.speed[1]],
            );
            // Exact algebraic equality, not synchronization to ground speed.
            result.omega = 0.0;
            return result;
        }

        // The unconstrained holding solution identifies the active brake bound.
        // Wheel inertia remains part of the longitudinal contact response.
        let brake = holding_brake.signum() * self.brake_budget;
        let driven_omega = self.omega + brake / self.inertia;
        let mut response = self.response;
        response[0][0] += self.radius * self.radius / self.inertia;
        let tangent = friction_disk(
            response,
            [driven_omega * self.radius - self.speed[0], -self.speed[1]],
            self.grip_impulse,
        );
        self.finish(
            tangent,
            brake,
            response,
            [driven_omega * self.radius - self.speed[0], -self.speed[1]],
        )
    }
}

/// Peak and fully sliding friction coefficients for one tire/surface pair.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TireFriction {
    /// Friction coefficient used by ordinary rolling contact.
    pub peak: Real,
    /// Friction coefficient at full sliding.
    pub sliding: Real,
}

impl TireFriction {
    /// Creates finite coefficients satisfying `0 <= sliding <= peak`.
    /// Panics if either coefficient is invalid.
    pub fn new(peak: Real, sliding: Real) -> Self {
        assert!(
            peak.is_finite()
                && sliding.is_finite()
                && peak >= 0.0
                && sliding >= 0.0
                && sliding <= peak
        );
        Self { peak, sliding }
    }

    fn sliding_ratio(self) -> Real {
        if self.peak > 0.0 {
            self.sliding / self.peak
        } else {
            1.0
        }
    }

    fn multiplier(self, forward_slip: Real, side_slip: Real) -> Real {
        let speed = forward_slip.hypot(side_slip);
        let t = ((speed - SLIDING_START_SPEED) / (SLIDING_FULL_SPEED - SLIDING_START_SPEED))
            .clamp(0.0, 1.0);
        1.0 - (1.0 - self.sliding_ratio()) * t * t * (3.0 - 2.0 * t)
    }
}

/// Preview from immutable previous-step state. Only the final wheel solve commits
/// the returned multiplier, so ABS/TC searches cannot advance recovery repeatedly.
/// The incoming contact budget is PEAK grip, avoiding division by zero when a
/// surface explicitly configures zero sliding grip.
fn solve_with_grip_recovery(
    mut contact: CoupledContact,
    kinetic_grip: Real,
    previous_grip: Real,
    sliding_floor: Real,
    dt: Real,
) -> (CoupledContactSolution, Real) {
    // Do not carry a deeper loss from a previous surface onto this surface.
    let previous_grip = previous_grip.max(sliding_floor);
    let mut grip = previous_grip.min(kinetic_grip);
    if kinetic_grip > previous_grip && dt > 0.0 {
        let demand = CoupledContact {
            grip_impulse: Real::MAX,
            ..contact
        }
        .solve()
        .tangent;
        let required = demand[0].hypot(demand[1]);
        let unheld_capacity = contact.grip_impulse * kinetic_grip;
        let ratio = if unheld_capacity <= 0.0 {
            0.0
        } else if required == 0.0 {
            1.0
        } else {
            (unheld_capacity / required).clamp(0.0, 1.0)
        };
        // Original baseline recovery plus a smooth boost near supportable demand.
        let t = ((ratio - RECOVERY_THRESHOLD) / (1.0 - RECOVERY_THRESHOLD)).clamp(0.0, 1.0);
        let boost = t * t * (3.0 - 2.0 * t);
        let rate = RECOVERY_BASE_RATE + (RECOVERY_RATE - RECOVERY_BASE_RATE) * boost;
        grip = (grip + rate * dt).min(kinetic_grip);
    }
    contact.grip_impulse *= grip;
    (contact.solve(), grip)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dynamics::RigidBodyBuilder;

    fn example_contact() -> CoupledContact {
        CoupledContact {
            speed: [40.0, 4.0],
            response: [[0.004, 0.001], [0.001, 0.006]],
            omega: 40.0 / 0.35,
            radius: 0.35,
            inertia: 1.5,
            grip_impulse: 50.0,
            brake_budget: 100.0,
        }
    }

    fn check_solution(contact: CoupledContact, solution: CoupledContactSolution) {
        let impulse = solution.tangent[0].hypot(solution.tangent[1]);
        assert!(impulse <= contact.grip_impulse + 0.0001);
        assert!(solution.brake.abs() <= contact.brake_budget + 0.0001);
        assert!(solution.omega.is_finite() && solution.speed.iter().all(|v| v.is_finite()));
        let slip = [
            solution.speed[0] - solution.omega * contact.radius,
            solution.speed[1],
        ];
        let scale = impulse.max(1.0) * slip[0].hypot(slip[1]).max(1.0);
        // Optimality of friction and brake constraints; includes opposite rotation,
        // reverse motion, existing lock, cornering and the transitions between them.
        assert!(dot(solution.tangent, slip) <= 0.0001 * scale);
        assert!(
            (solution.tangent[0] * slip[1] - solution.tangent[1] * slip[0]).abs() <= 0.0001 * scale,
            "friction not opposite resulting slip: {contact:?}, {solution:?}"
        );
        if impulse < contact.grip_impulse - 0.001 {
            assert!(slip[0].hypot(slip[1]) < 0.001);
        }
        assert!(solution.brake * solution.omega <= 0.001);
        if solution.brake.abs() < contact.brake_budget - 0.001 {
            assert!(solution.omega.abs() < 0.001);
        }
        let change = dot(contact.speed, solution.tangent)
            + 0.5
                * dot(
                    solution.tangent,
                    multiply(contact.response, solution.tangent),
                )
            + 0.5
                * contact.inertia
                * (solution.omega * solution.omega - contact.omega * contact.omega);
        assert!(
            change <= 0.01,
            "contact added energy: {change}, {contact:?}, {solution:?}"
        );
    }

    #[test]
    fn combined_contact_obeys_friction_brake_and_energy_constraints() {
        for speed in [-40.0, -2.0, 0.0, 2.0, 40.0] {
            for side_speed in [-20.0, 0.0, 20.0] {
                for omega in [-200.0, 0.0, 200.0] {
                    for grip in [0.0, 0.1, 50.0, 100_000.0] {
                        for brake in [0.0, 1.0, 100.0, 10_000.0] {
                            let contact = CoupledContact {
                                speed: [speed, side_speed],
                                omega,
                                grip_impulse: grip,
                                brake_budget: brake,
                                ..example_contact()
                            };
                            check_solution(contact, contact.solve());
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn unsaturated_straight_contact_matches_existing_inertia_and_brake_solve() {
        for speed in [-40.0, 0.0, 40.0] {
            for omega in [-100.0, 0.0, 100.0] {
                for brake in [0.0, 10.0, 10_000.0] {
                    let contact = CoupledContact {
                        speed: [speed, 0.0],
                        omega,
                        response: [[0.004, 0.0], [0.0, 0.004]],
                        grip_impulse: 100_000.0,
                        brake_budget: brake,
                        ..example_contact()
                    };
                    let result = contact.solve();
                    let (forward, braking) = braked_contact_impulses(
                        omega,
                        speed,
                        contact.radius,
                        contact.inertia,
                        0.004,
                        brake,
                    );
                    let mut expected_omega =
                        omega - (forward + braking) * contact.radius / contact.inertia;
                    apply_opposing_angular_impulse(
                        &mut expected_omega,
                        speed / contact.radius,
                        contact.inertia,
                        brake,
                    );
                    assert!((result.tangent[0] - forward - braking).abs() < 0.01);
                    assert!((result.omega - expected_omega).abs() < 0.001);
                    check_solution(contact, result);
                }
            }
        }
    }

    #[test]
    fn locked_contact_is_invariant_to_steering_coordinate_rotation() {
        let base = CoupledContact {
            omega: 0.0,
            brake_budget: 10_000.0,
            ..example_contact()
        };
        let expected = base.solve();
        for angle in [-1.2 as Real, -0.4, 0.0, 0.4, 1.2] {
            let rotation = [[angle.cos(), angle.sin()], [-angle.sin(), angle.cos()]];
            let mut response = [[0.0; 2]; 2];
            for i in 0..2 {
                for j in 0..2 {
                    response[i][j] = dot(rotation[i], multiply(base.response, rotation[j]));
                }
            }
            let contact = CoupledContact {
                speed: multiply(rotation, base.speed),
                response,
                ..base
            };
            let result = contact.solve();
            let rotated_expected = multiply(rotation, expected.tangent);
            assert_eq!(result.omega, 0.0);
            assert!((result.tangent[0] - rotated_expected[0]).abs() < 0.001);
            assert!((result.tangent[1] - rotated_expected[1]).abs() < 0.001);
            check_solution(contact, result);
        }
    }

    #[test]
    fn brake_sweep_has_no_force_jump_and_monotone_underspeed() {
        for grip in [1.0, 50.0, 5_000.0] {
            let mut previous: Option<CoupledContactSolution> = None;
            for step in 0..2_000 {
                let contact = CoupledContact {
                    grip_impulse: grip,
                    brake_budget: step as Real * 0.5,
                    ..example_contact()
                };
                let result = contact.solve();
                if let Some(old) = previous {
                    let underspeed = result.speed[0] - result.omega * contact.radius;
                    let old_underspeed = old.speed[0] - old.omega * contact.radius;
                    assert!(underspeed >= old_underspeed - 0.001);
                    assert!((result.tangent[0] - old.tangent[0]).abs() < 2.0);
                    assert!((result.tangent[1] - old.tangent[1]).abs() < 2.0);
                }
                check_solution(contact, result);
                previous = Some(result);
            }
        }
    }

    #[test]
    fn response_cross_terms_predict_actual_chassis_impulses() {
        use crate::dynamics::{RigidBodyBuilder, RigidBodySet};
        use crate::geometry::{ColliderBuilder, ColliderSet};
        use crate::math::{Point, Vector};

        for angle in [-0.6 as Real, 0.0, 0.6] {
            let mut bodies = RigidBodySet::new();
            let chassis = bodies.insert(
                RigidBodyBuilder::dynamic()
                    .linvel(Vector::new(4.0, 0.0, 40.0))
                    .angvel(Vector::new(0.1, 0.3, -0.2)),
            );
            let mut colliders = ColliderSet::new();
            colliders.insert_with_parent(
                ColliderBuilder::cuboid(0.8, 0.3, 1.5).mass(1_200.0),
                chassis,
                &mut bodies,
            );
            bodies[chassis].recompute_mass_properties_from_colliders(&colliders);
            let point = Point::new(0.8, -0.3, 1.3);
            let axes = [
                Vector::new(angle.sin(), 0.0, angle.cos()),
                Vector::new(angle.cos(), 0.0, -angle.sin()),
            ];
            let body = &bodies[chassis];
            let velocity = body.velocity_at_point(&point);
            let offset = point - body.center_of_mass();
            let angular =
                axes.map(|axis| body.mprops.effective_world_inv_inertia_sqrt * offset.cross(&axis));
            let mut response = [[0.0; 2]; 2];
            for i in 0..2 {
                for j in 0..2 {
                    response[i][j] = axes[i].dot(&axes[j]) * body.mprops.local_mprops.inv_mass
                        + angular[i].dot(&angular[j]);
                }
            }
            let contact = CoupledContact {
                speed: [axes[0].dot(&velocity), axes[1].dot(&velocity)],
                response,
                ..example_contact()
            };
            let result = contact.solve();
            bodies[chassis].apply_impulse_at_point(
                axes[0] * result.tangent[0] + axes[1] * result.tangent[1],
                point,
                false,
            );
            let after = bodies[chassis].velocity_at_point(&point);
            assert!((axes[0].dot(&after) - result.speed[0]).abs() < 0.0001);
            assert!((axes[1].dot(&after) - result.speed[1]).abs() < 0.0001);
            check_solution(contact, result);
        }
    }

    #[test]
    fn cornering_torque_sweeps_preserve_assist_search_ordering() {
        for direction in [-1.0, 1.0] {
            for side in [-20.0, 0.0, 20.0] {
                for grip in [1.0, 50.0, 500.0] {
                    let contact = CoupledContact {
                        speed: [40.0 * direction, side],
                        omega: 40.0 * direction / 0.35,
                        grip_impulse: grip,
                        ..example_contact()
                    };
                    let mut last_drive_gap = -Real::MAX;
                    let mut last_brake_gap = -Real::MAX;
                    for step in 0..=200 {
                        let fraction = step as Real / 200.0;
                        let driven = CoupledContact {
                            omega: contact.omega + direction * fraction * 100.0,
                            ..contact
                        }
                        .solve();
                        let drive_gap =
                            (driven.omega * contact.radius - driven.speed[0]) * direction;
                        assert!(drive_gap >= last_drive_gap - 0.001);
                        last_drive_gap = drive_gap;
                        let braked = CoupledContact {
                            brake_budget: fraction * 1_000.0,
                            ..contact
                        }
                        .solve();
                        let brake_gap =
                            (braked.speed[0] - braked.omega * contact.radius) * direction;
                        assert!(brake_gap >= last_brake_gap - 0.001);
                        last_brake_gap = brake_gap;
                    }
                }
            }
        }
    }

    #[test]
    fn locked_contact_recovers_through_inertia_after_brake_release() {
        for hz in [30, 60, 120] {
            let dt = 1.0 / hz as Real;
            let mut contact = CoupledContact {
                speed: [40.0, 0.0],
                omega: 0.0,
                brake_budget: 0.0,
                response: [[1.0 / 300.0, 0.0], [0.0, 1.0 / 300.0]],
                grip_impulse: 3_000.0 * 0.05 * 0.85 * dt,
                ..example_contact()
            };
            for step in 0..hz * 8 {
                let result = contact.solve();
                check_solution(contact, result);
                if step == 0 {
                    assert!(result.omega > 0.0 && result.omega * contact.radius < 2.0);
                }
                contact.omega = result.omega;
                contact.speed = result.speed;
            }
            assert!((contact.omega * contact.radius - contact.speed[0]).abs() < 0.001);
        }
    }

    #[test]
    fn nested_abs_tc_preview_cost_smoke_test() {
        let start = std::time::Instant::now();
        let mut calls = 0;
        for step in 0..200 {
            let contact = CoupledContact {
                speed: [40.0, step as Real * 0.1],
                brake_budget: if step < 100 { 100.0 } else { 1_000.0 },
                ..example_contact()
            };
            let predict = |drive_fraction: Real, brake_fraction: Real| {
                CoupledContact {
                    omega: contact.omega + 100.0 * drive_fraction,
                    brake_budget: contact.brake_budget * brake_fraction,
                    ..contact
                }
                .solve()
            };
            let counter = std::cell::Cell::new(0);
            let tc = |brake_fraction| {
                super::traction_control_torque_fraction(0.8, |drive_fraction| {
                    counter.set(counter.get() + 1);
                    let result = predict(drive_fraction, brake_fraction);
                    result.omega * contact.radius - result.speed[0]
                })
            };
            let brake_fraction = super::anti_lock_brake_torque_fraction(0.8, 1.0, |fraction| {
                let result = predict(tc(fraction), fraction);
                counter.set(counter.get() + 1);
                (result.omega * contact.radius, result.speed[0])
            });
            let drive_fraction = tc(brake_fraction);
            let result = predict(drive_fraction, brake_fraction);
            std::hint::black_box(result);
            assert!((0.0..=1.0).contains(&brake_fraction));
            assert!((0.0..=1.0).contains(&drive_fraction));
            assert!(result.omega * contact.radius - result.speed[0] <= 4.011);
            assert!(
                result.speed[0] - result.omega * contact.radius
                    <= result.speed[0].abs() * 0.28 + 0.011
            );
            if step >= 100 {
                assert!(brake_fraction < 1.0, "exercise active ABS as well as TC");
            }
            calls += counter.get() + 1;
        }
        eprintln!(
            "200 coupled ABS/TC previews: {calls} contact solves in {:?}",
            start.elapsed()
        );
    }

    fn tire_test_contact() -> CoupledContact {
        CoupledContact {
            speed: [40.0, 0.0],
            response: [[0.003, 0.0], [0.0, 0.003]],
            omega: 40.0 / 0.35,
            radius: 0.35,
            inertia: 1.5,
            grip_impulse: 50.0,
            brake_budget: 0.0,
        }
    }

    #[test]
    fn surface_pairs_preserve_original_combined_speed_transition() {
        for ratio in [0.0, 0.5, 0.85, 1.0] {
            let tire = TireFriction::new(1.2, 1.2 * ratio);
            for (speed, blend) in [
                (0.0, 0.0),
                (3.0, 0.0),
                (4.0, 0.0),
                (6.0, 0.5),
                (8.0, 1.0),
                (20.0, 1.0),
            ] {
                for (x, y) in [(1.0, 0.0), (0.0, 1.0), (0.6, 0.8), (-0.6, -0.8)] {
                    assert!(
                        (tire.multiplier(speed * x, speed * y) - (1.0 - (1.0 - ratio) * blend))
                            .abs()
                            < 1e-6
                    );
                }
            }
        }
    }

    #[test]
    fn ordinary_contact_keeps_rigid_traction_without_force_buildup() {
        for road in [5.0, 30.0, 80.0] {
            for side in [0.01, 0.1, 0.5, 3.0] {
                let c = CoupledContact {
                    speed: [road, side],
                    omega: road / 0.35,
                    ..tire_test_contact()
                };
                let expected = c.solve();
                let kinetic = TireFriction::new(1.0, 0.85).multiplier(0.0, side);
                let (actual, grip) = solve_with_grip_recovery(c, kinetic, 1.0, 0.85, 1.0 / 60.0);
                assert_eq!(grip, 1.0);
                assert_eq!(actual.tangent, expected.tangent);
                assert_eq!(actual.speed, expected.speed);
                assert_eq!(actual.omega, expected.omega);
                if side <= 0.1 {
                    assert!(actual.speed[1].abs() < 1e-5);
                }
            }
        }
    }

    #[test]
    fn recovery_uses_original_baseline_boost_and_immutable_previews() {
        for hz in [30, 60, 120] {
            let dt = 1.0 / hz as Real;
            let c = tire_test_contact();
            let (_, boosted) = solve_with_grip_recovery(c, 1.0, 0.85, 0.85, dt);
            assert!((boosted - (0.85 + 0.3 * dt)).abs() < 1e-6);
            let overloaded = CoupledContact { omega: 200.0, ..c };
            let (first, grip) = solve_with_grip_recovery(overloaded, 1.0, 0.85, 0.85, dt);
            assert!((grip - (0.85 + 0.15 * dt)).abs() < 1e-6);
            for _ in 0..24 {
                let (preview, g) = solve_with_grip_recovery(overloaded, 1.0, 0.85, 0.85, dt);
                assert_eq!(preview.tangent, first.tangent);
                assert_eq!(g, grip);
            }
            let mut recovered = 0.85;
            for _ in 0..hz {
                recovered = solve_with_grip_recovery(c, 1.0, recovered, 0.85, dt).1;
            }
            assert_eq!(recovered, 1.0);
            let (_, dropped) = solve_with_grip_recovery(c, 0.85, recovered, 0.85, dt);
            assert_eq!(dropped, 0.85);
        }
    }

    #[test]
    fn surface_changes_and_zero_sliding_grip_are_finite_and_not_double_scaled() {
        for (peak, sliding) in [(1.2, 1.02), (0.55, 0.55), (1.0, 0.0), (0.0, 0.0)] {
            let friction = TireFriction::new(peak, sliding);
            let c = CoupledContact {
                speed: [40.0, 20.0],
                omega: 0.0,
                grip_impulse: 50.0 * peak,
                brake_budget: 1e5,
                ..tire_test_contact()
            };
            let kinetic = friction.multiplier(-40.0, 20.0);
            let (result, grip) =
                solve_with_grip_recovery(c, kinetic, 0.85, friction.sliding_ratio(), 0.02);
            assert_eq!(grip, friction.sliding_ratio());
            assert!(result.omega.is_finite() && result.speed.iter().all(|v| v.is_finite()));
            assert!((result.tangent[0].hypot(result.tangent[1]) - 50.0 * sliding).abs() < 1e-4);
        }
    }

    fn esc_test_controller() -> DynamicRayCastVehicleController {
        let mut controller = DynamicRayCastVehicleController::new(
            RigidBodyHandle::invalid(),
            VehicleControllerConfig::default(),
        );
        controller.index_forward_axis = 2;
        controller.index_up_axis = 1;
        controller.current_vehicle_speed = 20.0;

        for (position, axle) in [
            (Point::new(0.8, 0.0, 1.25), WheelAxle::Front),
            (Point::new(-0.8, 0.0, 1.25), WheelAxle::Front),
            (Point::new(0.8, 0.0, -1.25), WheelAxle::Rear),
            (Point::new(-0.8, 0.0, -1.25), WheelAxle::Rear),
        ] {
            controller.add_wheel(
                position,
                -Vector::y(),
                -Vector::x(),
                0.4,
                0.35,
                &WheelTuning::default(),
                WheelRole::new(axle, true, axle == WheelAxle::Front),
            );
        }

        controller
    }

    fn friction_test_controller(
        forward_speed: Real,
        grounded: bool,
        suspension_force: Real,
    ) -> (DynamicRayCastVehicleController, RigidBodySet, ColliderSet) {
        let mut bodies = RigidBodySet::new();
        let chassis = bodies.insert(
            RigidBodyBuilder::dynamic()
                .linvel(Vector::z() * forward_speed)
                .build(),
        );
        let mut colliders = ColliderSet::new();
        let ground =
            grounded.then(|| colliders.insert(crate::geometry::ColliderBuilder::ball(1.0)));
        let mut config = VehicleControllerConfig::default();
        config.dynamics.traction_control_strength = 0.0;
        config.dynamics.esc_strength = 0.0;
        let mut controller = DynamicRayCastVehicleController::new(chassis, config);
        controller.index_forward_axis = 2;
        controller.index_up_axis = 1;
        controller.current_vehicle_speed = forward_speed;
        let wheel = controller.add_wheel(
            Point::origin(),
            -Vector::y(),
            Vector::x(),
            0.4,
            WHEEL_REFERENCE_RADIUS,
            &WheelTuning::default(),
            WheelRole::new(WheelAxle::Rear, true, false),
        );
        wheel.traction_control = 0.0;
        wheel.raycast_info.is_in_contact = grounded;
        wheel.raycast_info.ground_object = ground;
        wheel.raycast_info.contact_normal_ws = Vector::y();
        wheel.raycast_info.contact_point_ws = Point::origin();
        wheel.wheel_suspension_force = suspension_force;

        (controller, bodies, colliders)
    }

    fn four_wheel_test_vehicle(
        speed: Real,
        traction_control: Real,
    ) -> (DynamicRayCastVehicleController, RigidBodySet, ColliderSet) {
        let mut bodies = RigidBodySet::new();
        let chassis = bodies.insert(RigidBodyBuilder::dynamic().linvel(Vector::z() * speed));
        let mut colliders = ColliderSet::new();
        colliders.insert_with_parent(
            crate::geometry::ColliderBuilder::cuboid(0.8, 0.3, 1.5).mass(1_200.0),
            chassis,
            &mut bodies,
        );
        bodies[chassis].recompute_mass_properties_from_colliders(&colliders);
        assert!((bodies[chassis].mass() - 1_200.0).abs() < 0.01);
        let ground = colliders.insert(crate::geometry::ColliderBuilder::cuboid(100.0, 0.1, 100.0));
        let mut config = VehicleControllerConfig::default();
        config.transmission.automatic = false;
        config.transmission.auto_reverse = false;
        config.transmission.auto_clutch = true;
        config.transmission.shift_cooldown = 0.0;
        config.transmission.forward_ratios.push(0.7);
        config.engine.torque_curve = vec![(900.0, 600.0), (6_500.0, 600.0)];
        config.dynamics.esc_strength = 0.0;
        config.dynamics.traction_control_strength = traction_control;
        let mut controller = DynamicRayCastVehicleController::new(chassis, config);
        controller.index_forward_axis = 2;
        for (x, z, axle) in [
            (-0.8, 1.3, WheelAxle::Front),
            (0.8, 1.3, WheelAxle::Front),
            (-0.8, -1.3, WheelAxle::Rear),
            (0.8, -1.3, WheelAxle::Rear),
        ] {
            let point = Point::new(x, 0.0, z);
            let wheel = controller.add_wheel(
                point,
                -Vector::y(),
                Vector::x(),
                0.4,
                0.35,
                &WheelTuning::default(),
                WheelRole::new(axle, axle == WheelAxle::Rear, axle == WheelAxle::Front),
            );
            wheel.raycast_info.is_in_contact = true;
            wheel.raycast_info.ground_object = Some(ground);
            wheel.raycast_info.contact_point_ws = point;
            wheel.raycast_info.contact_normal_ws = Vector::y();
            wheel.wheel_suspension_force = 3_000.0;
            wheel.friction_slip = 0.3;
            wheel.angular_velocity = speed / wheel.radius;
        }
        (controller, bodies, colliders)
    }

    fn driven_overspeed(
        controller: &DynamicRayCastVehicleController,
        bodies: &RigidBodySet,
        direction: Real,
    ) -> Real {
        controller
            .wheels
            .iter()
            .filter(|wheel| wheel.role.driven)
            .map(|wheel| {
                let road_speed = bodies[controller.chassis]
                    .velocity_at_point(&wheel.raycast_info.contact_point_ws)
                    .z;
                ((wheel.angular_velocity * wheel.radius - road_speed) * direction).max(0.0)
            })
            .fold(0.0, Real::max)
    }

    #[test]
    fn full_tc_accelerates_without_excess_slip_in_first_sixth_and_reverse() {
        for hz in [30, 60, 120] {
            let dt = 1.0 / hz as Real;
            for (gear, speed, ratio) in [(1, 0.0, 3.2), (6, 40.0, 0.7), (-1, 0.0, -3.2)] {
                let (mut controller, mut bodies, colliders) = four_wheel_test_vehicle(speed, 1.0);
                controller.powertrain.set_gear(gear);
                controller.powertrain.state_mut().current_gear = gear;
                controller.powertrain.state_mut().engine_rpm =
                    (speed / 0.35 * ratio * 3.7 * 60.0 / std::f64::consts::TAU as Real).max(900.0);
                controller.set_input(VehicleInput {
                    throttle: 1.0,
                    ..VehicleInput::default()
                });
                let mut max_overspeed: Real = 0.0;
                for _ in 0..hz * 2 {
                    controller.current_vehicle_speed = bodies[controller.chassis].linvel().z;
                    let output = controller.update_powertrain(dt);
                    controller.apply_powertrain_output(output);
                    controller.update_friction(&mut bodies, &colliders, dt);
                    max_overspeed =
                        max_overspeed.max(driven_overspeed(&controller, &bodies, ratio.signum()));
                }
                let speed_gain = (bodies[controller.chassis].linvel().z - speed) * ratio.signum();
                assert!(
                    max_overspeed < 0.1,
                    "{hz} Hz gear {gear}: overspeed {max_overspeed}, speed gain {speed_gain}"
                );
                assert!(
                    speed_gain > 0.2,
                    "{hz} Hz gear {gear}: insufficient acceleration {speed_gain}"
                );
            }
        }
    }

    fn set_test_drive(controller: &mut DynamicRayCastVehicleController, torque: Real) {
        for wheel in &mut controller.wheels {
            if wheel.role.driven {
                wheel.wheel_coupling_torque = torque;
                wheel.target_rotation = 100.0 * torque.signum();
                wheel.drivetrain_connected = true;
                wheel.drive_throttle = 0.05;
            }
        }
    }

    #[test]
    fn tc_strength_progressively_controls_real_wheelspin_at_light_throttle() {
        let mut overspeeds = Vec::new();
        for strength in [0.0, 0.5, 1.0] {
            let (mut controller, mut bodies, colliders) = four_wheel_test_vehicle(40.0, strength);
            set_test_drive(&mut controller, 1_000.0);
            for _ in 0..120 {
                controller.update_friction(&mut bodies, &colliders, 1.0 / 60.0);
            }
            overspeeds.push(driven_overspeed(&controller, &bodies, 1.0));
            for wheel in controller.wheels.iter().filter(|w| w.role.driven) {
                if strength == 0.0 {
                    assert_eq!(wheel.traction_control_cut, 0.0);
                } else {
                    assert!(wheel.traction_control_cut > 0.0);
                    assert!(wheel.traction_control_cut <= 1.0);
                }
            }
            assert!(bodies[controller.chassis].linvel().z > 40.2);
        }
        assert!(
            overspeeds[0] > overspeeds[1] && overspeeds[1] > 1.0,
            "{overspeeds:?}"
        );
        assert!(overspeeds[2] < 0.1, "{overspeeds:?}");
    }

    #[test]
    fn full_tc_regrips_after_airborne_spin_and_handles_changing_cornering_capacity() {
        for hz in [30, 60, 120] {
            let dt = 1.0 / hz as Real;
            let (mut controller, mut bodies, colliders) = four_wheel_test_vehicle(20.0, 1.0);
            set_test_drive(&mut controller, 1_000.0);
            let ground = controller.wheels[0].raycast_info.ground_object;
            for wheel in &mut controller.wheels {
                wheel.raycast_info.ground_object = None;
                wheel.raycast_info.is_in_contact = false;
            }
            for _ in 0..hz {
                controller.update_friction(&mut bodies, &colliders, dt);
            }
            let airborne_overspeed = driven_overspeed(&controller, &bodies, 1.0);
            assert!(airborne_overspeed > 10.0);
            assert!(controller
                .wheels
                .iter()
                .all(|w| w.traction_control_cut == 0.0));
            assert_eq!(bodies[controller.chassis].linvel().z, 20.0);
            for wheel in &mut controller.wheels {
                wheel.raycast_info.ground_object = ground;
                wheel.raycast_info.is_in_contact = true;
                wheel.friction_slip = 1.5;
            }
            controller.update_friction(&mut bodies, &colliders, dt);
            assert!(controller
                .wheels
                .iter()
                .filter(|w| w.role.driven)
                .all(|w| w.traction_control_cut == 1.0));
            let landing_overspeed = driven_overspeed(&controller, &bodies, 1.0);
            assert!(landing_overspeed > 0.1 && landing_overspeed < airborne_overspeed);
            for _ in 0..hz * 3 {
                controller.update_friction(&mut bodies, &colliders, dt);
            }
            assert!(driven_overspeed(&controller, &bodies, 1.0) < 0.1);
            // Lateral demand and a surface change must immediately reduce admissible drive.
            controller.set_input(VehicleInput {
                steering: 1.0,
                throttle: 0.05,
                ..VehicleInput::default()
            });
            let velocity = *bodies[controller.chassis].linvel();
            bodies[controller.chassis].set_linvel(velocity + Vector::x(), true);
            for wheel in &mut controller.wheels {
                wheel.friction_slip = 0.2;
            }
            for _ in 0..hz {
                controller.update_friction(&mut bodies, &colliders, dt);
                assert!(driven_overspeed(&controller, &bodies, 1.0) < 0.1);
            }
        }
    }

    #[test]
    fn longitudinal_contact_brakes_and_holds_a_massive_four_wheel_vehicle() {
        for hz in [30, 60, 120] {
            for direction in [-1.0, 1.0] {
                let dt = 1.0 / hz as Real;
                let (mut controller, mut bodies, colliders) =
                    four_wheel_test_vehicle(10.0 * direction, 1.0);
                for wheel in &mut controller.wheels {
                    wheel.brake = 1.0;
                    wheel.max_brake_force = 180_000.0;
                    wheel.anti_lock_brake = 0.0;
                    wheel.friction_slip = 1.0;
                }
                let mut distance = 0.0;
                for _ in 0..hz * 5 {
                    controller.current_vehicle_speed = bodies[controller.chassis].linvel().z;
                    controller.update_friction(&mut bodies, &colliders, dt);
                    distance += bodies[controller.chassis].linvel().z.abs() * dt;
                    assert!(controller
                        .wheels
                        .iter()
                        .all(|w| w.angular_velocity.abs() < 0.001));
                }
                assert!(
                    bodies[controller.chassis].linvel().norm() < 0.01,
                    "{hz} Hz direction {direction}: speed after five seconds of braking is {}",
                    bodies[controller.chassis].linvel().norm()
                );
                assert!(distance < 20.0, "{hz} Hz: braking distance {distance}");
                let speed = bodies[controller.chassis].linvel().norm();
                set_test_drive(&mut controller, 50.0 * direction);
                for _ in 0..hz {
                    controller.update_friction(&mut bodies, &colliders, dt);
                }
                assert!(bodies[controller.chassis].linvel().norm() < speed + 0.01);
            }
        }
    }

    #[test]
    fn locked_wheel_steering_does_not_redirect_combined_friction() {
        for hz in [30, 60, 120] {
            for anti_roll in [0.0, 0.3, 1.0] {
                for direction in [-1.0, 1.0] {
                    let mut baseline: Option<(Real, Real)> = None;
                    for steering in [0.0 as Real, -0.4, 0.4] {
                        let dt = 1.0 / hz as Real;
                        let (mut controller, mut bodies, colliders) =
                            four_wheel_test_vehicle(40.0 * direction, 0.0);
                        for wheel in &mut controller.wheels {
                            wheel.anti_roll = anti_roll;
                            wheel.raycast_info.contact_point_ws.y = -0.3;
                            wheel.angular_velocity = 0.0;
                            wheel.anti_lock_brake = 0.0;
                            wheel.brake = 1.0;
                            wheel.max_brake_force = 180_000.0;
                            wheel.friction_slip = 1.0;
                            if wheel.role.steered {
                                wheel.steering = steering;
                                wheel.wheel_axle_ws =
                                    Vector::new(steering.cos(), 0.0, -steering.sin());
                            }
                        }
                        for _ in 0..hz / 2 {
                            controller.current_vehicle_speed =
                                bodies[controller.chassis].linvel().z;
                            controller.update_friction(&mut bodies, &colliders, dt);
                            assert!(controller.wheels.iter().all(|wheel| wheel.lock));
                        }
                        let body = &bodies[controller.chassis];
                        if let Some((yaw, sideways)) = baseline {
                            assert!((body.angvel().y - yaw).abs() < 0.0001
                            && (body.linvel().x - sideways).abs() < 0.0001,
                            "{hz} Hz anti_roll {anti_roll} direction {direction} steering {steering}: yaw {} vs {yaw}, sideways {} vs {sideways}",
                            body.angvel().y, body.linvel().x);
                        } else {
                            baseline = Some((body.angvel().y, body.linvel().x));
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn coupled_contact_applies_equal_linear_reaction_to_dynamic_ground() {
        let (mut controller, mut bodies, mut colliders) = four_wheel_test_vehicle(20.0, 0.0);
        let ground_collider = controller.wheels[0].raycast_info.ground_object.unwrap();
        let ground = bodies.insert(RigidBodyBuilder::dynamic().linvel(Vector::z() * -5.0));
        colliders.set_parent(ground_collider, Some(ground), &mut bodies);
        colliders
            .get_mut(ground_collider)
            .unwrap()
            .set_mass(1_200.0);
        bodies[ground].recompute_mass_properties_from_colliders(&colliders);
        let momentum = bodies[controller.chassis].linvel() * bodies[controller.chassis].mass()
            + bodies[ground].linvel() * bodies[ground].mass();
        for wheel in &mut controller.wheels {
            wheel.angular_velocity = 0.0;
            wheel.brake = 1.0;
            wheel.max_brake_force = 3_000.0;
            wheel.anti_lock_brake = 0.0;
        }
        controller.update_friction(&mut bodies, &colliders, 1.0 / 60.0);
        let after = bodies[controller.chassis].linvel() * bodies[controller.chassis].mass()
            + bodies[ground].linvel() * bodies[ground].mass();
        assert!((after - momentum).norm() < 0.01);
        assert!(bodies[ground].linvel().z > -5.0);
        assert!(bodies[controller.chassis].linvel().z < 20.0);
    }

    #[test]
    fn skid_info_uses_weighted_branch_demand_and_effective_grip() {
        let contact = CoupledContact {
            speed: [0.0, 4.0],
            response: [[1.0, 0.0], [0.0, 1.0]],
            omega: 6.0,
            radius: 1.0,
            inertia: 1.0,
            grip_impulse: 2.5,
            brake_budget: 0.0,
        };
        // Wheel inertia gives a forward request of three; the lateral request
        // is minus four. Only reporting weights that lateral request by 0.2.
        let demand = (3.0 as Real).hypot(0.8);
        let result = contact.solve();
        assert_eq!(result.requested_tangent, [3.0, -4.0]);
        assert!((contact.skid_info(result) - 2.5 / demand).abs() < 1e-6);
        assert!(result.tangent[0].hypot(result.tangent[1]) <= 2.5 + 1e-6);
        for (speed, omega, brake, budget, expected) in [
            ([0.0, 0.0], 10.0, 0.0, 2.5, 0.5),
            ([0.0, 5.0], 0.0, 0.0, 2.5, 1.0),
            ([3.0, 4.0], 0.0, 100.0, 2.5, 2.5 / demand),
            ([0.0, 4.0], 6.0, 0.0, 5.0, 1.0),
            ([6.0, 0.0], 6.0, 0.0, 2.5, 1.0),
            ([6.0, 0.0], 6.0, 0.0, 0.0, 0.0),
        ] {
            let sample = CoupledContact {
                speed,
                omega,
                brake_budget: brake,
                grip_impulse: budget,
                ..contact
            };
            assert!((sample.skid_info(sample.solve()) - expected).abs() < 1e-6);
        }
        let (result, grip) = solve_with_grip_recovery(contact, 0.85, 1.0, 0.85, 1.0 / 60.0);
        let effective = CoupledContact {
            grip_impulse: contact.grip_impulse * grip,
            ..contact
        };
        assert!((effective.skid_info(result) - 2.125 / demand).abs() < 1e-6);
        assert_eq!(result.tangent, effective.solve().tangent);
    }

    #[test]
    fn locked_wheel_skid_reporting_retains_the_selected_brake_constraint() {
        for direction in [-1.0, 1.0] {
            for hz in [30, 60, 120] {
                let contact = CoupledContact {
                    speed: [20.0 * direction, 0.0],
                    response: [[0.004, 0.0], [0.0, 0.004]],
                    omega: 0.0,
                    radius: 0.35,
                    inertia: 1.2,
                    grip_impulse: 3000.0 / hz as Real,
                    brake_budget: 1500.0 / hz as Real,
                };
                let result = contact.solve();
                assert_eq!(result.omega, 0.0);
                assert!((result.tangent[0] + direction * contact.grip_impulse).abs() < 1e-4);
                assert!((result.requested_tangent[0] + direction * 5000.0).abs() < 0.001);
                assert!((contact.skid_info(result) - contact.grip_impulse / 5000.0).abs() < 1e-6);
                // Relaxing grip would choose a rotating wheel, which must not
                // replace the constraint used to report this locked contact.
                let unlimited = CoupledContact {
                    grip_impulse: Real::MAX,
                    ..contact
                }
                .solve();
                assert!(unlimited.omega.abs() > 1.0);
            }
        }
    }

    #[test]
    fn lateral_skid_reporting_preserves_demand_scale_without_scaling_impulses() {
        for (side_speed, expected) in [(1.0, 1.0), (2.0, 0.5), (10.0, 0.1)] {
            let contact = CoupledContact {
                speed: [20.0, side_speed],
                response: [[0.004, 0.0], [0.0, 0.004]],
                omega: 20.0 / 0.35,
                radius: 0.35,
                inertia: 1.2,
                grip_impulse: 50.0,
                brake_budget: 0.0,
            };
            let result = contact.solve();
            assert!((contact.skid_info(result) - expected).abs() < 1e-5);
            assert!((result.tangent[1] + 50.0).abs() < 1e-4);
        }
    }

    #[test]
    fn controller_skid_info_matches_the_applied_coupled_contact() {
        for brake in [0.0, 1.0] {
            let dt = 1.0 / 60.0;
            let (mut controller, mut bodies, colliders) = four_wheel_test_vehicle(20.0, 0.0);
            bodies[controller.chassis].set_linvel(Vector::new(5.0, 0.0, 20.0), true);
            let wheel = &mut controller.wheels[0];
            wheel.angular_velocity = 0.0;
            wheel.brake = brake;
            wheel.anti_lock_brake = 0.0;
            wheel.max_brake_force = 500.0;
            let axes = [Vector::z(), Vector::x()];
            let body = &bodies[controller.chassis];
            let offset = wheel.raycast_info.contact_point_ws - body.center_of_mass();
            let contact = CoupledContact {
                speed: [20.0, 5.0],
                response: tangent_impulse_response(
                    body,
                    axes,
                    axes.map(|axis| offset.cross(&axis)),
                ),
                omega: 0.0,
                radius: wheel.radius,
                inertia: wheel_angular_inertia(wheel.radius),
                grip_impulse: wheel.wheel_suspension_force.min(wheel.max_suspension_force)
                    * dt
                    * wheel.friction_slip
                    * 0.85,
                brake_budget: wheel.max_brake_force * brake * dt * wheel.radius,
            };
            let expected = contact.solve();
            controller.update_friction(&mut bodies, &colliders, dt);
            let wheel = &controller.wheels[0];
            assert!((wheel.skid_info - contact.skid_info(expected)).abs() < 1e-6);
            assert!((wheel.forward_impulse - expected.tangent[0]).abs() < 1e-5);
            assert!((wheel.side_impulse - expected.tangent[1]).abs() < 1e-5);
        }
    }

    #[test]
    fn capped_suspension_load_limits_grip_and_assist_predictions() {
        for hz in [30, 60, 120] {
            for assist in [0.0, 0.5, 1.0] {
                for brake in [0.0, 1.0] {
                    for cap in [0.0, 100.0, 3000.0] {
                        let mut outcomes = Vec::new();
                        for raw_load in [cap, cap + 20_000.0] {
                            let (mut controller, mut bodies, colliders) =
                                four_wheel_test_vehicle(20.0, assist);
                            set_test_drive(&mut controller, 2000.0);
                            for wheel in &mut controller.wheels {
                                wheel.wheel_suspension_force = raw_load;
                                wheel.max_suspension_force = cap;
                                wheel.brake = brake;
                                wheel.anti_lock_brake = assist;
                                wheel.max_brake_force = 180_000.0;
                            }
                            controller.update_friction(&mut bodies, &colliders, 1.0 / hz as Real);
                            let result: Vec<_> = controller
                                .wheels
                                .iter()
                                .map(|wheel| {
                                    assert!(
                                        wheel.forward_impulse.hypot(wheel.side_impulse)
                                            <= cap / hz as Real
                                                * wheel.friction_slip
                                                * wheel.ground_friction
                                                + 1e-4
                                    );
                                    [
                                        wheel.forward_impulse,
                                        wheel.side_impulse,
                                        wheel.angular_velocity,
                                        wheel.traction_control_cut,
                                        wheel.abs_release,
                                        wheel.skid_info,
                                    ]
                                })
                                .collect();
                            outcomes.push(result);
                        }
                        assert_eq!(
                            outcomes[0], outcomes[1],
                            "{hz} Hz, assist {assist}, brake {brake}, cap {cap}"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn brake_contact_solve_respects_rolling_constraint_and_torque_budget() {
        for inertia in [1.5, 12.0] {
            for inverse_mass in [0.0, 1.0 / 1_200.0] {
                for (speed, omega) in [
                    (0.0, 0.0),
                    (0.0, 20.0),
                    (20.0, 0.0),
                    (20.0, -30.0),
                    (-20.0, -60.0),
                ] {
                    for budget in [0.0, 1.0, 10_000.0] {
                        let radius = 0.35;
                        let (forward, braking) = braked_contact_impulses(
                            omega,
                            speed,
                            radius,
                            inertia,
                            inverse_mass,
                            budget,
                        );
                        let impulse = forward + braking;
                        let mut final_omega = omega - impulse * radius / inertia;
                        let before_brake = final_omega;
                        apply_opposing_angular_impulse(
                            &mut final_omega,
                            speed / radius,
                            inertia,
                            budget,
                        );
                        let final_speed = speed + impulse * inverse_mass;
                        assert!((final_speed - final_omega * radius).abs() < 0.001);
                        assert!((before_brake - final_omega).abs() * inertia <= budget + 0.001);
                        assert!(before_brake * final_omega >= 0.0);
                    }
                }
            }
        }
    }

    #[test]
    fn insufficient_braking_torque_keeps_wheels_rolling() {
        for hz in [30, 60, 120] {
            for direction in [-1.0, 1.0] {
                let dt = 1.0 / hz as Real;
                let (mut controller, mut bodies, colliders) =
                    four_wheel_test_vehicle(20.0 * direction, 0.0);
                for wheel in &mut controller.wheels {
                    wheel.brake = 1.0;
                    wheel.max_brake_force = 300.0;
                    wheel.anti_lock_brake = 0.0;
                    wheel.friction_slip = 1.0;
                }
                for _ in 0..hz {
                    controller.current_vehicle_speed = bodies[controller.chassis].linvel().z;
                    controller.update_friction(&mut bodies, &colliders, dt);
                    assert!(controller
                        .wheels
                        .iter()
                        .all(|wheel| !wheel.lock && wheel.angular_velocity * direction > 0.0));
                }
                let speed = bodies[controller.chassis].linvel().z * direction;
                assert!(speed > 18.0 && speed < 20.0, "{hz} Hz: speed {speed}");
            }
        }
    }

    #[test]
    fn released_locked_wheels_recover_promptly_at_high_speed() {
        for hz in [30, 60, 120] {
            for speed in [40.0, 60.0, -40.0] {
                let dt = 1.0 / hz as Real;
                let (mut controller, mut bodies, colliders) = four_wheel_test_vehicle(speed, 0.0);
                for wheel in &mut controller.wheels {
                    wheel.brake = 1.0;
                    wheel.max_brake_force = 180_000.0;
                    wheel.anti_lock_brake = 0.0;
                    wheel.friction_slip = 1.0;
                }
                for _ in 0..hz / 4 {
                    controller.current_vehicle_speed = bodies[controller.chassis].linvel().z;
                    controller.update_friction(&mut bodies, &colliders, dt);
                }
                assert!(controller.wheels.iter().all(|wheel| wheel.lock));
                assert!(bodies[controller.chassis].linvel().z.abs() > 30.0);
                for wheel in &mut controller.wheels {
                    wheel.brake = 0.0;
                }

                let mut recovered = false;
                for step in 0..hz / 2 {
                    controller.current_vehicle_speed = bodies[controller.chassis].linvel().z;
                    controller.update_friction(&mut bodies, &colliders, dt);
                    let gap = controller
                        .wheels
                        .iter()
                        .map(|wheel| {
                            let road_speed = bodies[controller.chassis]
                                .velocity_at_point(&wheel.raycast_info.contact_point_ws)
                                .z;
                            (wheel.angular_velocity * wheel.radius - road_speed).abs()
                        })
                        .fold(0.0, Real::max);
                    assert!(controller.wheels.iter().all(|wheel| !wheel.lock));
                    if step == 0 {
                        // Release must still spin the wheels up through tire reaction.
                        assert!(gap > 1.0);
                    }
                    // A compliant tire approaches rolling continuously, not via
                    // exact speed synchronization: require less than 1% slip.
                    if gap < speed.abs() * 0.01 {
                        recovered = true;
                        break;
                    }
                }
                assert!(
                    recovered,
                    "{hz} Hz at {speed} m/s: wheels did not recover within 0.5 seconds"
                );
                for _ in 0..hz / 2 {
                    controller.update_friction(&mut bodies, &colliders, dt);
                }
                for wheel in &controller.wheels {
                    let road = bodies[controller.chassis]
                        .velocity_at_point(&wheel.raycast_info.contact_point_ws)
                        .z;
                    assert!((wheel.angular_velocity * wheel.radius - road).abs() < 0.1);
                }
            }
        }
    }

    #[test]
    fn abs_slip_target_scales_with_speed_without_limiting_brake_release() {
        assert_eq!(
            anti_lock_brake_torque_fraction(0.0, 1.0, |_| panic!("ABS off")),
            1.0
        );
        for strength in [0.05, 0.2, 0.5, 0.8, 1.0] {
            for speed in [2.0, 20.0, 80.0] {
                for direction in [-1.0, 1.0] {
                    let allowance = speed * (1.0 - strength);
                    let fraction = anti_lock_brake_torque_fraction(strength, direction, |f| {
                        (speed * (1.0 - 2.0 * f) * direction, speed * direction)
                    });
                    assert!(
                        (fraction - (allowance + ASSIST_SURFACE_SPEED_TOLERANCE) / (2.0 * speed))
                            .abs()
                            < 0.000001
                    );
                    // Below the target, ABS must not manufacture braking slip.
                    assert_eq!(
                        anti_lock_brake_torque_fraction(strength, direction, |_| {
                            (speed * direction, speed * direction)
                        }),
                        1.0
                    );
                    // A locked wheel can require full release at any enabled strength.
                    assert_eq!(
                        anti_lock_brake_torque_fraction(strength, direction, |_| {
                            (0.0, speed * direction)
                        }),
                        0.0
                    );
                }
            }
        }
    }

    #[test]
    fn partial_abs_controls_slip_on_low_grip_and_surface_changes() {
        for hz in [30, 60, 120] {
            for direction in [-1.0, 1.0] {
                for speed in [20.0, 60.0] {
                    for strength in [0.0, 0.05, 0.2, 0.5, 0.8, 1.0] {
                        for initial_grip in [0.05, 0.2, 1.0] {
                            let dt = 1.0 / hz as Real;
                            let (mut controller, mut bodies, colliders) =
                                four_wheel_test_vehicle(speed * direction, 0.0);
                            for wheel in &mut controller.wheels {
                                wheel.anti_lock_brake = strength;
                                wheel.brake = 1.0;
                                wheel.max_brake_force = 180_000.0;
                                wheel.friction_slip = 1.0;
                            }
                            let mut locked = false;
                            let mut maximum_release: Real = 0.0;
                            for step in 0..hz {
                                let grip = if step < hz / 2 { initial_grip } else { 0.05 };
                                controller
                                    .tire_types
                                    .get_mut("default")
                                    .unwrap()
                                    .default_friction = TireFriction::new(grip, grip * 0.85);
                                controller.current_vehicle_speed =
                                    bodies[controller.chassis].linvel().z;
                                controller.update_friction(&mut bodies, &colliders, dt);
                                for wheel in &controller.wheels {
                                    locked |= wheel.lock;
                                    maximum_release = maximum_release.max(wheel.abs_release);
                                    if strength > 0.0 {
                                        let road = bodies[controller.chassis]
                                            .velocity_at_point(&wheel.raycast_info.contact_point_ws)
                                            .z;
                                        let slip = (road - wheel.angular_velocity * wheel.radius)
                                            * direction;
                                        assert!(!wheel.lock && slip <= road.abs() * (1.0 - strength) + 0.1,
                                            "{hz} Hz speed {speed} direction {direction} grip {grip} ABS {strength}: slip {slip}, road {road}");
                                    }
                                }
                            }
                            assert_eq!(locked, strength == 0.0);
                            if strength > 0.0 && strength < 1.0 {
                                assert!(maximum_release > strength,
                                    "ABS {strength} must release beyond the old strength cap: {maximum_release}");
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn partial_abs_releases_existing_lock_on_low_grip() {
        for hz in [30, 60, 120] {
            for direction in [-1.0, 1.0] {
                for strength in [0.05, 0.2, 0.5, 0.8] {
                    let dt = 1.0 / hz as Real;
                    let (mut controller, mut bodies, colliders) =
                        four_wheel_test_vehicle(40.0 * direction, 0.0);
                    controller
                        .tire_types
                        .get_mut("default")
                        .unwrap()
                        .default_friction = TireFriction::new(0.05, 0.05);
                    controller.current_vehicle_speed = 40.0 * direction;
                    for wheel in &mut controller.wheels {
                        wheel.anti_lock_brake = strength;
                        wheel.angular_velocity = 0.0;
                        wheel.brake = 1.0;
                        wheel.max_brake_force = 180_000.0;
                        wheel.friction_slip = 1.0;
                    }
                    controller.update_friction(&mut bodies, &colliders, dt);
                    for wheel in &controller.wheels {
                        assert_eq!(wheel.abs_release, 1.0);
                        assert!(!wheel.lock && wheel.angular_velocity * direction > 0.0);
                        assert!(
                            wheel.angular_velocity.abs() * wheel.radius < 2.0,
                            "recovery must integrate tire torque, not snap to road speed"
                        );
                    }
                    // Very low grip needs several seconds to spin an already locked
                    // wheel back up through its physical inertia.
                    for _ in 0..hz * 8 {
                        controller.current_vehicle_speed = bodies[controller.chassis].linvel().z;
                        controller.update_friction(&mut bodies, &colliders, dt);
                    }
                    for wheel in &controller.wheels {
                        let road = bodies[controller.chassis]
                            .velocity_at_point(&wheel.raycast_info.contact_point_ws)
                            .z;
                        let slip = (road - wheel.angular_velocity * wheel.radius) * direction;
                        assert!(!wheel.lock && slip <= road.abs() * (1.0 - strength) + 0.1,
                            "{hz} Hz direction {direction} ABS {strength}: recovery slip {slip}, road {road}");
                    }
                }
            }
        }
    }

    #[test]
    fn full_abs_prevents_braking_lock_with_useful_deceleration() {
        for hz in [30, 60, 120] {
            for direction in [-1.0, 1.0] {
                for abs in [0.0, 1.0] {
                    let dt = 1.0 / hz as Real;
                    let (mut controller, mut bodies, colliders) =
                        four_wheel_test_vehicle(40.0 * direction, 0.0);
                    controller.set_input(VehicleInput {
                        brake: 1.0,
                        clutch: 1.0,
                        ..VehicleInput::default()
                    });
                    for wheel in &mut controller.wheels {
                        wheel.brake = 1.0;
                        wheel.max_brake_force = 180_000.0;
                        wheel.anti_lock_brake = abs;
                        wheel.friction_slip = 1.0;
                        // Start from rolling contact, not a pre-seeded ABS signal.
                        wheel.last_skid_info = 1.0;
                    }
                    let mut locked = false;
                    let mut active = false;
                    for _ in 0..hz * 2 {
                        controller.current_vehicle_speed = bodies[controller.chassis].linvel().z;
                        let output = controller.update_powertrain(dt);
                        controller.apply_powertrain_output(output);
                        controller.update_friction(&mut bodies, &colliders, dt);
                        for wheel in &controller.wheels {
                            locked |= wheel.lock;
                            active |= wheel.is_anti_lock_brake;
                            if abs == 1.0 {
                                let road = bodies[controller.chassis]
                                    .velocity_at_point(&wheel.raycast_info.contact_point_ws)
                                    .z;
                                let underspeed =
                                    (road - wheel.angular_velocity * wheel.radius) * direction;
                                assert!(!wheel.lock && underspeed < 0.1,
                                    "{hz} Hz direction {direction}: ABS wheel underspeed {underspeed}");
                            }
                        }
                    }
                    assert_eq!(locked, abs == 0.0);
                    assert_eq!(active, abs > 0.0);
                    let speed = bodies[controller.chassis].linvel().z * direction;
                    assert!(
                        speed < 35.0 && speed > 1.0,
                        "{hz} Hz ABS {abs}: insufficient braking or premature stop, speed {speed}"
                    );
                }
            }
        }
    }

    #[test]
    fn full_abs_releases_existing_lock_and_recovers_without_speed_sync() {
        for hz in [30, 60, 120] {
            for direction in [-1.0, 1.0] {
                let dt = 1.0 / hz as Real;
                let (mut controller, mut bodies, colliders) =
                    four_wheel_test_vehicle(40.0 * direction, 0.0);
                let (mut released, mut released_bodies, released_colliders) =
                    four_wheel_test_vehicle(40.0 * direction, 0.0);
                for vehicle in [&mut controller, &mut released] {
                    vehicle.current_vehicle_speed = 40.0 * direction;
                    for wheel in &mut vehicle.wheels {
                        wheel.angular_velocity = 0.0;
                        wheel.last_skid_info = 1.0;
                        wheel.friction_slip = 1.0;
                        wheel.anti_lock_brake = 1.0;
                        wheel.max_brake_force = 180_000.0;
                    }
                }
                for wheel in &mut controller.wheels {
                    wheel.brake = 1.0;
                }
                controller.update_friction(&mut bodies, &colliders, dt);
                released.update_friction(&mut released_bodies, &released_colliders, dt);
                for (wheel, unbraked) in controller.wheels.iter().zip(&released.wheels) {
                    assert!(wheel.is_anti_lock_brake && !wheel.lock);
                    assert!((wheel.angular_velocity - unbraked.angular_velocity).abs() < 0.001);
                    assert!(
                        wheel.angular_velocity.abs() > 0.0
                            && wheel.angular_velocity.abs() * wheel.radius < 20.0
                    );
                }
                for _ in 0..hz * 2 {
                    controller.current_vehicle_speed = bodies[controller.chassis].linvel().z;
                    controller.update_friction(&mut bodies, &colliders, dt);
                }
                for wheel in &controller.wheels {
                    let road = bodies[controller.chassis]
                        .velocity_at_point(&wheel.raycast_info.contact_point_ws)
                        .z;
                    assert!(
                        !wheel.lock
                            && (road - wheel.angular_velocity * wheel.radius) * direction < 0.1
                    );
                }
                assert!(bodies[controller.chassis].linvel().z.abs() < 35.0);
            }
        }
    }

    #[test]
    fn progressive_abs_recovers_during_cornering_with_tc_and_esc() {
        for hz in [30, 60, 120] {
            for strength in [0.2, 0.5, 0.8, 0.99, 1.0] {
                let dt = 1.0 / hz as Real;
                let (mut controller, mut bodies, colliders) =
                    four_wheel_test_vehicle(40.0, strength);
                controller.esc = 1.0;
                controller.set_input(VehicleInput {
                    steering: 0.12,
                    throttle: 0.3,
                    brake: 1.0,
                    ..VehicleInput::default()
                });
                bodies[controller.chassis].set_linvel(Vector::new(2.0, 0.0, 40.0), true);
                bodies[controller.chassis].set_angvel(Vector::y() * 0.1, true);
                set_test_drive(&mut controller, 600.0);
                for wheel in &mut controller.wheels {
                    wheel.anti_lock_brake = strength;
                    wheel.brake = 1.0;
                    wheel.max_brake_force = 60_000.0;
                    wheel.friction_slip = 0.6;
                    wheel.last_skid_info = 1.0;
                    if wheel.role.steered {
                        wheel.steering = 0.12;
                        wheel.wheel_axle_ws =
                            Vector::new((0.12 as Real).cos(), 0.0, -(0.12 as Real).sin());
                    }
                    let forward =
                        aligned_wheel_forward(&Vector::y(), &wheel.wheel_axle_ws, &Vector::z());
                    wheel.angular_velocity = forward.dot(
                        &bodies[controller.chassis]
                            .velocity_at_point(&wheel.raycast_info.contact_point_ws),
                    ) / wheel.radius;
                }
                let mut abs_active = false;
                let mut esc_active = false;
                for _ in 0..hz * 3 {
                    controller.current_vehicle_speed = bodies[controller.chassis].linvel().z;
                    controller.update_friction(&mut bodies, &colliders, dt);
                    esc_active |= controller.state().esc_activity > 0.0;
                    for wheel in &controller.wheels {
                        abs_active |= wheel.is_anti_lock_brake;
                        let forward =
                            aligned_wheel_forward(&Vector::y(), &wheel.wheel_axle_ws, &Vector::z());
                        let speed = forward.dot(
                            &bodies[controller.chassis]
                                .velocity_at_point(&wheel.raycast_info.contact_point_ws),
                        );
                        let underspeed = speed - wheel.angular_velocity * wheel.radius;
                        assert!(underspeed.is_finite());
                        assert!((0.0..=1.0).contains(&wheel.abs_release));
                        assert!(wheel.traction_control_cut <= 1.0);
                        // Allow for later wheels/lateral impulses changing road speed.
                        assert!(
                            !wheel.lock && underspeed < speed.abs() * (1.0 - strength + 0.03),
                            "{hz} Hz strength {strength} cornering: underspeed {underspeed}"
                        );
                        if strength == 1.0 {
                            assert!(
                                !wheel.lock && underspeed < speed.abs() * 0.03,
                                "{hz} Hz strength {strength} cornering: underspeed {underspeed}"
                            );
                            if wheel.role.driven {
                                assert!(-underspeed < speed.abs() * 0.03);
                            }
                        }
                    }
                }
                assert!(abs_active && esc_active);
                assert!(bodies[controller.chassis].linvel().z < 38.0);
            }
        }
    }

    #[test]
    fn continuous_assist_strengths_reduce_wheelspin_and_braking_lock_duration() {
        for hz in [30, 60, 120] {
            for direction in [-1.0, 1.0] {
                for braking in [false, true] {
                    let dt = 1.0 / hz as Real;
                    let mut means = Vec::new();
                    let mut lock_counts = Vec::new();
                    for strength in [0.0, 0.05, 0.2, 0.5, 0.8, 0.95, 1.0] {
                        let (mut controller, mut bodies, colliders) = four_wheel_test_vehicle(
                            40.0 * direction,
                            if braking { 0.0 } else { strength },
                        );
                        if !braking {
                            set_test_drive(&mut controller, 1_000.0 * direction);
                        }
                        for wheel in &mut controller.wheels {
                            wheel.anti_lock_brake = strength;
                            if braking {
                                wheel.brake = 1.0;
                                wheel.max_brake_force = 6_000.0;
                            }
                        }
                        let mut sum = 0.0;
                        let mut samples = 0;
                        let mut locked_steps = 0;
                        for _ in 0..hz * 3 {
                            controller.current_vehicle_speed =
                                bodies[controller.chassis].linvel().z;
                            controller.update_friction(&mut bodies, &colliders, dt);
                            for wheel in controller
                                .wheels
                                .iter()
                                .filter(|w| braking || w.role.driven)
                            {
                                let road = bodies[controller.chassis]
                                    .velocity_at_point(&wheel.raycast_info.contact_point_ws)
                                    .z;
                                let slip = (wheel.angular_velocity * wheel.radius - road)
                                    * direction
                                    * if braking { -1.0 } else { 1.0 };
                                assert!(slip.is_finite());
                                assert!((0.0..=1.0).contains(&wheel.abs_release));
                                assert!(wheel.traction_control_cut <= 1.0);
                                locked_steps += usize::from(wheel.lock);
                                if strength == 0.0 {
                                    assert_eq!(wheel.abs_release, 0.0);
                                    assert_eq!(wheel.traction_control_cut, 0.0);
                                }
                                if strength == 1.0 {
                                    assert!(slip < 0.1 && !wheel.lock);
                                }
                                sum += slip.max(0.0) / road.abs().max(1.0);
                                samples += 1;
                            }
                        }
                        let speed = bodies[controller.chassis].linvel().z * direction;
                        assert!(if braking {
                            speed < 38.0 && speed > 1.0
                        } else {
                            speed > 40.2
                        });
                        means.push(sum / samples as Real);
                        lock_counts.push(locked_steps);
                    }
                    eprintln!("{hz} Hz direction {direction} braking {braking}: {means:?}");
                    for pair in means.windows(2) {
                        assert!(
                            pair[0] > pair[1] + 0.0001,
                            "{hz} Hz direction {direction} braking {braking}: {means:?}"
                        );
                    }
                    if braking {
                        assert!(means[2] > means[4] + 0.1);
                    } else {
                        // TC 0.8 allows one quarter of TC 0.2's fixed gap.
                        assert!((means[4] - means[2] * 0.25).abs() < 0.003);
                        assert!(means[4] < means[0] * 0.5);
                    }
                    if braking {
                        assert!(lock_counts[0] > 0, "ABS off must still permit lock");
                        assert!(lock_counts[1..].iter().all(|count| *count == 0));
                    }
                }
            }
        }
    }

    #[test]
    fn tc_gap_preview_uses_full_correction_only_above_the_allowance() {
        assert_eq!(
            traction_control_torque_fraction(0.0, |_| panic!("TC off")),
            1.0
        );
        for strength in [0.0001, 0.2, 0.4, 0.6, 0.8, 1.0] {
            let gap = TRACTION_CONTROL_MAX_SPEED_GAP * (1.0 - strength);
            assert_eq!(traction_control_torque_fraction(strength, |_| gap), 1.0);
            assert_eq!(traction_control_torque_fraction(strength, |_| -10.0), 1.0);
            let uncontrolled_gap = TRACTION_CONTROL_MAX_SPEED_GAP * 2.0;
            let fraction = traction_control_torque_fraction(strength, |f| uncontrolled_gap * f);
            assert!(
                (fraction - (gap + ASSIST_SURFACE_SPEED_TOLERANCE) / uncontrolled_gap).abs()
                    < 0.000001
            );
            assert_eq!(
                traction_control_torque_fraction(strength, |_| gap + 1.0),
                0.0
            );
        }
    }

    #[test]
    fn tc_gap_is_independent_of_road_speed_and_excess_torque() {
        for hz in [30, 60, 120] {
            for direction in [-1.0, 1.0] {
                for speed in [0.0, 40.0] {
                    for torque in [10.0, 1_000.0, 3_000.0] {
                        for strength in [0.2, 0.4, 0.6, 0.8, 1.0] {
                            let (mut controller, mut bodies, colliders) =
                                four_wheel_test_vehicle(speed * direction, strength);
                            set_test_drive(&mut controller, torque * direction);
                            let dt = 1.0 / hz as Real;
                            let allowance = TRACTION_CONTROL_MAX_SPEED_GAP * (1.0 - strength);
                            for _ in 0..hz * 2 {
                                controller.current_vehicle_speed =
                                    bodies[controller.chassis].linvel().z;
                                controller.update_friction(&mut bodies, &colliders, dt);
                                let slip = driven_overspeed(&controller, &bodies, direction);
                                assert!(
                                    slip < allowance + 0.1,
                                    "{hz} Hz speed {speed} torque {torque} TC {strength}: {slip}"
                                );
                                if torque == 10.0 {
                                    assert!(controller
                                        .wheels
                                        .iter()
                                        .all(|w| w.traction_control_cut == 0.0));
                                    assert!(slip < 0.1, "TC must not manufacture wheelspin");
                                }
                            }
                            let final_slip = driven_overspeed(&controller, &bodies, direction);
                            if torque >= 1_000.0 {
                                assert!((final_slip - allowance).abs() < 0.1,
                                    "{hz} Hz torque {torque} TC {strength}: failed to hold gap {final_slip}/{allowance}");
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn assist_preview_scales_required_correction_without_accumulating() {
        for strength in [0.0, 0.0001, 0.05, 0.2, 0.3, 0.6, 0.8, 0.9999, 1.0] {
            // Full intervention retains 40% of the uncut requested torque.
            let preview = || {
                assist_torque_fraction(strength, |fraction| {
                    (fraction - 0.4) * 10.0 + ASSIST_SURFACE_SPEED_TOLERANCE
                })
            };
            assert!((preview() - (1.0 - strength * 0.6)).abs() < 0.000001);
            for _ in 0..1200 {
                assert_eq!(preview(), preview());
                // Even unrecoverable slip cannot request more than the selected strength.
                assert_eq!(assist_torque_fraction(strength, |_| 100.0), 1.0 - strength);
            }
            assert_eq!(assist_torque_fraction(strength, |_| 0.0), 1.0);
            assert_eq!(assist_torque_fraction(strength, |_| -100.0), 1.0);
        }
        assert_eq!(
            assist_torque_fraction(0.0, |_| panic!("disabled assist previewed")),
            1.0
        );
    }

    #[test]
    fn partial_assists_do_not_prevent_regrip_after_input_release_and_grip_changes() {
        for hz in [30, 60, 120] {
            for strength in [0.2, 0.5, 0.8, 1.0] {
                for braking in [false, true] {
                    let dt = 1.0 / hz as Real;
                    let (mut controller, mut bodies, colliders) =
                        four_wheel_test_vehicle(40.0, strength);
                    if !braking {
                        set_test_drive(&mut controller, 1_000.0);
                    }
                    for wheel in &mut controller.wheels {
                        wheel.angular_velocity = if braking { 0.0 } else { 120.0 / wheel.radius };
                        wheel.anti_lock_brake = strength;
                        wheel.brake = if braking { 1.0 } else { 0.0 };
                        wheel.max_brake_force = 60_000.0;
                    }
                    for step in 0..hz * 12 {
                        let friction = if step < hz * 2 || step >= hz * 4 {
                            1.0
                        } else {
                            0.3
                        };
                        for wheel in &mut controller.wheels {
                            wheel.friction_slip = friction;
                            if step >= hz * 4 {
                                wheel.wheel_coupling_torque = 0.0;
                                wheel.drive_throttle = 0.0;
                                wheel.brake = 0.0;
                            }
                        }
                        controller.current_vehicle_speed = bodies[controller.chassis].linvel().z;
                        controller.update_friction(&mut bodies, &colliders, dt);
                        if step == 0 && !braking {
                            assert!(controller
                                .wheels
                                .iter()
                                .filter(|w| w.role.driven)
                                .all(|w| w.angular_velocity * w.radius > 80.0));
                        }
                    }
                    for wheel in controller
                        .wheels
                        .iter()
                        .filter(|w| braking || w.role.driven)
                    {
                        let road = bodies[controller.chassis]
                            .velocity_at_point(&wheel.raycast_info.contact_point_ws)
                            .z;
                        let slip = (wheel.angular_velocity * wheel.radius - road)
                            * if braking { -1.0 } else { 1.0 };
                        assert_eq!(wheel.traction_control_cut, 0.0);
                        assert_eq!(wheel.abs_release, 0.0);
                        assert!(road.abs() <= 1.0 || (!wheel.lock && slip.abs() < 0.1),
                            "{hz} Hz strength {strength} braking {braking}: recovery slip {slip}, road {road}");
                    }
                }
            }
        }
    }

    #[test]
    fn assist_actuator_memory_clears_on_disable_input_release_and_airborne() {
        for transition in 0..4 {
            let (mut controller, mut bodies, colliders) = four_wheel_test_vehicle(20.0, 0.5);
            set_test_drive(&mut controller, 1_000.0);
            controller.current_vehicle_speed = 20.0;
            for wheel in &mut controller.wheels {
                wheel.abs_release = 0.8;
                wheel.traction_control_cut = 0.8;
                wheel.anti_lock_brake = 0.5;
                wheel.brake = 1.0;
                match transition {
                    0 => {
                        wheel.anti_lock_brake = 0.0;
                        wheel.traction_control = 0.0;
                    }
                    1 => {
                        wheel.brake = 0.0;
                        wheel.wheel_coupling_torque = 0.0;
                        wheel.drive_throttle = 0.0;
                    }
                    2 => {
                        wheel.raycast_info.ground_object = None;
                    }
                    _ => {
                        wheel.handbrake_overrides_abs = true;
                        wheel.wheel_coupling_torque = 0.0;
                        wheel.drive_throttle = 0.0;
                    }
                }
            }
            controller.update_friction(&mut bodies, &colliders, 1.0 / 60.0);
            for wheel in &controller.wheels {
                assert_eq!(wheel.abs_release, 0.0);
                assert_eq!(wheel.traction_control_cut, 0.0);
                assert!(!wheel.is_anti_lock_brake);
            }
        }
    }

    #[test]
    fn tc_launch_wheelspin_reduces_progressively_through_first_gear() {
        for hz in [30, 60, 120] {
            let dt = 1.0 / hz as Real;
            let mut launch_slip = Vec::new();
            for strength in [0.0, 0.0001, 0.2, 0.3, 0.6, 0.8, 0.95, 1.0] {
                let (mut controller, mut bodies, colliders) =
                    four_wheel_test_vehicle(0.0, strength);
                controller.powertrain.set_gear(1);
                controller.powertrain.state_mut().current_gear = 1;
                controller.set_input(VehicleInput {
                    throttle: 1.0,
                    ..VehicleInput::default()
                });
                let mut early_slip = 0.0;
                let mut early_samples = 0;
                let mut max_slip: Real = 0.0;
                let mut spin_onset_rpm = None;
                for step in 0..hz * 24 {
                    controller.current_vehicle_speed = bodies[controller.chassis].linvel().z;
                    let output = controller.update_powertrain(dt);
                    controller.apply_powertrain_output(output);
                    controller.update_friction(&mut bodies, &colliders, dt);
                    let slip = driven_overspeed(&controller, &bodies, 1.0);
                    if slip > 0.5 && spin_onset_rpm.is_none() {
                        spin_onset_rpm = Some(controller.state().engine_rpm);
                    }
                    max_slip = max_slip.max(slip);
                    if step < hz {
                        early_slip += slip;
                        early_samples += 1;
                    }
                    for wheel in controller.wheels.iter().filter(|w| w.role.driven) {
                        assert!(wheel.traction_control_cut <= 1.0);
                    }
                }
                let road_speed = bodies[controller.chassis].linvel().z;
                let first_gear_max = controller.powertrain.config.engine.rev_limit_rpm
                    * std::f64::consts::TAU as Real
                    / 60.0
                    * 0.35
                    / (controller.powertrain.config.transmission.forward_ratios[0]
                        * controller.powertrain.config.transmission.final_drive_ratio);
                assert!(
                    road_speed > first_gear_max * 0.95,
                    "{hz} Hz TC {strength}: first gear stalled at {road_speed}"
                );
                assert_eq!(controller.state().current_gear, 1);
                if strength <= 0.6 {
                    assert!(
                        spin_onset_rpm.is_some_and(
                            |rpm| rpm < controller.powertrain.config.engine.max_rpm * 0.6
                        ),
                        "{hz} Hz TC {strength}: spin delayed until {spin_onset_rpm:?} RPM"
                    );
                }
                if strength > 0.0 {
                    let allowed_gap = TRACTION_CONTROL_MAX_SPEED_GAP * (1.0 - strength);
                    assert!(
                        max_slip < allowed_gap + 0.1,
                        "{hz} Hz TC {strength}: maximum slip {max_slip}, allowance {allowed_gap}"
                    );
                }
                launch_slip.push(early_slip / early_samples as Real);
            }
            eprintln!("{hz} Hz launch slip: {launch_slip:?}");
            assert!(
                launch_slip[2] > 1.0,
                "weak TC must not eliminate launch spin"
            );
            assert!(launch_slip[1] < TRACTION_CONTROL_MAX_SPEED_GAP + 0.1);
            for pair in launch_slip.windows(2) {
                assert!(pair[0] > pair[1], "{hz} Hz launch slip: {launch_slip:?}");
            }
        }
    }

    #[test]
    fn sustained_acceleration_with_wheel_inertia_reaches_automatic_upshifts() {
        for hz in [30, 60, 120] {
            for strength in [0.0, 0.2, 0.5, 0.8, 1.0] {
                let dt = 1.0 / hz as Real;
                let (mut controller, mut bodies, colliders) =
                    four_wheel_test_vehicle(0.0, strength);
                controller.powertrain.config.transmission.automatic = true;
                controller.set_input(VehicleInput {
                    throttle: 1.0,
                    ..VehicleInput::default()
                });
                let mut max_gear = 0;
                let mut negative_power_steps = 0;
                for _ in 0..hz * 40 {
                    controller.current_vehicle_speed = bodies[controller.chassis].linvel().z;
                    let output = controller.update_powertrain(dt);
                    if output.wheel_coupling_torque < 0.0 {
                        negative_power_steps += 1;
                    }
                    controller.apply_powertrain_output(output);
                    controller.update_friction(&mut bodies, &colliders, dt);
                    max_gear = max_gear.max(controller.state().current_gear);
                }
                let speed = bodies[controller.chassis].linvel().z;
                eprintln!("{hz} Hz TC {strength}: speed {speed}, gear {max_gear}, negative steps {negative_power_steps}");
                assert!(
                    negative_power_steps < hz / 2,
                    "sustained drive/brake oscillation"
                );
                assert!(
                    speed > 30.0 && max_gear >= 3,
                    "{hz} Hz TC {strength}: speed {speed}, gear {max_gear}"
                );
                if strength >= 0.8 {
                    let (wheel_speed, radius) = controller.driven_wheel_speed_and_radius();
                    let ratio = controller.powertrain.config.transmission.forward_ratios
                        [(controller.state().current_gear - 1) as usize];
                    let shaft_rpm = wheel_speed / radius
                        * ratio
                        * controller.powertrain.config.transmission.final_drive_ratio
                        * 60.0
                        / std::f64::consts::TAU as Real;
                    assert!((controller.state().engine_rpm - shaft_rpm).abs() < 200.0,
                        "TC must not leave the engine free-revving against controlled wheels: {} / {shaft_rpm}", controller.state().engine_rpm);
                }
            }
        }
    }

    #[test]
    fn tc_memory_survives_limiter_and_overrun_without_cutting_negative_torque() {
        let (mut controller, mut bodies, colliders) = four_wheel_test_vehicle(20.0, 0.5);
        controller.current_vehicle_speed = 20.0;
        set_test_drive(&mut controller, 1000.0);
        for wheel in controller.wheels.iter_mut().filter(|w| w.role.driven) {
            wheel.traction_control_cut = 0.6;
            wheel.wheel_coupling_torque = 0.0;
        }
        controller.update_friction(&mut bodies, &colliders, 1.0 / 60.0);
        for wheel in controller.wheels.iter_mut().filter(|w| w.role.driven) {
            assert_eq!(wheel.traction_control_cut, 0.6);
            wheel.wheel_coupling_torque = -100.0;
        }
        controller.update_friction(&mut bodies, &colliders, 1.0 / 60.0);
        assert!(controller
            .wheels
            .iter()
            .filter(|w| w.role.driven)
            .all(|w| w.traction_control_cut == 0.6));
    }

    #[test]
    fn tc_upshifts_preserve_gap_targets_without_rpm_flares() {
        for strength in [0.2, 0.5, 0.8, 1.0] {
            for hz in [30, 60, 120] {
                let dt = 1.0 / hz as Real;
                let (mut controller, mut bodies, colliders) =
                    four_wheel_test_vehicle(0.0, strength);
                controller.powertrain.config.transmission.automatic = true;
                controller.set_input(VehicleInput {
                    throttle: 1.0,
                    ..VehicleInput::default()
                });
                let mut previous_gear = 0;
                let mut previous_rpm = controller.state().engine_rpm;
                let mut since_shift = Real::MAX;
                let mut shifts = 0;
                let mut max_shift_rise: Real = 0.0;
                let mut max_gap: Real = 0.0;
                for _ in 0..hz * 80 {
                    controller.current_vehicle_speed = bodies[controller.chassis].linvel().z;
                    let output = controller.update_powertrain(dt);
                    controller.apply_powertrain_output(output);
                    controller.update_friction(&mut bodies, &colliders, dt);
                    let state = controller.state();
                    if state.current_gear > previous_gear && previous_gear >= 1 {
                        since_shift = 0.0;
                        shifts += 1;
                    }
                    if since_shift < 0.5 {
                        max_shift_rise = max_shift_rise.max((state.engine_rpm - previous_rpm) / dt);
                    }
                    since_shift += dt;
                    previous_rpm = state.engine_rpm;
                    previous_gear = state.current_gear;
                    max_gap = max_gap.max(driven_overspeed(&controller, &bodies, 1.0));
                }
                eprintln!("TC {strength}, {hz} Hz: {shifts} upshifts, maximum shift rise {max_shift_rise} RPM/s, gap {max_gap} m/s");
                assert!(shifts >= 3);
                assert!(
                    max_gap < TRACTION_CONTROL_MAX_SPEED_GAP * (1.0 - strength) + 0.1,
                    "TC must retain its original fixed speed gap"
                );
                // A multi-thousand-RPM bounce in one or two ticks is a regression,
                // not the gradual engine acceleration following a normal upshift.
                assert!(
                    max_shift_rise < 3_000.0,
                    "post-shift RPM flare: {max_shift_rise}"
                );
            }
        }
    }

    #[test]
    fn ordinary_rolling_does_not_feed_smoke_slip_or_skid_demand() {
        for hz in [30, 60, 120] {
            for speed in [20.0, 40.0, 80.0] {
                let (mut controller, mut bodies, colliders) = four_wheel_test_vehicle(speed, 1.0);
                set_test_drive(&mut controller, 10.0);
                for _ in 0..hz {
                    controller.current_vehicle_speed = bodies[controller.chassis].linvel().z;
                    controller.update_friction(&mut bodies, &colliders, 1.0 / hz as Real);
                    for wheel in &controller.wheels {
                        let gap = (wheel.angular_velocity * wheel.radius
                            - bodies[controller.chassis].linvel().z)
                            .abs();
                        assert!(gap < 0.1, "rolling smoke-speed input: {gap}");
                        assert!(
                            wheel.skid_info > 0.99,
                            "rolling smoke-skid input: {}",
                            wheel.skid_info
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn progressive_tc_operates_through_powertrain_in_first_sixth_and_reverse() {
        for hz in [30, 60, 120] {
            for strength in [0.05, 0.2, 0.5, 0.8, 0.99, 1.0] {
                for (gear, speed, ratio) in [(1, 0.0, 3.2), (6, 40.0, 0.7), (-1, 0.0, -3.2)] {
                    let dt = 1.0 / hz as Real;
                    let (mut controller, mut bodies, colliders) =
                        four_wheel_test_vehicle(speed, strength);
                    controller.powertrain.set_gear(gear);
                    controller.powertrain.state_mut().current_gear = gear;
                    controller.powertrain.state_mut().engine_rpm =
                        (speed / 0.35 * ratio * 3.7 * 60.0 / std::f64::consts::TAU as Real)
                            .max(900.0);
                    controller.set_input(VehicleInput {
                        throttle: 1.0,
                        ..VehicleInput::default()
                    });
                    let mut active = false;
                    for _ in 0..hz * 3 {
                        controller.current_vehicle_speed = bodies[controller.chassis].linvel().z;
                        let output = controller.update_powertrain(dt);
                        controller.apply_powertrain_output(output);
                        controller.update_friction(&mut bodies, &colliders, dt);
                        for wheel in controller.wheels.iter().filter(|w| w.role.driven) {
                            active |= wheel.traction_control_cut > 0.0;
                            assert!(wheel.angular_velocity.is_finite());
                            if strength > 0.0 {
                                let road = bodies[controller.chassis]
                                    .velocity_at_point(&wheel.raycast_info.contact_point_ws)
                                    .z;
                                assert!(
                                    (wheel.angular_velocity * wheel.radius - road) * ratio.signum()
                                        < TRACTION_CONTROL_MAX_SPEED_GAP * (1.0 - strength) + 0.1
                                );
                            }
                        }
                    }
                    assert!(active);
                    assert!((bodies[controller.chassis].linvel().z - speed) * ratio.signum() > 0.2);
                }
            }
        }
    }

    #[test]
    fn abs_strength_and_handbrake_override_survive_powertrain_updates() {
        let (mut controller, mut bodies, colliders) = four_wheel_test_vehicle(20.0, 0.0);
        assert!(controller
            .wheels
            .iter()
            .all(|wheel| wheel.anti_lock_brake == 1.0));
        controller.wheels[0].anti_lock_brake = 0.0;
        controller.wheels[1].anti_lock_brake = 0.5;
        for wheel in &mut controller.wheels {
            wheel.max_brake_force = 60_000.0;
        }
        controller.set_input(VehicleInput {
            handbrake: 1.0,
            ..VehicleInput::default()
        });
        let output = controller.powertrain.update(1.0 / 60.0, 20.0, 20.0, 0.35);
        controller.apply_powertrain_output(output);
        assert_eq!(controller.wheels[0].anti_lock_brake, 0.0);
        assert_eq!(controller.wheels[1].anti_lock_brake, 0.5);
        assert!(controller
            .wheels
            .iter()
            .filter(|wheel| wheel.role.axle == WheelAxle::Rear)
            .all(|wheel| wheel.handbrake_overrides_abs && wheel.anti_lock_brake == 1.0));
        controller.update_friction(&mut bodies, &colliders, 1.0 / 60.0);
        assert!(controller
            .wheels
            .iter()
            .filter(|wheel| wheel.role.axle == WheelAxle::Rear)
            .all(|wheel| wheel.lock && !wheel.is_anti_lock_brake));
        controller.set_input(VehicleInput::default());
        let output = controller.powertrain.update(1.0 / 60.0, 20.0, 0.0, 0.35);
        controller.apply_powertrain_output(output);
        assert!(controller
            .wheels
            .iter()
            .all(|wheel| !wheel.handbrake_overrides_abs));
        controller.reset();
        assert_eq!(controller.wheels[0].anti_lock_brake, 0.0);
        assert_eq!(controller.wheels[1].anti_lock_brake, 0.5);
        assert_eq!(controller.wheels[2].anti_lock_brake, 1.0);
    }

    #[test]
    fn full_abs_allows_low_speed_brake_holding() {
        for hz in [30, 60, 120] {
            let dt = 1.0 / hz as Real;
            let (mut controller, mut bodies, colliders) = four_wheel_test_vehicle(0.5, 0.0);
            for wheel in &mut controller.wheels {
                wheel.brake = 1.0;
                wheel.max_brake_force = 180_000.0;
                wheel.friction_slip = 1.0;
            }
            for _ in 0..hz {
                controller.current_vehicle_speed = bodies[controller.chassis].linvel().z;
                controller.update_friction(&mut bodies, &colliders, dt);
                assert!(controller
                    .wheels
                    .iter()
                    .all(|wheel| wheel.lock && !wheel.is_anti_lock_brake));
            }
            assert!(bodies[controller.chassis].linvel().norm() < 0.01);
        }
    }

    #[test]
    fn tc_shares_weighted_tire_capacity_with_abs_and_esc() {
        for abs in [0.0, 1.0] {
            for esc in [0.0, 1.0] {
                let (mut controller, mut bodies, colliders) = four_wheel_test_vehicle(20.0, 1.0);
                controller.esc = esc;
                controller.current_vehicle_speed = 20.0;
                bodies[controller.chassis].set_angvel(Vector::y(), true);
                controller.set_input(VehicleInput {
                    steering: 0.3,
                    ..VehicleInput::default()
                });
                set_test_drive(&mut controller, 1_000.0);
                for wheel in &mut controller.wheels {
                    wheel.brake = 0.2;
                    wheel.max_brake_force = 60_000.0;
                    wheel.anti_lock_brake = abs;
                    wheel.last_skid_info = 0.1;
                    wheel.angular_velocity = bodies[controller.chassis]
                        .velocity_at_point(&wheel.raycast_info.contact_point_ws)
                        .z
                        / wheel.radius;
                    if wheel.role.steered {
                        wheel.steering = 0.3;
                    }
                }
                controller.update_friction(&mut bodies, &colliders, 1.0 / 60.0);
                assert_eq!(
                    controller.wheels.iter().any(|w| w.is_anti_lock_brake),
                    abs > 0.0
                );
                assert_eq!(controller.state().esc_activity > 0.0, esc > 0.0);
                for _ in 0..120 {
                    controller.current_vehicle_speed = bodies[controller.chassis].linvel().z;
                    controller.update_friction(&mut bodies, &colliders, 1.0 / 60.0);
                    let overspeed = driven_overspeed(&controller, &bodies, 1.0);
                    assert!(
                        overspeed < 0.1,
                        "ABS {abs} ESC {esc}: overspeed {overspeed}"
                    );
                    assert!(controller
                        .wheels
                        .iter()
                        .all(|w| w.angular_velocity.is_finite()
                            && w.skid_info >= 0.0
                            && w.skid_info <= 1.0));
                }
                assert!(bodies[controller.chassis].linvel().norm() < 20.0);
            }
        }
    }

    #[test]
    fn esc_understeer_targets_inside_rear_wheel() {
        let mut controller = esc_test_controller();
        controller.wheels[0].steering = 0.2;
        controller.wheels[1].steering = 0.2;
        let chassis = RigidBodyBuilder::dynamic()
            .linvel(Vector::z() * 20.0)
            .build();

        let intervention = controller.esc_intervention(&chassis);

        assert!(intervention.brake_strength > 0.0);
        assert!(intervention.activity > 0.0);
        assert_eq!(intervention.brake_axle, Some(WheelAxle::Rear));
        assert_eq!(intervention.brake_side, 1.0);
    }

    #[test]
    fn esc_oversteer_targets_outside_front_wheel() {
        let mut controller = esc_test_controller();
        controller.wheels[0].steering = 0.2;
        controller.wheels[1].steering = 0.2;
        let chassis = RigidBodyBuilder::dynamic()
            .linvel(Vector::z() * 20.0)
            .angvel(Vector::y() * 3.0)
            .build();

        let intervention = controller.esc_intervention(&chassis);

        assert!(intervention.brake_strength > 0.0);
        assert_eq!(intervention.brake_axle, Some(WheelAxle::Front));
        assert_eq!(intervention.brake_side, -1.0);
    }

    #[test]
    fn esc_steering_reversal_stabilizes_with_front_wheel() {
        let mut controller = esc_test_controller();
        controller.wheels[0].steering = -0.2;
        controller.wheels[1].steering = -0.2;
        let chassis = RigidBodyBuilder::dynamic()
            .linvel(Vector::z() * 20.0 + Vector::x() * 4.0)
            .angvel(Vector::y())
            .build();

        let intervention = controller.esc_intervention(&chassis);

        assert!(intervention.brake_strength > 0.0);
        assert_eq!(intervention.brake_axle, Some(WheelAxle::Front));
        assert_eq!(intervention.brake_side, -1.0);
    }

    #[test]
    fn esc_sideslip_opposing_yaw_demand_uses_front_wheel() {
        let mut controller = esc_test_controller();
        controller.wheels[0].steering = 0.2;
        controller.wheels[1].steering = 0.2;
        let chassis = RigidBodyBuilder::dynamic()
            .linvel(Vector::z() * 20.0 - Vector::x() * 10.0)
            .angvel(Vector::y())
            .build();

        let intervention = controller.esc_intervention(&chassis);

        assert!(intervention.brake_strength > 0.0);
        assert_eq!(intervention.brake_axle, Some(WheelAxle::Front));
        assert_eq!(intervention.brake_side, -1.0);
    }

    #[test]
    fn esc_detects_sideslip_without_steering_or_yaw_rate() {
        let controller = esc_test_controller();
        let chassis = RigidBodyBuilder::dynamic()
            .linvel(Vector::z() * 20.0 + Vector::x() * 4.0)
            .build();

        let intervention = controller.esc_intervention(&chassis);

        assert!(intervention.brake_strength > 0.0);
        assert_eq!(intervention.brake_axle, Some(WheelAxle::Front));
        assert_eq!(intervention.brake_side, 1.0);
    }

    #[test]
    fn esc_detects_spin_without_steering_or_sideslip() {
        let controller = esc_test_controller();
        let chassis = RigidBodyBuilder::dynamic()
            .linvel(Vector::z() * 20.0)
            .angvel(Vector::y())
            .build();

        let intervention = controller.esc_intervention(&chassis);

        assert!(intervention.brake_strength > 0.0);
        assert_eq!(intervention.brake_axle, Some(WheelAxle::Front));
        assert_eq!(intervention.brake_side, -1.0);
    }

    #[test]
    fn wheel_forward_matches_configured_chassis_forward_for_either_axle_direction() {
        let normal = Vector::y();
        let chassis_forward = Vector::z();

        let positive_axle = aligned_wheel_forward(&normal, &Vector::x(), &chassis_forward);
        let negative_axle = aligned_wheel_forward(&normal, &-Vector::x(), &chassis_forward);

        assert!(positive_axle.dot(&chassis_forward) > 0.999);
        assert!(negative_axle.dot(&chassis_forward) > 0.999);
    }

    #[test]
    fn driven_wheel_speed_preserves_rotation_direction() {
        let mut controller = DynamicRayCastVehicleController::new(
            RigidBodyHandle::invalid(),
            VehicleControllerConfig::default(),
        );
        controller.add_wheel(
            Point::origin(),
            -Vector::y(),
            Vector::x(),
            0.4,
            0.35,
            &WheelTuning::default(),
            WheelRole::new(WheelAxle::Rear, true, false),
        );
        let physical_angular_velocity = -5.0;
        controller.wheels[0].angular_velocity = physical_angular_velocity;

        let (speed, radius) = controller.driven_wheel_speed_and_radius();

        assert!((speed - physical_angular_velocity * radius).abs() < 1.0e-6);
        assert_eq!(radius, 0.35);
    }

    #[test]
    fn anti_roll_bar_transfers_load_without_changing_axle_total() {
        let mut controller = esc_test_controller();
        controller
            .powertrain
            .config
            .dynamics
            .front_anti_roll_bar_stiffness = 20.0;
        for wheel in &mut controller.wheels {
            wheel.raycast_info.is_in_contact = true;
            wheel.raycast_info.suspension_length = wheel.suspension_rest_length;
            wheel.wheel_suspension_force = 1_000.0;
        }
        controller.wheels[1].raycast_info.suspension_length = 0.35;

        controller.apply_anti_roll_bars(1_000.0);

        assert_eq!(controller.wheels[0].wheel_suspension_force, 0.0);
        assert_eq!(controller.wheels[1].wheel_suspension_force, 2_000.0);
        assert_eq!(controller.wheels[2].wheel_suspension_force, 1_000.0);
        assert_eq!(controller.wheels[3].wheel_suspension_force, 1_000.0);
    }

    #[test]
    fn anti_roll_bar_respects_each_wheel_suspension_force_limit() {
        assert_eq!(
            anti_roll_bar_transfer(0.2, 0.0, 40.0, 1_000.0, 950.0, 500.0, 1_000.0, 1_000.0),
            50.0,
        );
        assert_eq!(
            anti_roll_bar_transfer(0.0, 0.2, 40.0, 1_000.0, 500.0, 950.0, 1_000.0, 1_000.0),
            -50.0,
        );
    }

    #[test]
    fn anti_roll_bar_does_not_act_without_two_grounded_axle_wheels() {
        let mut controller = esc_test_controller();
        controller
            .powertrain
            .config
            .dynamics
            .front_anti_roll_bar_stiffness = 20.0;
        for wheel in &mut controller.wheels {
            wheel.raycast_info.suspension_length = wheel.suspension_rest_length;
            wheel.wheel_suspension_force = 1_000.0;
        }
        controller.wheels[0].raycast_info.is_in_contact = true;
        controller.wheels[0].raycast_info.suspension_length = 0.35;

        controller.apply_anti_roll_bars(1_000.0);

        assert_eq!(controller.wheels[0].wheel_suspension_force, 1_000.0);
        assert_eq!(controller.wheels[1].wheel_suspension_force, 1_000.0);
    }

    #[test]
    fn legacy_wheel_anti_roll_defaults_to_disabled() {
        assert_eq!(test_wheel().anti_roll, 0.0);
    }

    fn test_wheel() -> Wheel {
        let mut wheel = Wheel::new(WheelDesc {
            chassis_connection_cs: Point::origin(),
            direction_cs: -Vector::y(),
            axle_cs: Vector::x(),
            suspension_rest_length: 0.4,
            max_suspension_travel: 5.0,
            radius: 0.35,
            suspension_stiffness: 5.88,
            damping_compression: 0.83,
            damping_relaxation: 0.88,
            friction_slip: 10.5,
            max_suspension_force: 6000.0,
            tire_type: "default".to_string(),
            role: WheelRole::new(WheelAxle::Rear, true, false),
        });
        wheel.raycast_info.is_in_contact = true;
        wheel.skid_info = 1.0;
        wheel.target_rotation = 20.0;
        wheel.wheel_coupling_torque = 500.0;
        wheel.drive_throttle = 1.0;
        wheel.drivetrain_connected = true;
        wheel
    }

    #[test]
    fn wheel_inertia_scales_with_radius_squared() {
        let reference = wheel_angular_inertia(WHEEL_REFERENCE_RADIUS);
        let doubled = wheel_angular_inertia(WHEEL_REFERENCE_RADIUS * 2.0);

        assert_eq!(reference, WHEEL_EFFECTIVE_INERTIA);
        assert!((doubled - reference * 4.0).abs() < 1.0e-5);
    }

    #[test]
    fn powered_wheelspin_reallocates_shared_grip_without_a_lateral_multiplier() {
        for hz in [30, 60, 120] {
            let mut impulses = Vec::new();
            for spinning in [false, true] {
                let (mut controller, mut bodies, colliders) = four_wheel_test_vehicle(10.0, 0.0);
                bodies[controller.chassis].set_linvel(Vector::new(1.0, 0.0, 10.0), true);
                set_test_drive(&mut controller, 100.0);
                for wheel in &mut controller.wheels {
                    wheel.wheel_suspension_force = 3_000.0;
                    if spinning && wheel.role.driven {
                        wheel.angular_velocity += 100.0 / wheel.radius;
                    }
                }
                controller.update_friction(&mut bodies, &colliders, 1.0 / hz as Real);
                impulses.push((
                    controller.wheels[0].side_impulse,
                    controller.wheels[2].side_impulse,
                ));
            }
            assert!((impulses[1].0 - impulses[0].0).abs() < 0.001);
            assert!(
                impulses[1].1.abs() < impulses[0].1.abs(),
                "{hz} Hz: wheelspin must leave less lateral grip: {impulses:?}"
            );
        }
    }

    #[test]
    fn rear_drive_allocation_preserves_cornering_speed_difference() {
        let (mut controller, _, _) = four_wheel_test_vehicle(10.0, 0.0);
        set_test_drive(&mut controller, 500.0);
        let mut contacts = vec![WheelContactState::default(); 4];
        for (id, road_speed) in [(2, 8.0), (3, 12.0)] {
            contacts[id].is_grounded = true;
            contacts[id].forward_speed = road_speed;
            controller.wheels[id].angular_velocity = (road_speed + 2.0) / 0.35;
        }
        let mut torques = [0.0, 0.0, 500.0, 500.0];
        controller.redistribute_rear_drive_torque(&contacts, &mut torques, 1.0 / 60.0);
        assert!((torques[2] - 500.0).abs() < 0.001);
        assert!((torques[3] - 500.0).abs() < 0.001);
    }

    #[test]
    fn rear_drive_allocation_corrects_excess_spin_with_inertia_and_timestep() {
        for hz in [30, 60, 120] {
            for direction in [-1.0, 1.0] {
                let (mut controller, _, _) = four_wheel_test_vehicle(10.0 * direction, 0.0);
                set_test_drive(&mut controller, 1_000.0 * direction);
                controller.wheels[3].radius = 0.4;
                let mut contacts = vec![WheelContactState::default(); 4];
                for (id, road_speed, excess) in [(2, 8.0, 3.0), (3, 12.0, 1.0)] {
                    contacts[id].is_grounded = true;
                    contacts[id].forward_speed = road_speed * direction;
                    controller.wheels[id].angular_velocity =
                        (road_speed + excess) * direction / controller.wheels[id].radius;
                }
                let dt = 1.0 / hz as Real;
                let mut torques = [0.0, 0.0, 1_000.0 * direction, 1_000.0 * direction];
                controller.redistribute_rear_drive_torque(&contacts, &mut torques, dt);
                let transfer = 1_000.0 - torques[2] * direction;
                let correction = transfer
                    * dt
                    * (0.35 / wheel_angular_inertia(0.35) + 0.4 / wheel_angular_inertia(0.4));
                assert!(transfer > 0.0);
                assert!((correction - 2.0).abs() < 0.001);
                assert!((torques[2] + torques[3] - 2_000.0 * direction).abs() < 0.001);
            }
        }
    }

    #[test]
    fn rear_drive_allocation_can_transfer_all_drive_but_cannot_create_braking() {
        for spinning_id in [2, 3] {
            for direction in [-1.0, 1.0] {
                let (mut controller, _, _) = four_wheel_test_vehicle(0.0, 0.0);
                set_test_drive(&mut controller, 100.0 * direction);
                controller.wheels[spinning_id].angular_velocity = 100.0 * direction;
                let mut contacts = vec![WheelContactState::default(); 4];
                contacts[2].is_grounded = true;
                contacts[3].is_grounded = true;
                let mut torques = [0.0, 0.0, 100.0 * direction, 100.0 * direction];
                controller.redistribute_rear_drive_torque(&contacts, &mut torques, 1.0 / 60.0);
                assert_eq!(torques[spinning_id], 0.0);
                assert_eq!(torques[5 - spinning_id], 200.0 * direction);
            }
        }
    }

    #[test]
    fn rear_drive_allocation_bypasses_unpowered_braked_or_ungrounded_pairs() {
        for case in 0..8 {
            let (mut controller, _, _) = four_wheel_test_vehicle(0.0, 0.0);
            set_test_drive(&mut controller, 100.0);
            controller.wheels[2].angular_velocity = 100.0;
            let mut contacts = vec![WheelContactState::default(); 4];
            contacts[2].is_grounded = true;
            contacts[3].is_grounded = true;
            let mut torques = [0.0, 0.0, 100.0, 100.0];
            match case {
                0 => contacts[2].is_grounded = false,
                1 => controller.wheels[3].brake = 0.5,
                2 => controller.wheels[2].drive_throttle = 0.0,
                3 => controller.wheels[3].drivetrain_connected = false,
                4 => controller.wheels[3].role.driven = false,
                5 => controller.wheels[3].target_rotation = -100.0,
                6 => torques[2] = -100.0,
                _ => controller.wheels[3].role.axle = WheelAxle::Front,
            }
            let original = torques;
            controller.redistribute_rear_drive_torque(&contacts, &mut torques, 1.0 / 60.0);
            assert_eq!(torques, original, "case {case}");
        }
    }

    #[test]
    fn rear_drive_allocation_sends_real_contact_force_to_the_loaded_wheel() {
        for hz in [30, 60, 120] {
            let mut results = Vec::new();
            for redistribution in [false, true] {
                let (mut controller, mut bodies, colliders) = four_wheel_test_vehicle(10.0, 0.0);
                set_test_drive(&mut controller, 1_000.0);
                controller.wheels[2].angular_velocity += 10.0 / 0.35;
                controller.wheels[2].wheel_suspension_force = 1_000.0;
                controller.wheels[3].wheel_suspension_force = 8_000.0;
                for wheel in &mut controller.wheels[2..] {
                    wheel.friction_slip = 1.0;
                    if !redistribution {
                        // Keep identical wheel torque, bypass only the new allocation.
                        wheel.drive_throttle = 0.0;
                    }
                }
                controller.update_friction(&mut bodies, &colliders, 1.0 / hz as Real);
                results.push((
                    controller.wheels[2].angular_velocity,
                    controller.wheels[3].forward_impulse,
                ));
            }
            assert!(results[1].0 < results[0].0, "{hz} Hz: donor must spin less");
            assert!(
                results[1].1 > results[0].1,
                "{hz} Hz: recipient must gain tire force"
            );
        }
    }

    #[test]
    fn opposing_angular_impulse_stops_without_reversing_the_wheel() {
        let inertia = wheel_angular_inertia(WHEEL_REFERENCE_RADIUS);
        let mut angular_velocity = 2.0;
        let remaining =
            apply_opposing_angular_impulse(&mut angular_velocity, 2.0, inertia, inertia * 3.0);

        assert_eq!(angular_velocity, 0.0);
        assert!((remaining - inertia).abs() < 1.0e-5);
    }

    #[test]
    fn redistributed_drive_feedback_does_not_create_an_engine_rpm_deficit() {
        for hz in [30, 60, 120] {
            for direction in [-1.0, 1.0] {
                for transfer in [-300.0, 300.0] {
                    let (mut controller, _, _) = four_wheel_test_vehicle(15.0 * direction, 0.0);
                    let gear = if direction > 0.0 { 1 } else { -1 };
                    controller.set_gear(gear);
                    controller.powertrain.config.transmission.auto_clutch = false;
                    controller.powertrain.config.turbo.enabled = false;
                    controller.set_input(VehicleInput {
                        throttle: 1.0,
                        ..VehicleInput::default()
                    });
                    controller.powertrain.state_mut().engine_rpm = 5_000.0;
                    let config = &controller.powertrain.config;
                    let ratio = if gear > 0 {
                        config.transmission.forward_ratios[0]
                    } else {
                        config.transmission.reverse_ratio
                    };
                    let gearing = ratio * config.transmission.final_drive_ratio;
                    let base_torque = 600.0
                        * ratio.abs().powf(config.engine.gear_force_exponent)
                        * config.transmission.final_drive_ratio
                        * config.engine.drivetrain_efficiency
                        * config.engine.force_scale
                        * direction
                        / 2.0;
                    let omega = 5_000.0 * std::f64::consts::TAU as Real / 60.0 / gearing;
                    controller.wheels[2].angular_velocity = omega * 0.9;
                    controller.wheels[3].angular_velocity = omega;
                    // Steady wheel motion: road load exactly balances the actual
                    // redistributed torque, including when the fastest wheel donates.
                    for (id, signed_transfer) in [(2, -transfer), (3, transfer)] {
                        controller.wheels[id].drive_torque_transfer = signed_transfer * direction;
                        controller.wheels[id].angular_load =
                            base_torque + signed_transfer * direction;
                    }
                    for _ in 0..hz * 60 {
                        controller.update_powertrain(1.0 / hz as Real);
                    }
                    assert!(
                        (controller.state().engine_rpm - 5_000.0).abs() < 1.0,
                        "{hz} Hz direction {direction} transfer {transfer}: RPM {}",
                        controller.state().engine_rpm
                    );
                }
            }
        }
    }

    #[test]
    fn drive_transfer_feedback_tracks_the_actual_torque_after_tc_and_clears_on_bypass() {
        for strength in [0.0, 0.5, 1.0] {
            let (mut controller, mut bodies, colliders) = four_wheel_test_vehicle(10.0, strength);
            set_test_drive(&mut controller, 1_000.0);
            controller.wheels[2].angular_velocity += 10.0 / 0.35;
            controller.update_friction(&mut bodies, &colliders, 1.0 / 60.0);
            for (id, transfer) in [(2, -1_000.0), (3, 1_000.0)] {
                let wheel = &controller.wheels[id];
                let expected = transfer * (1.0 - wheel.traction_control_cut);
                assert!((wheel.drive_torque_transfer - expected).abs() < 0.001);
            }
            controller.wheels[2].raycast_info.ground_object = None;
            controller.wheels[2].raycast_info.is_in_contact = false;
            controller.update_friction(&mut bodies, &colliders, 1.0 / 60.0);
            assert_eq!(controller.wheels[2].drive_torque_transfer, 0.0);
            assert_eq!(controller.wheels[3].drive_torque_transfer, 0.0);
        }
    }

    #[test]
    fn wheel_rotation_uses_integrated_angular_velocity() {
        let mut wheel = test_wheel();
        wheel.angular_velocity = -12.0;
        let dt = 1.0 / 60.0;

        update_wheel_rotation(&mut wheel, dt);

        assert!((wheel.rotation + 0.2).abs() < 1.0e-6);
        assert!((wheel.delta_rotation + 0.2).abs() < 1.0e-6);
    }

    #[test]
    fn airborne_drive_torque_integrates_wheel_angular_velocity() {
        let (mut controller, mut bodies, colliders) = friction_test_controller(0.0, false, 0.0);
        controller.wheels[0].wheel_coupling_torque = 120.0;
        controller.wheels[0].target_rotation = 1.0;
        controller.wheels[0].drive_throttle = 1.0;
        controller.wheels[0].drivetrain_connected = true;

        controller.update_friction(&mut bodies, &colliders, 0.1);

        let expected = 120.0 * 0.1 / wheel_angular_inertia(WHEEL_REFERENCE_RADIUS);
        assert!((controller.wheels[0].angular_velocity - expected).abs() < 1.0e-5);
        assert_eq!(bodies[controller.chassis].linvel(), &Vector::zeros());
    }

    #[test]
    fn drive_torque_exceeding_available_traction_produces_wheelspin() {
        let (mut controller, mut bodies, colliders) = friction_test_controller(0.0, true, 0.1);
        controller.wheels[0].wheel_coupling_torque = 1_200.0;
        controller.wheels[0].target_rotation = 1.0;
        controller.wheels[0].drive_throttle = 1.0;
        controller.wheels[0].drivetrain_connected = true;

        controller.update_friction(&mut bodies, &colliders, 1.0 / 60.0);

        let wheel_surface_speed =
            controller.wheels[0].angular_velocity * controller.wheels[0].radius;
        let chassis_speed = bodies[controller.chassis].linvel().z;
        assert!(wheel_surface_speed - chassis_speed > 0.5);
        assert!(controller.wheels[0].skid_info < 1.0);
    }

    #[test]
    fn tire_reaction_naturally_regrips_an_overspeeding_wheel() {
        let (mut controller, mut bodies, colliders) = friction_test_controller(0.0, true, 1_000.0);
        controller.wheels[0].angular_velocity = 30.0;
        let initial_speed_gap = controller.wheels[0].angular_velocity * controller.wheels[0].radius;

        for _ in 0..30 {
            controller.update_friction(&mut bodies, &colliders, 1.0 / 60.0);
        }

        let wheel_surface_speed =
            controller.wheels[0].angular_velocity * controller.wheels[0].radius;
        let chassis_speed = bodies[controller.chassis].linvel().z;
        assert!((wheel_surface_speed - chassis_speed).abs() < initial_speed_gap * 0.01);
    }

    #[test]
    fn excessive_grounded_braking_locks_the_wheel_without_reversing_it() {
        let forward_speed = 10.0;
        let (mut controller, mut bodies, colliders) =
            friction_test_controller(forward_speed, true, 1_000.0);
        controller.wheels[0].angular_velocity = forward_speed / controller.wheels[0].radius;
        controller.wheels[0].brake = 1.0;
        controller.wheels[0].anti_lock_brake = 0.0;
        controller.wheels[0].max_brake_force = 120_000.0;

        controller.update_friction(&mut bodies, &colliders, 1.0 / 60.0);

        assert_eq!(controller.wheels[0].angular_velocity, 0.0);
        assert!(controller.wheels[0].lock);
    }

    #[test]
    fn per_wheel_traction_control_strength_is_not_overwritten_by_powertrain_updates() {
        let config = VehicleControllerConfig::default();
        let expected_strength = config.dynamics.traction_control_strength;
        let mut controller =
            DynamicRayCastVehicleController::new(RigidBodyHandle::invalid(), config);
        controller.add_wheel(
            Point::origin(),
            -Vector::y(),
            Vector::x(),
            0.4,
            0.35,
            &WheelTuning::default(),
            WheelRole::new(WheelAxle::Rear, true, false),
        );
        let output = || super::super::vehicle_powertrain::PowertrainOutput {
            drive_torque: 100.0,
            engine_brake_torque: 0.0,
            wheel_coupling_torque: 100.0,
            wheel_target_velocity: 10.0,
            drive_throttle: 1.0,
            drivetrain_connected: true,
            service_brake: 0.0,
        };

        controller.wheels[0].traction_control = 0.25;
        controller.apply_powertrain_output(output());

        assert_ne!(expected_strength, 0.25);
        assert_eq!(controller.wheels[0].traction_control, 0.25);
    }

    #[test]
    fn steering_assist_does_not_countersteer_in_reverse() {
        let mut config = VehicleControllerConfig::default();
        config.steering.assist = true;
        let mut controller =
            DynamicRayCastVehicleController::new(RigidBodyHandle::invalid(), config);
        controller.index_forward_axis = 2;
        controller.index_up_axis = 1;
        controller.current_vehicle_speed = -10.0;
        controller.powertrain.state_mut().wheels_in_contact = 4;

        let chassis = RigidBodyBuilder::dynamic()
            .linvel(-Vector::z() * 10.0 + Vector::x() * 0.1)
            .build();
        controller.update_steering(&chassis, 1.0 / 60.0);

        assert!(controller.state().steering_angle.abs() <= Real::EPSILON);
    }

    #[test]
    fn road_wheel_curve_blends_linear_and_cubic_input_symmetrically() {
        assert_eq!(curved_steering_input(0.5, 0.0), 0.5);
        assert_eq!(curved_steering_input(0.5, 1.0), 0.125);

        let blended = curved_steering_input(0.5, 0.25);
        assert!((blended - 0.40625).abs() < 1.0e-5);
        assert!((curved_steering_input(-0.5, 0.25) + blended).abs() < 1.0e-5);
        assert_eq!(curved_steering_input(0.0, 0.25), 0.0);
        assert_eq!(curved_steering_input(1.0, 0.25), 1.0);
        assert_eq!(curved_steering_input(-1.0, 0.25), -1.0);
    }

    #[test]
    fn road_wheel_curve_is_applied_before_wheel_steering_geometry() {
        let mut config = VehicleControllerConfig::default();
        config.steering.max_angle = 0.6;
        config.steering.road_wheel_curve = 0.25;
        let mut controller =
            DynamicRayCastVehicleController::new(RigidBodyHandle::invalid(), config);
        controller.index_forward_axis = 2;
        controller.index_up_axis = 1;
        controller.add_wheel(
            Point::origin(),
            -Vector::y(),
            Vector::x(),
            0.4,
            0.35,
            &WheelTuning::default(),
            WheelRole::new(WheelAxle::Front, false, true),
        );
        controller.set_input(VehicleInput {
            steering: 0.5,
            ..VehicleInput::default()
        });

        let chassis = RigidBodyBuilder::dynamic().build();
        controller.update_steering(&chassis, 1.0 / 60.0);

        let expected = 0.40625 * 0.6;
        assert!((controller.state().driver_steering_angle - 0.3).abs() < 1.0e-5);
        assert!((controller.state().steering_angle - expected).abs() < 1.0e-5);
        assert!((controller.wheels[0].steering - expected).abs() < 1.0e-5);
    }

    #[test]
    fn disabling_steering_assist_restores_the_full_steering_range_at_speed() {
        let mut config = VehicleControllerConfig::default();
        config.steering.assist = true;
        config.steering.speed_sensitivity = 20.0;
        config.steering.minimum_speed_factor = 0.25;
        let max_angle = config.steering.max_angle;
        let mut controller =
            DynamicRayCastVehicleController::new(RigidBodyHandle::invalid(), config);
        controller.index_forward_axis = 2;
        controller.index_up_axis = 1;
        controller.current_vehicle_speed = 20.0;
        controller.set_input(VehicleInput {
            steering: 1.0,
            ..VehicleInput::default()
        });

        let chassis = RigidBodyBuilder::dynamic()
            .linvel(Vector::z() * 20.0)
            .build();
        controller.update_steering(&chassis, 1.0 / 60.0);
        assert!((controller.state().steering_angle - max_angle * 0.25).abs() < 1.0e-5);

        controller.set_minimum_speed_factor(0.7);
        controller.update_steering(&chassis, 1.0 / 60.0);
        assert!((controller.state().steering_angle - max_angle * 0.7).abs() < 1.0e-5);

        controller.set_minimum_speed_factor(2.0);
        assert_eq!(
            controller.powertrain.config.steering.minimum_speed_factor,
            1.0
        );
        controller.set_minimum_speed_factor(-1.0);
        assert_eq!(
            controller.powertrain.config.steering.minimum_speed_factor,
            0.0
        );
        controller.set_minimum_speed_factor(Real::NAN);
        assert_eq!(
            controller.powertrain.config.steering.minimum_speed_factor,
            0.0
        );

        controller.set_steering_assist(false);
        controller.update_steering(&chassis, 1.0 / 60.0);
        assert!((controller.state().steering_angle - max_angle).abs() < 1.0e-5);
    }

    #[test]
    fn counter_steer_assist_speed_activation_blends_from_five_to_ten_meters_per_second() {
        assert_eq!(counter_steer_assist_speed_activation(-10.0), 0.0);
        assert_eq!(counter_steer_assist_speed_activation(5.0), 0.0);
        assert!((counter_steer_assist_speed_activation(7.5) - 0.5).abs() < 1.0e-5);
        assert_eq!(counter_steer_assist_speed_activation(10.0), 1.0);
        assert_eq!(counter_steer_assist_speed_activation(20.0), 1.0);
    }

    #[test]
    fn counter_steer_assist_requires_steering_assist() {
        let mut config = VehicleControllerConfig::default();
        config.steering.assist = false;
        config.steering.counter_steer_assist = 1.0;
        let mut controller =
            DynamicRayCastVehicleController::new(RigidBodyHandle::invalid(), config);
        controller.index_forward_axis = 2;
        controller.index_up_axis = 1;
        controller.current_vehicle_speed = 10.0;
        controller.powertrain.state_mut().wheels_in_contact = 4;
        controller.set_input(VehicleInput {
            steering: 0.2,
            ..VehicleInput::default()
        });

        let chassis = RigidBodyBuilder::dynamic()
            .linvel(Vector::z() * 10.0 + Vector::x() * 4.0)
            .build();
        controller.update_steering(&chassis, 1.0 / 60.0);

        let expected = 0.2 * controller.powertrain.config.steering.max_angle;
        assert!((controller.state().steering_angle - expected).abs() < 1.0e-5);
        assert!(!controller.counter_steer_assist_active);
        assert_eq!(controller.counter_steer_assist_offset, 0.0);
    }

    #[test]
    fn steering_assist_does_not_step_when_crossing_the_minimum_speed() {
        let mut config = VehicleControllerConfig::default();
        config.steering.assist = true;
        let mut controller =
            DynamicRayCastVehicleController::new(RigidBodyHandle::invalid(), config);
        controller.index_forward_axis = 2;
        controller.index_up_axis = 1;
        controller.powertrain.state_mut().wheels_in_contact = 4;
        controller.set_input(VehicleInput {
            steering: 1.0,
            ..VehicleInput::default()
        });

        let minimum_speed_chassis = RigidBodyBuilder::dynamic()
            .linvel(Vector::z() * 5.0 + Vector::x() * 5.0)
            .build();
        controller.current_vehicle_speed = 5.0;
        controller.update_steering(&minimum_speed_chassis, 1.0 / 60.0);
        assert_eq!(controller.counter_steer_assist_offset, 0.0);

        let just_above_minimum_chassis = RigidBodyBuilder::dynamic()
            .linvel(Vector::z() * 5.001 + Vector::x() * 5.0)
            .build();
        controller.current_vehicle_speed = 5.001;
        controller.update_steering(&just_above_minimum_chassis, 1.0 / 60.0);

        assert!(controller.counter_steer_assist_offset.abs() > 0.0);
        assert!(controller.counter_steer_assist_offset.abs() < 1.0e-6);
    }

    #[test]
    fn steering_assist_stays_inactive_below_the_drift_threshold() {
        let mut config = VehicleControllerConfig::default();
        config.steering.assist = true;
        let mut controller =
            DynamicRayCastVehicleController::new(RigidBodyHandle::invalid(), config);
        controller.index_forward_axis = 2;
        controller.index_up_axis = 1;
        controller.current_vehicle_speed = 10.0;
        controller.powertrain.state_mut().wheels_in_contact = 4;
        controller.set_input(VehicleInput {
            steering: 0.2,
            ..VehicleInput::default()
        });

        let lateral_speed = 10.0 * (5.0 as Real).to_radians().tan();
        let chassis = RigidBodyBuilder::dynamic()
            .linvel(Vector::z() * 10.0 + Vector::x() * lateral_speed)
            .build();
        controller.update_steering(&chassis, 1.0 / 60.0);

        let steering = &controller.powertrain.config.steering;
        let normalized = 10.0 / steering.speed_sensitivity;
        let speed_factor = steering.minimum_speed_factor
            + (1.0 - normalized).powi(2) * (1.0 - steering.minimum_speed_factor);
        let expected = 0.2 * steering.max_angle * speed_factor;
        assert!((controller.state().steering_angle - expected).abs() < 1.0e-5);
        assert!(!controller.counter_steer_assist_active);
        assert_eq!(controller.counter_steer_assist_offset, 0.0);
    }

    #[test]
    fn steering_assist_smoothly_approaches_matching_correction() {
        let mut config = VehicleControllerConfig::default();
        config.steering.assist = true;
        let mut controller =
            DynamicRayCastVehicleController::new(RigidBodyHandle::invalid(), config);
        controller.index_forward_axis = 2;
        controller.index_up_axis = 1;
        controller.current_vehicle_speed = 10.0;
        controller.powertrain.state_mut().wheels_in_contact = 4;
        controller.set_input(VehicleInput {
            steering: 0.2,
            ..VehicleInput::default()
        });

        let chassis = RigidBodyBuilder::dynamic()
            .linvel(Vector::z() * 10.0 + Vector::x() * 4.0)
            .build();
        let steering = &controller.powertrain.config.steering;
        let normalized = 10.0 / steering.speed_sensitivity;
        let speed_factor = steering.minimum_speed_factor
            + (1.0 - normalized).powi(2) * (1.0 - steering.minimum_speed_factor);
        let player_angle = 0.2 * steering.max_angle * speed_factor;
        let velocity_dir = chassis.linvel().normalize();
        let drift_angle = Vector::y()
            .dot(&velocity_dir.cross(&Vector::z()))
            .atan2(velocity_dir.dot(&Vector::z()));
        let correction_angle = -drift_angle;

        controller.update_steering(&chassis, 1.0 / 60.0);

        assert!(controller.state().steering_angle > player_angle);
        assert!(controller.state().steering_angle < correction_angle);
        assert!(controller.counter_steer_assist_offset > 0.0);
        assert!(controller.counter_steer_assist_offset < correction_angle - player_angle);
    }

    #[test]
    fn counter_steer_assist_strength_blends_between_player_and_full_assistance() {
        fn controller(strength: Real) -> DynamicRayCastVehicleController {
            let mut config = VehicleControllerConfig::default();
            config.steering.assist = true;
            config.steering.counter_steer_assist = strength;
            config.steering.road_wheel_curve = 0.25;
            let mut controller =
                DynamicRayCastVehicleController::new(RigidBodyHandle::invalid(), config);
            controller.index_forward_axis = 2;
            controller.index_up_axis = 1;
            controller.current_vehicle_speed = 10.0;
            controller.powertrain.state_mut().wheels_in_contact = 4;
            controller.set_input(VehicleInput {
                steering: 0.2,
                ..VehicleInput::default()
            });
            controller
        }

        let chassis = RigidBodyBuilder::dynamic()
            .linvel(Vector::z() * 10.0 + Vector::x() * 4.0)
            .build();
        let mut full = controller(1.0);
        let mut half = controller(0.5);

        for _ in 0..180 {
            full.update_steering(&chassis, 1.0 / 60.0);
            half.update_steering(&chassis, 1.0 / 60.0);
        }

        let steering = &full.powertrain.config.steering;
        let normalized = 10.0 / steering.speed_sensitivity;
        let speed_factor = steering.minimum_speed_factor
            + (1.0 - normalized).powi(2) * (1.0 - steering.minimum_speed_factor);
        let player_angle = curved_steering_input(0.2, steering.road_wheel_curve)
            * steering.max_angle
            * speed_factor;
        let velocity_dir = chassis.linvel().normalize();
        let drift_angle = Vector::y()
            .dot(&velocity_dir.cross(&Vector::z()))
            .atan2(velocity_dir.dot(&Vector::z()));
        let correction_angle = -drift_angle;

        assert!((full.state().steering_angle - correction_angle).abs() < 1.0e-4);
        assert!(
            (full.state().driver_steering_angle - 0.2 * steering.max_angle * speed_factor).abs()
                < 1.0e-4
        );
        assert!(
            (half.state().steering_angle - (player_angle + correction_angle) * 0.5).abs() < 1.0e-4
        );
    }

    #[test]
    fn steering_assist_ignores_zero_or_opposite_user_input() {
        let mut config = VehicleControllerConfig::default();
        config.steering.assist = true;
        let mut controller =
            DynamicRayCastVehicleController::new(RigidBodyHandle::invalid(), config);
        controller.index_forward_axis = 2;
        controller.index_up_axis = 1;
        controller.current_vehicle_speed = 10.0;
        controller.powertrain.state_mut().wheels_in_contact = 4;

        let chassis = RigidBodyBuilder::dynamic()
            .linvel(Vector::z() * 10.0 + Vector::x() * 4.0)
            .build();
        controller.update_steering(&chassis, 1.0 / 60.0);
        assert!(controller.state().steering_angle.abs() <= Real::EPSILON);

        controller.set_input(VehicleInput {
            steering: -1.0,
            ..VehicleInput::default()
        });
        controller.update_steering(&chassis, 1.0 / 60.0);

        let steering = &controller.powertrain.config.steering;
        let normalized = 10.0 / steering.speed_sensitivity;
        let speed_factor = steering.minimum_speed_factor
            + (1.0 - normalized).powi(2) * (1.0 - steering.minimum_speed_factor);
        let expected = -steering.max_angle * speed_factor;
        assert!((controller.state().steering_angle - expected).abs() < 1.0e-5);
        assert_eq!(controller.counter_steer_assist_offset, 0.0);
    }

    #[test]
    fn opposite_input_releases_counter_steer_assist_immediately() {
        let mut config = VehicleControllerConfig::default();
        config.steering.assist = true;
        let mut controller =
            DynamicRayCastVehicleController::new(RigidBodyHandle::invalid(), config);
        controller.index_forward_axis = 2;
        controller.index_up_axis = 1;
        controller.current_vehicle_speed = 10.0;
        controller.powertrain.state_mut().wheels_in_contact = 4;
        controller.set_input(VehicleInput {
            steering: 0.2,
            ..VehicleInput::default()
        });

        let chassis = RigidBodyBuilder::dynamic()
            .linvel(Vector::z() * 10.0 + Vector::x() * 4.0)
            .build();
        for _ in 0..60 {
            controller.update_steering(&chassis, 1.0 / 60.0);
        }
        assert!(controller.counter_steer_assist_offset > 0.1);

        controller.set_input(VehicleInput {
            steering: -0.4,
            ..VehicleInput::default()
        });
        controller.update_steering(&chassis, 1.0 / 60.0);

        let steering = &controller.powertrain.config.steering;
        let normalized = 10.0 / steering.speed_sensitivity;
        let speed_factor = steering.minimum_speed_factor
            + (1.0 - normalized).powi(2) * (1.0 - steering.minimum_speed_factor);
        let expected = -0.4 * steering.max_angle * speed_factor;
        assert!((controller.state().steering_angle - expected).abs() < 1.0e-5);
        assert_eq!(controller.counter_steer_assist_offset, 0.0);
    }

    #[test]
    fn drift_end_smoothly_releases_the_assist_offset() {
        let mut config = VehicleControllerConfig::default();
        config.steering.assist = true;
        let mut controller =
            DynamicRayCastVehicleController::new(RigidBodyHandle::invalid(), config);
        controller.index_forward_axis = 2;
        controller.index_up_axis = 1;
        controller.current_vehicle_speed = 10.0;
        controller.powertrain.state_mut().wheels_in_contact = 4;
        controller.set_input(VehicleInput {
            steering: 0.2,
            ..VehicleInput::default()
        });

        let drifting = RigidBodyBuilder::dynamic()
            .linvel(Vector::z() * 10.0 + Vector::x() * 4.0)
            .build();
        for _ in 0..60 {
            controller.update_steering(&drifting, 1.0 / 60.0);
        }
        let assisted_angle = controller.state().steering_angle;

        let straight = RigidBodyBuilder::dynamic()
            .linvel(Vector::z() * 10.0)
            .build();
        controller.update_steering(&straight, 1.0 / 60.0);

        let steering = &controller.powertrain.config.steering;
        let normalized = 10.0 / steering.speed_sensitivity;
        let speed_factor = steering.minimum_speed_factor
            + (1.0 - normalized).powi(2) * (1.0 - steering.minimum_speed_factor);
        let player_angle = 0.2 * steering.max_angle * speed_factor;
        assert!(controller.state().steering_angle > player_angle);
        assert!(controller.state().steering_angle < assisted_angle);

        for _ in 0..60 {
            controller.update_steering(&straight, 1.0 / 60.0);
        }
        assert!((controller.state().steering_angle - player_angle).abs() < 1.0e-4);
    }

    #[test]
    fn released_input_smoothly_returns_assisted_steering_to_zero() {
        let mut config = VehicleControllerConfig::default();
        config.steering.assist = true;
        let mut controller =
            DynamicRayCastVehicleController::new(RigidBodyHandle::invalid(), config);
        controller.index_forward_axis = 2;
        controller.index_up_axis = 1;
        controller.current_vehicle_speed = 10.0;
        controller.powertrain.state_mut().wheels_in_contact = 4;
        controller.set_input(VehicleInput {
            steering: 0.2,
            ..VehicleInput::default()
        });

        let chassis = RigidBodyBuilder::dynamic()
            .linvel(Vector::z() * 10.0 + Vector::x() * 4.0)
            .build();
        for _ in 0..60 {
            controller.update_steering(&chassis, 1.0 / 60.0);
        }
        let assisted_angle = controller.state().steering_angle;

        controller.set_input(VehicleInput::default());
        controller.update_steering(&chassis, 1.0 / 60.0);
        assert!(controller.state().steering_angle > 0.0);
        assert!(controller.state().steering_angle < assisted_angle);

        for _ in 0..60 {
            controller.update_steering(&chassis, 1.0 / 60.0);
        }
        assert!(controller.state().steering_angle.abs() < 1.0e-4);
    }

    #[test]
    fn counter_steer_assist_setter_clamps_to_normalized_range() {
        let mut controller = DynamicRayCastVehicleController::new(
            RigidBodyHandle::invalid(),
            VehicleControllerConfig::default(),
        );

        controller.set_counter_steer_assist(2.0);
        assert_eq!(
            controller.powertrain.config.steering.counter_steer_assist,
            1.0
        );
        controller.set_counter_steer_assist(-1.0);
        assert_eq!(
            controller.powertrain.config.steering.counter_steer_assist,
            0.0
        );
    }

    #[test]
    fn reset_clears_controller_and_wheel_runtime_state_without_changing_tuning() {
        let mut controller = DynamicRayCastVehicleController::new(
            RigidBodyHandle::invalid(),
            VehicleControllerConfig::default(),
        );
        controller.add_wheel(
            Point::origin(),
            -Vector::y(),
            Vector::x(),
            0.4,
            0.35,
            &WheelTuning::default(),
            WheelRole::new(WheelAxle::Front, true, true),
        );
        controller.set_input(VehicleInput {
            throttle: 1.0,
            steering: 0.5,
            ..VehicleInput::default()
        });
        controller.powertrain.state_mut().engine_rpm = 5000.0;
        controller.powertrain.state_mut().current_gear = 3;
        controller.current_vehicle_speed = 25.0;
        controller.last_steering_compression = 1.0;
        controller.counter_steer_assist_active = true;
        controller.counter_steer_assist_offset = 0.3;
        controller.counter_steer_assist_direction = 1.0;
        controller.timer = 5.0;
        let wheel = &mut controller.wheels[0];
        wheel.rotation = 10.0;
        wheel.delta_rotation = 2.0;
        wheel.target_rotation = 3.0;
        wheel.angular_velocity = 20.0;
        wheel.drive_torque_transfer = 300.0;
        wheel.abs_release = 0.7;
        wheel.traction_control_cut = 0.6;
        wheel.traction_control = 0.35;
        wheel.engine_force = 100.0;
        wheel.brake = 0.5;
        wheel.steering = 0.4;
        wheel.skid_info = 0.2;
        wheel.sliding_grip = 0.85;
        wheel.ground_type = "asphalt".to_string();
        let suspension_rest_length = wheel.suspension_rest_length;

        controller.reset();

        assert_eq!(controller.input(), VehicleInput::default());
        assert_eq!(
            controller.state().engine_rpm,
            controller.powertrain.config.engine.idle_rpm
        );
        assert!(controller.state().engine_running);
        assert_eq!(controller.state().current_gear, 0);
        assert_eq!(controller.current_vehicle_speed, 0.0);
        assert!(!controller.counter_steer_assist_active);
        assert_eq!(controller.counter_steer_assist_offset, 0.0);
        assert_eq!(controller.timer, 0.0);
        let wheel = &controller.wheels[0];
        assert_eq!(wheel.rotation, 0.0);
        assert_eq!(wheel.delta_rotation, 0.0);
        assert_eq!(wheel.target_rotation, 0.0);
        assert_eq!(wheel.angular_velocity, 0.0);
        assert_eq!(wheel.drive_torque_transfer, 0.0);
        assert_eq!(wheel.traction_control, 0.35);
        assert_eq!(wheel.traction_control_cut, 0.0);
        assert_eq!(wheel.abs_release, 0.0);
        assert_eq!(wheel.engine_force, 0.0);
        assert_eq!(wheel.brake, 0.0);
        assert_eq!(wheel.steering, 0.0);
        assert_eq!(wheel.skid_info, 0.0);
        assert_eq!(wheel.sliding_grip, 1.0);
        assert!(wheel.ground_type.is_empty());
        assert_eq!(wheel.suspension_rest_length, suspension_rest_length);
    }
}

// struct WheelContactPoint<'a> {
//     body0: &'a RigidBody,
//     body1: Option<&'a RigidBody>,
//     friction_position_world: Point<Real>,
//     friction_direction_world: Vector<Real>,
//     jac_diag_ab_inv: Real,
//     max_impulse: Real,
// }

// impl<'a> WheelContactPoint<'a> {
//     pub fn new(
//         body0: &'a RigidBody,
//         body1: Option<&'a RigidBody>,
//         friction_position_world: Point<Real>,
//         friction_direction_world: Vector<Real>,
//         max_impulse: Real,
//     ) -> Self {
//         fn impulse_denominator(body: &RigidBody, pos: &Point<Real>, n: &Vector<Real>) -> Real {
//             let dpt = pos - body.center_of_mass();
//             let gcross = dpt.gcross(*n);
//             let v = (body.mprops.effective_world_inv_inertia_sqrt
//                 * (body.mprops.effective_world_inv_inertia_sqrt * gcross))
//                 .gcross(dpt);
//             // TODO: take the effective inv mass into account instead of the inv_mass?
//             body.mprops.local_mprops.inv_mass + n.dot(&v)
//         }
//         let denom0 =
//             impulse_denominator(body0, &friction_position_world, &friction_direction_world);
//         let denom1 = body1
//             .map(|body1| {
//                 impulse_denominator(body1, &friction_position_world, &friction_direction_world)
//             })
//             .unwrap_or(0.0);
//         let relaxation = 1.0;
//         let jac_diag_ab_inv = relaxation / (denom0 + denom1);

//         Self {
//             body0,
//             body1,
//             friction_position_world,
//             friction_direction_world,
//             jac_diag_ab_inv,
//             max_impulse,
//         }
//     }

//     pub fn calc_rolling_friction(&self, num_wheels_on_ground: usize) -> Real {
//         let contact_pos_world = self.friction_position_world;
//         let max_impulse = self.max_impulse;

//         let vel1 = self.body0.velocity_at_point(&contact_pos_world);
//         let vel2 = self
//             .body1
//             .map(|b| b.velocity_at_point(&contact_pos_world))
//             .unwrap_or_else(Vector::zeros);
//         let vel = vel1 - vel2;
//         let vrel = self.friction_direction_world.dot(&vel);

//         // calculate friction that moves us to zero relative velocity
//         (-vrel * self.jac_diag_ab_inv / (num_wheels_on_ground as Real))
//             .clamp(-max_impulse, max_impulse)
//     }
// }

fn relative_velocity_at_contact(
    bodies: &RigidBodySet,
    colliders: &ColliderSet,
    chassis: RigidBodyHandle,
    ground_object: Option<ColliderHandle>,
    point: &Point<Real>,
) -> Vector<Real> {
    let chassis_velocity = bodies[chassis].velocity_at_point(point);
    let ground_velocity = ground_object
        .and_then(|h| colliders[h].parent())
        .map(|h| &bodies[h])
        .map(|b| b.velocity_at_point(point))
        .unwrap_or_else(Vector::zeros);

    chassis_velocity - ground_velocity
}

fn tangent_impulse_response(
    body: &RigidBody,
    axes: [Vector<Real>; 2],
    angular_axes: [Vector<Real>; 2],
) -> [[Real; 2]; 2] {
    let angular = angular_axes.map(|axis| body.mprops.effective_world_inv_inertia_sqrt * axis);
    let mut response = [[0.0; 2]; 2];
    for i in 0..2 {
        for j in 0..2 {
            response[i][j] = axes[i].dot(&axes[j].component_mul(&body.mprops.effective_inv_mass))
                + angular[i].gdot(angular[j]);
        }
    }
    response
}

#[derive(Clone, Debug, PartialEq)]
/// Dynamic tire type with configurable friction coefficients for different surfaces
pub struct TireType {
    /// Name of the tire type
    pub name: String,
    /// Default friction coefficient
    pub default_friction: TireFriction,
    /// Map of surface material names to friction coefficients
    pub surface_friction: HashMap<String, TireFriction>,
}

impl TireType {
    /// Creates a new tire type with the given name and default friction
    pub fn new(name: &str, default_friction: TireFriction) -> Self {
        Self {
            name: name.to_string(),
            default_friction,
            surface_friction: HashMap::new(),
        }
    }

    /// Adds a surface material and its friction coefficient
    pub fn add_surface(&mut self, surface_name: &str, friction: TireFriction) {
        self.surface_friction
            .insert(surface_name.to_string(), friction);
    }

    /// Gets the friction coefficient for a given surface material
    pub fn get_friction(&self, surface_name: &str) -> TireFriction {
        self.surface_friction
            .get(surface_name)
            .copied()
            .unwrap_or(self.default_friction)
    }

    /// Removes a surface material
    pub fn remove_surface(&mut self, surface_name: &str) -> Option<TireFriction> {
        self.surface_friction.remove(surface_name)
    }

    /// Lists all configured surface materials
    pub fn list_surfaces(&self) -> Vec<&String> {
        self.surface_friction.keys().collect()
    }
}
