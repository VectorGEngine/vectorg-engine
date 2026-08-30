use crate::dynamics::{RigidBody, RigidBodyHandle, RigidBodySet};
use crate::geometry::{ColliderHandle, ColliderSet, Ray};
use crate::math::{Point, Real, Rotation, Vector, DIM};
use crate::pipeline::{QueryFilter, QueryPipeline};
use crate::utils::{SimdCross, SimdDot};
use std::collections::HashMap;

use super::vehicle_powertrain::{
    VehicleControllerConfig, VehicleEngineState, VehicleInput, VehiclePowertrain,
    VehicleShiftOutcome, VehicleState, WheelAxle, WheelRole,
};

const DRIFT_ASSIST_MIN_SPEED: Real = 5.0;
const DRIFT_ASSIST_FULL_SPEED: Real = 10.0;
const DRIFT_ASSIST_MIN_CONTACTS: usize = 2;
const DRIFT_ASSIST_ENTER_ANGLE: Real = 0.104_719_76; // 6 degrees.
const DRIFT_ASSIST_EXIT_ANGLE: Real = 0.052_359_88; // 3 degrees.
const DRIFT_ASSIST_FULL_ANGLE: Real = 0.349_065_84; // 20 degrees.
const DRIFT_ASSIST_RESPONSE: Real = 8.0;
const DRIFT_ASSIST_RELEASE_RESPONSE: Real = 15.0;
const DRIFT_ASSIST_YAW_DAMPING: Real = 0.08;
const DRIFT_ASSIST_INPUT_DEADZONE: Real = 0.01;
const POWERED_SLIP_MIN_SURFACE_SPEED_GAP: Real = 0.5;
const POWERED_SLIP_MIN_ROAD_ACCELERATION: Real = 0.5;
const POWERED_SLIP_RESISTANCE_MAX_ROAD_SPEED: Real = 1.0;
const POWERED_SLIP_RESISTANCE_DELAY: Real = 0.18;
const POWERED_SLIP_ENGAGE_RESPONSE: Real = 14.0;
const POWERED_SLIP_RELEASE_RESPONSE: Real = 3.0;
const POWERED_SLIP_EFFECTIVE_INERTIA: Real = 12.0;
const POWERED_SLIP_ROTATIONAL_DAMPING: Real = 0.35;
const WHEEL_ROTATION_RETENTION: Real = 0.99;
const VISUAL_WHEEL_REGRIP_RESPONSE: Real = 24.0;
const VISUAL_WHEEL_BRAKE_RESPONSE: Real = 30.0;
const VISUAL_WHEEL_SYNC_TOLERANCE: Real = 0.05;
const TRACTION_CONTROL_MIN_BODY_SPEED: Real = 0.75;
const TRACTION_CONTROL_LOW_SPEED_GAP_START: Real = 0.5;
const TRACTION_CONTROL_LOW_SPEED_GAP_FULL: Real = 3.0;
const TRACTION_CONTROL_SLIP_START: Real = 0.12;
const TRACTION_CONTROL_SLIP_FULL: Real = 0.55;
const TRACTION_CONTROL_SPEED_OFF: Real = 2.0;
const TRACTION_CONTROL_SPEED_FULL: Real = 8.0;
const TRACTION_CONTROL_LOW_SPEED_OVERRIDE_FULL: Real = 0.0;
const TRACTION_CONTROL_LOW_SPEED_OVERRIDE_OFF: Real = 10.0;
const TRACTION_CONTROL_STEERING_OVERRIDE_START: Real = 0.2;
const TRACTION_CONTROL_STEERING_OVERRIDE_FULL: Real = 0.8;
const TRACTION_CONTROL_ENGAGE_RESPONSE: Real = 18.0;
const TRACTION_CONTROL_RELEASE_RESPONSE: Real = 8.0;
const LONGITUDINAL_SLIP_REFERENCE_SPEED: Real = 10.0;
const LONGITUDINAL_SLIP_START: Real = 0.15;
const LONGITUDINAL_SLIP_FULL: Real = 0.30;
const LONGITUDINAL_SLIP_FORWARD_GRIP_MIN: Real = 0.75;
const DYNAMIC_FRICTION_RATIO: Real = 0.85;
// Equivalent to the legacy static clamp producing skid_info below approximately 0.3.
const DYNAMIC_FRICTION_ENTER_UTILIZATION_SQUARED: Real = 11.111_111;
const CONTACT_DAMPING_SPEED_START: Real = 16.666_667; // 60 km/h.
const CONTACT_DAMPING_FRONT_MAX: Real = 0.3;
const CONTACT_DAMPING_REAR_MAX: Real = 0.4;
const ESC_SIDESLIP_YAW_GAIN: Real = 2.0;
const TAU: Real = 6.283_185_307_179_586 as Real;

fn drift_assist_speed_activation(forward_speed: Real) -> Real {
    let normalized = ((forward_speed - DRIFT_ASSIST_MIN_SPEED)
        / (DRIFT_ASSIST_FULL_SPEED - DRIFT_ASSIST_MIN_SPEED))
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
    drift_assist_active: bool,
    drift_assist_offset: Real,
    drift_assist_direction: Real,

    timer: Real,
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
    /// The multiplier of friction between a tire and the collider it's on top of.
    pub side_friction_stiffness: Real,
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
            side_friction_stiffness: 1.0,
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
    /// The multiplier of friction between a tire and the collider it's on top of.
    pub side_friction_stiffness: Real,
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
    /// The multiplier of friction between a tire and the collider it's on top of.
    pub side_friction_stiffness: Real,
    /// The wheel’s current rotation on its axle.
    pub rotation: Real,
    /// The change in rotation since the last update.
    pub delta_rotation: Real,
    /// The target angular velocity of the wheel.
    pub target_rotation: Real,
    powered_angular_velocity: Real,
    powered_slip: Real,
    powered_slip_timer: Real,
    powered_spin_requested: bool,
    visual_angular_velocity: Real,
    visual_regrip_active: bool,
    previous_rolling_surface_speed: Real,
    wheel_coupling_torque: Real,
    wheel_limit_velocity: Real,
    drive_throttle: Real,
    drivetrain_connected: bool,
    drive_slip_demand: Real,
    traction_control_cut: Real,
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
    /// The maximum amount of braking impulse applied to slow down the vehicle.
    pub max_brake_force: Real,
    /// The anti-lock braking system strength applied to this wheel.
    pub anti_lock_brake: Real,
    /// traction control system force applied to this wheel.
    pub is_anti_lock_brake: bool,
    /// traction control system force applied to this wheel.
    pub traction_control: Real,
    /// The impulse applied from tire to engine
    pub engine_force_feedback: Real,
    /// The side factor for the wheel, used to calculate the side impulse.
    pub side_factor: Real,
    /// The forward factor for the wheel, used to calculate the forward impulse.
    pub fwd_factor: Real,
    /// The brake factor for the wheel, used to calculate the brake impulse.
    pub brake_factor: Real,
    /// The damping applied to the contact point of the wheel.
    pub contact_damping: Real,
    /// The configured contact damping before runtime slip adjustment.
    pub base_contact_damping: Real,
    lock: bool,

    clipped_inv_contact_dot_suspension: Real,
    suspension_relative_velocity: Real,
    contact_forward_speed: Real,
    contact_side_speed: Real,
    /// The force applied by the suspension.
    pub wheel_suspension_force: Real,
    /// The amount of skid information for this wheel.
    pub skid_info: Real,
    last_skid_info: Real,
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
            powered_angular_velocity: 0.0,
            powered_slip: 0.0,
            powered_slip_timer: 0.0,
            powered_spin_requested: false,
            visual_angular_velocity: 0.0,
            visual_regrip_active: false,
            previous_rolling_surface_speed: 0.0,
            wheel_coupling_torque: 0.0,
            wheel_limit_velocity: 0.0,
            drive_throttle: 0.0,
            drivetrain_connected: false,
            drive_slip_demand: 0.0,
            traction_control_cut: 0.0,
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
            side_impulse: 0.0,
            brake_impulse: 0.0,
            forward_impulse: 0.0,
            side_friction_stiffness: info.side_friction_stiffness,
            lock: false,
            tire_type: info.tire_type,
            suspension_compression_rate: 0.0,
            ground_friction: 1.0,
            ground_type: String::new(),
            side_factor: 1.0,
            fwd_factor: 1.0,
            brake_factor: 1.0,
            contact_damping: 0.2, // This is a default value, can be adjusted later
            base_contact_damping: 0.2,
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
        self.visual_angular_velocity = 0.0;
        self.visual_regrip_active = false;
        self.target_rotation = 0.0;
        self.clear_powered_spin_state();
        self.previous_rolling_surface_speed = 0.0;
        self.wheel_coupling_torque = 0.0;
        self.wheel_limit_velocity = 0.0;
        self.drive_throttle = 0.0;
        self.drivetrain_connected = false;
        self.drive_slip_demand = 0.0;
        self.traction_control_cut = 0.0;
        self.forward_impulse = 0.0;
        self.side_impulse = 0.0;
        self.brake_impulse = 0.0;
        self.steering = 0.0;
        self.engine_force = 0.0;
        self.brake = 0.0;
        self.anti_lock_brake = 0.0;
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
        self.ground_friction = 1.0;
        self.ground_type.clear();
        self.suspension_compression_rate = 0.0;
        self.contact_damping = self.base_contact_damping;
    }

    fn clear_powered_spin_state(&mut self) {
        self.powered_angular_velocity = 0.0;
        self.powered_slip = 0.0;
        self.powered_slip_timer = 0.0;
        self.powered_spin_requested = false;
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
    side_dir: Vector<Real>,
    forward_speed: Real,
    side_speed: Real,
    friction_limit: Real,
}

impl Default for WheelContactState {
    fn default() -> Self {
        Self {
            is_grounded: false,
            ground_object: None,
            forward_dir: Vector::zeros(),
            side_dir: Vector::zeros(),
            forward_speed: 0.0,
            side_speed: 0.0,
            friction_limit: 0.0,
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

fn update_powered_wheel_rotation(
    wheel: &mut Wheel,
    rolling_angular_velocity: Real,
    dt: Real,
) -> Real {
    let dt = dt.max(Real::EPSILON);
    let radius = wheel.radius.max(Real::EPSILON);
    let direction = wheel.target_rotation.signum();
    let rolling_surface_speed = rolling_angular_velocity * radius;
    let speed_gap = (wheel.target_rotation * radius - rolling_surface_speed) * direction;
    let road_acceleration =
        ((rolling_surface_speed - wheel.previous_rolling_surface_speed) / dt) * direction;
    let powered = wheel.role.driven
        && wheel.drivetrain_connected
        && wheel.drive_throttle > 0.1
        && direction != 0.0;
    let opposing_motion = rolling_angular_velocity * wheel.target_rotation < 0.0;
    let resisted = powered
        && wheel.raycast_info.is_in_contact
        && rolling_surface_speed.abs() < POWERED_SLIP_RESISTANCE_MAX_ROAD_SPEED
        && speed_gap > POWERED_SLIP_MIN_SURFACE_SPEED_GAP
        && road_acceleration < POWERED_SLIP_MIN_ROAD_ACCELERATION;
    let airborne = powered && !wheel.raycast_info.is_in_contact;

    if resisted {
        wheel.powered_slip_timer =
            (wheel.powered_slip_timer + dt).min(POWERED_SLIP_RESISTANCE_DELAY);
    } else {
        wheel.powered_slip_timer = (wheel.powered_slip_timer - dt * 2.0).max(0.0);
    }

    let resistance_confirmed =
        resisted && wheel.powered_slip_timer >= POWERED_SLIP_RESISTANCE_DELAY;
    let unrestricted_slip = opposing_motion || airborne || resistance_confirmed;
    let requested_slip = if unrestricted_slip {
        1.0
    } else {
        wheel.drive_slip_demand.clamp(0.0, 1.0)
    };
    let spin_requested = powered && requested_slip > Real::EPSILON;
    wheel.powered_spin_requested = spin_requested;
    let response = if spin_requested {
        POWERED_SLIP_ENGAGE_RESPONSE
    } else {
        POWERED_SLIP_RELEASE_RESPONSE
    };
    let target_slip = if spin_requested { requested_slip } else { 0.0 };
    let blend = 1.0 - (-response * dt).exp();
    let previous_slip = wheel.powered_slip;
    wheel.powered_slip += (target_slip - wheel.powered_slip) * blend;

    if previous_slip <= Real::EPSILON && wheel.powered_slip > Real::EPSILON {
        wheel.powered_angular_velocity = rolling_angular_velocity;
    }

    if wheel.powered_slip > Real::EPSILON {
        let inertia = POWERED_SLIP_EFFECTIVE_INERTIA * (radius / 0.35).powi(2);
        wheel.powered_angular_velocity += wheel.wheel_coupling_torque / inertia * dt;
        wheel.powered_angular_velocity *= (-POWERED_SLIP_ROTATIONAL_DAMPING * dt).exp();

        let limit = wheel.wheel_limit_velocity.abs();
        if limit > Real::EPSILON {
            let directed_velocity = (wheel.powered_angular_velocity * direction).clamp(0.0, limit);
            wheel.powered_angular_velocity = directed_velocity * direction;
        }
    } else {
        wheel.powered_angular_velocity = rolling_angular_velocity;
    }

    if wheel.lock {
        wheel.clear_powered_spin_state();
    }

    wheel.previous_rolling_surface_speed = rolling_surface_speed;
    rolling_angular_velocity
        + (wheel.powered_angular_velocity - rolling_angular_velocity) * wheel.powered_slip
}

fn update_wheel_rotation(wheel: &mut Wheel, rolling_angular_velocity: Real, dt: Real) {
    let dt = dt.max(Real::EPSILON);
    let was_powered_spin_requested = wheel.powered_spin_requested;
    let is_grounded = wheel.raycast_info.is_in_contact;

    let physical_angular_velocity = if wheel.lock {
        wheel.clear_powered_spin_state();
        0.0
    } else if is_grounded {
        update_powered_wheel_rotation(wheel, rolling_angular_velocity, dt)
    } else {
        let previous_angular_velocity = wheel.delta_rotation / dt;
        update_powered_wheel_rotation(wheel, previous_angular_velocity, dt)
            * (1.0 - wheel.brake.clamp(0.0, 1.0))
    };

    if was_powered_spin_requested && !wheel.powered_spin_requested && is_grounded && !wheel.lock {
        wheel.visual_regrip_active = true;
    }

    if wheel.lock {
        wheel.visual_regrip_active = false;
        let blend = 1.0 - (-VISUAL_WHEEL_BRAKE_RESPONSE * dt).exp();
        wheel.visual_angular_velocity += (0.0 - wheel.visual_angular_velocity) * blend;
        if wheel.visual_angular_velocity.abs() <= VISUAL_WHEEL_SYNC_TOLERANCE {
            wheel.visual_angular_velocity = 0.0;
        }
    } else if is_grounded && wheel.powered_spin_requested {
        wheel.visual_regrip_active = false;
        wheel.visual_angular_velocity = physical_angular_velocity;
    } else if is_grounded && wheel.visual_regrip_active {
        let blend = 1.0 - (-VISUAL_WHEEL_REGRIP_RESPONSE * dt).exp();
        wheel.visual_angular_velocity +=
            (rolling_angular_velocity - wheel.visual_angular_velocity) * blend;
        if (wheel.visual_angular_velocity - rolling_angular_velocity).abs()
            <= VISUAL_WHEEL_SYNC_TOLERANCE
        {
            wheel.visual_angular_velocity = rolling_angular_velocity;
            wheel.visual_regrip_active = false;
        }
    } else if is_grounded {
        wheel.visual_angular_velocity = rolling_angular_velocity;
    } else if wheel.brake > Real::EPSILON {
        let blend = 1.0 - (-VISUAL_WHEEL_BRAKE_RESPONSE * dt).exp();
        wheel.visual_angular_velocity +=
            (physical_angular_velocity - wheel.visual_angular_velocity) * blend;
    } else {
        wheel.visual_angular_velocity = physical_angular_velocity;
    }

    wheel.rotation += wheel.visual_angular_velocity * dt;
    wheel.delta_rotation = physical_angular_velocity * dt * WHEEL_ROTATION_RETENTION;
}

fn smoothstep(edge0: Real, edge1: Real, value: Real) -> Real {
    let rate = ((value - edge0) / (edge1 - edge0).max(Real::EPSILON)).clamp(0.0, 1.0);
    rate * rate * (3.0 - 2.0 * rate)
}

fn theoretical_max_speed(config: &VehicleControllerConfig, wheel_radius: Real) -> Real {
    let highest_gear_ratio = config
        .transmission
        .forward_ratios
        .last()
        .copied()
        .unwrap_or(1.0)
        .abs()
        .max(0.01);

    config.engine.max_rpm
        / (highest_gear_ratio * config.transmission.final_drive_ratio.max(0.01))
        / 60.0
        * TAU
        * wheel_radius.max(0.01)
}

fn speed_adjusted_contact_damping(
    base: Real,
    axle: WheelAxle,
    body_speed: Real,
    maximum_speed: Real,
) -> Real {
    let speed_factor = smoothstep(
        CONTACT_DAMPING_SPEED_START,
        maximum_speed.max(CONTACT_DAMPING_SPEED_START + Real::EPSILON),
        body_speed.abs(),
    );
    let maximum = match axle {
        WheelAxle::Front => CONTACT_DAMPING_FRONT_MAX,
        WheelAxle::Rear => CONTACT_DAMPING_REAR_MAX,
    }
    .max(base);

    base + (maximum - base) * speed_factor
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

fn tire_slip(wheel_surface_speed: Real, forward_speed: Real, side_speed: Real) -> (Real, Real) {
    let reference_speed = forward_speed.abs().max(2.0);
    let longitudinal_slip = (wheel_surface_speed - forward_speed).abs() / reference_speed;
    let lateral_slip_angle = side_speed.abs().atan2(reference_speed);
    (longitudinal_slip, lateral_slip_angle)
}

fn tire_grip_ratio(
    impulse_utilization_squared: Real,
    longitudinal_slip: Real,
    lateral_slip_angle: Real,
) -> Real {
    if impulse_utilization_squared > DYNAMIC_FRICTION_ENTER_UTILIZATION_SQUARED
        && (longitudinal_slip >= 0.12 || lateral_slip_angle >= (5.0 as Real).to_radians())
    {
        DYNAMIC_FRICTION_RATIO
    } else {
        1.0
    }
}

fn traction_control_speed_factor(body_speed: Real) -> Real {
    smoothstep(
        TRACTION_CONTROL_SPEED_OFF,
        TRACTION_CONTROL_SPEED_FULL,
        body_speed.abs(),
    )
}

fn traction_control_steering_factor(steering_input: Real) -> Real {
    smoothstep(
        TRACTION_CONTROL_STEERING_OVERRIDE_START,
        TRACTION_CONTROL_STEERING_OVERRIDE_FULL,
        steering_input.abs(),
    )
}

fn traction_control_low_speed_factor(body_speed: Real) -> Real {
    smoothstep(
        TRACTION_CONTROL_LOW_SPEED_OVERRIDE_FULL,
        TRACTION_CONTROL_LOW_SPEED_OVERRIDE_OFF,
        body_speed.abs(),
    )
}

fn effective_traction_control_strength(
    assigned_strength: Real,
    body_speed: Real,
    steering_input: Real,
) -> Real {
    let speed_recovery = traction_control_speed_factor(body_speed);
    let steering_override = traction_control_steering_factor(steering_input);
    let low_speed_factor = traction_control_low_speed_factor(body_speed);
    assigned_strength.clamp(0.0, 1.0)
        * low_speed_factor
        * (1.0 - steering_override * (1.0 - speed_recovery))
}

fn planar_speed(linear_velocity: &Vector<Real>, up: &Vector<Real>) -> Real {
    let planar_velocity = linear_velocity - up * linear_velocity.dot(up);
    planar_velocity.norm()
}

fn drive_capacity_excess(drive_demand: Real, forward_capacity: Real) -> Real {
    if drive_demand <= Real::EPSILON {
        0.0
    } else {
        ((drive_demand - forward_capacity.max(0.0)) / drive_demand).clamp(0.0, 1.0)
    }
}

fn residual_drive_slip_demand(
    controlled_drive_demand: Real,
    forward_capacity: Real,
    raw_drive_demand: Real,
) -> Real {
    let residual_excess = (controlled_drive_demand - forward_capacity).max(0.0);
    let capacity_scale = forward_capacity.max(raw_drive_demand * 0.1);
    (residual_excess / capacity_scale.max(Real::EPSILON)).clamp(0.0, 1.0)
}

fn traction_controlled_drive_slip_demand(
    raw_drive_demand: Real,
    forward_capacity: Real,
    traction_control_strength: Real,
) -> Real {
    let uncontrolled_slip =
        residual_drive_slip_demand(raw_drive_demand, forward_capacity, raw_drive_demand);
    uncontrolled_slip * (1.0 - traction_control_strength.clamp(0.0, 1.0))
}

fn contact_rolling_angular_velocity(
    contact_velocity: &Vector<Real>,
    wheel_forward: &Vector<Real>,
    wheel_radius: Real,
) -> Real {
    wheel_forward.dot(contact_velocity) / wheel_radius.max(Real::EPSILON)
}

fn driven_wheel_longitudinal_slip(
    wheel_surface_speed: Real,
    ground_speed: Real,
    drive_direction: Real,
) -> Real {
    if drive_direction == 0.0 {
        return 0.0;
    }

    let directed_wheel_speed = wheel_surface_speed * drive_direction;
    let directed_ground_speed = ground_speed * drive_direction;
    let speed_gap = directed_wheel_speed - directed_ground_speed;

    if speed_gap <= 0.0 {
        return 0.0;
    }

    let reference_speed = directed_ground_speed
        .abs()
        .max(LONGITUDINAL_SLIP_REFERENCE_SPEED);
    smoothstep(
        LONGITUDINAL_SLIP_START,
        LONGITUDINAL_SLIP_FULL,
        speed_gap / reference_speed,
    )
}

fn longitudinal_slip_grip_scales(longitudinal_slip: Real) -> (Real, Real) {
    let activation = longitudinal_slip.clamp(0.0, 1.0);
    let loss = activation * activation;
    (1.0, 1.0 + (LONGITUDINAL_SLIP_FORWARD_GRIP_MIN - 1.0) * loss)
}

fn traction_control_target(
    strength: Real,
    wheel_surface_speed: Real,
    body_forward_speed: Real,
    drive_direction: Real,
    capacity_excess: Real,
    bypass: bool,
) -> Real {
    let strength = strength.clamp(0.0, 1.0);
    if strength <= Real::EPSILON || drive_direction == 0.0 || bypass {
        return 0.0;
    }

    let directed_body_speed = body_forward_speed * drive_direction;
    if directed_body_speed < -TRACTION_CONTROL_MIN_BODY_SPEED {
        return 0.0;
    }

    let directed_wheel_speed = wheel_surface_speed * drive_direction;
    let powered_overspeed = (directed_wheel_speed - directed_body_speed).max(0.0);
    let low_speed = directed_body_speed <= TRACTION_CONTROL_MIN_BODY_SPEED;
    let slip_intervention = if low_speed {
        ((powered_overspeed - TRACTION_CONTROL_LOW_SPEED_GAP_START)
            / (TRACTION_CONTROL_LOW_SPEED_GAP_FULL - TRACTION_CONTROL_LOW_SPEED_GAP_START))
            .clamp(0.0, 1.0)
    } else {
        let slip_ratio = powered_overspeed / directed_body_speed;
        ((slip_ratio - TRACTION_CONTROL_SLIP_START)
            / (TRACTION_CONTROL_SLIP_FULL - TRACTION_CONTROL_SLIP_START))
            .clamp(0.0, 1.0)
    };

    strength * slip_intervention.max(capacity_excess.clamp(0.0, 1.0))
}

impl DynamicRayCastVehicleController {
    /// Creates a new vehicle represented by the given rigid-body.
    ///
    /// Wheels have to be attached afterwards calling [`Self::add_wheel`].
    pub fn new(chassis: RigidBodyHandle, config: VehicleControllerConfig) -> Self {
        let mut tire_types = HashMap::new();

        // Create default tire types
        tire_types.insert("default".to_string(), TireType::new("default", 1.0));

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
            drift_assist_active: false,
            drift_assist_offset: 0.0,
            drift_assist_direction: 0.0,
            timer: 0.0,
        }
    }

    /// Sets the normalized driver inputs consumed by the next vehicle update.
    pub fn set_input(&mut self, input: VehicleInput) {
        self.powertrain.set_input(input);
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
        self.drift_assist_active = false;
        self.drift_assist_offset = 0.0;
        self.drift_assist_direction = 0.0;
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
            self.drift_assist_active = false;
            self.drift_assist_offset = 0.0;
            self.drift_assist_direction = 0.0;
        }
    }

    /// Sets drift-correction strength (`0.0` = none, `1.0` = full correction).
    pub fn set_drift_correction(&mut self, correction: Real) {
        self.powertrain.config.steering.drift_correction = correction.clamp(0.0, 1.0);

        if self.powertrain.config.steering.drift_correction <= Real::EPSILON {
            self.drift_assist_active = false;
            self.drift_assist_offset = 0.0;
            self.drift_assist_direction = 0.0;
        }
    }

    /// Adds a new tire type to the controller
    pub fn add_tire_type(&mut self, tire_type: &str, friction: Real) {
        self.tire_types
            .insert(tire_type.to_string(), TireType::new(tire_type, friction));
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
        friction: Real,
    ) {
        if let Some(tire_type) = self.tire_types.get_mut(tire_type_name) {
            tire_type.add_surface(surface_name, friction);
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
            side_friction_stiffness: tuning.side_friction_stiffness,
            tire_type: tuning.tire_type.clone(),
            role,
        };

        let wheel_id = self.wheels.len();
        let mut wheel = Wheel::new(ci);
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

    fn driven_wheel_speed_and_radius(&self, dt: Real) -> (Real, Real) {
        let mut driven_speed: Real = 0.0;
        let mut radius_sum: Real = 0.0;
        let mut driven_count: usize = 0;
        let dt = dt.max(Real::EPSILON);

        for wheel in &self.wheels {
            if wheel.role.driven {
                let speed = wheel.delta_rotation * wheel.radius
                    / (dt * WHEEL_ROTATION_RETENTION.max(Real::EPSILON));
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
        let correction_strength = steering_config.drift_correction.clamp(0.0, 1.0);
        let assist_speed_activation = drift_assist_speed_activation(self.current_vehicle_speed);
        let mut target_assist_offset = None;
        let mut cancel_immediately = false;
        let grounded = self.powertrain.state().wheels_in_contact >= DRIFT_ASSIST_MIN_CONTACTS;
        let can_assist = assist_enabled
            && correction_strength > Real::EPSILON
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
            self.drift_assist_active = if self.drift_assist_active {
                absolute_angle > DRIFT_ASSIST_EXIT_ANGLE
            } else {
                absolute_angle > DRIFT_ASSIST_ENTER_ANGLE
            };

            if self.drift_assist_active {
                let yaw_rate = self.chassis_yaw_rate(chassis);
                let correction_angle =
                    (-angle - yaw_rate * DRIFT_ASSIST_YAW_DAMPING).clamp(-max_angle, max_angle);
                self.drift_assist_direction = correction_angle.signum();
                let matching_input = input.steering.abs() > DRIFT_ASSIST_INPUT_DEADZONE
                    && input.steering * correction_angle > 0.0;

                if matching_input {
                    let normalized_angle = ((absolute_angle - DRIFT_ASSIST_ENTER_ANGLE)
                        / (DRIFT_ASSIST_FULL_ANGLE - DRIFT_ASSIST_ENTER_ANGLE))
                        .clamp(0.0, 1.0);
                    let activation = normalized_angle
                        * normalized_angle
                        * (3.0 - 2.0 * normalized_angle)
                        * assist_speed_activation;
                    target_assist_offset =
                        Some((correction_angle - player_angle) * correction_strength * activation);
                } else if input.steering.abs() > DRIFT_ASSIST_INPUT_DEADZONE {
                    cancel_immediately = true;
                }
            }
        } else {
            self.drift_assist_active = false;
        }

        if input.steering.abs() > DRIFT_ASSIST_INPUT_DEADZONE
            && self.drift_assist_direction != 0.0
            && input.steering * self.drift_assist_direction < 0.0
        {
            cancel_immediately = true;
        }

        if cancel_immediately {
            self.drift_assist_offset = 0.0;
            self.drift_assist_direction = 0.0;
        } else {
            let target_offset = target_assist_offset.unwrap_or(0.0);
            let response_rate = if target_offset.abs() < self.drift_assist_offset.abs() {
                DRIFT_ASSIST_RELEASE_RESPONSE
            } else {
                DRIFT_ASSIST_RESPONSE
            };
            let response = 1.0 - (-response_rate * dt.max(0.0)).exp();
            self.drift_assist_offset += (target_offset - self.drift_assist_offset) * response;

            if target_assist_offset.is_none() && self.drift_assist_offset.abs() <= 1.0e-4 {
                self.drift_assist_offset = 0.0;
                self.drift_assist_direction = 0.0;
            }
        }

        let mut center_angle = player_angle + self.drift_assist_offset;

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
                wheel.wheel_limit_velocity = output.wheel_limit_velocity;
                wheel.drive_throttle = output.drive_throttle;
                wheel.drivetrain_connected = output.drivetrain_connected;
            } else {
                wheel.engine_force = 0.0;
                wheel.target_rotation = 0.0;
                wheel.wheel_coupling_torque = 0.0;
                wheel.wheel_limit_velocity = 0.0;
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
            wheel.anti_lock_brake =
                if input.handbrake > service_brake && wheel.role.axle == WheelAxle::Rear {
                    0.0
                } else {
                    dynamics.abs_strength
                };
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
                .map(|wheel| wheel.traction_control_cut)
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

    /// Updates the vehicle’s velocity based on its suspension, engine force, and brake.
    #[profiling::function]
    pub fn update_vehicle(
        &mut self,
        dt: Real,
        bodies: &mut RigidBodySet,
        colliders: &ColliderSet,
        queries: &QueryPipeline,
        filter: QueryFilter,
    ) {
        self.timer += dt;
        let num_wheels = self.wheels.len();
        let chassis = &bodies[self.chassis];

        let forward_w = chassis.position() * Vector::ith(self.index_forward_axis, 1.0);
        self.current_vehicle_speed = forward_w.dot(chassis.linvel());
        let (driven_wheel_speed, driven_wheel_radius) = self.driven_wheel_speed_and_radius(dt);
        let output = self.powertrain.update(
            dt,
            self.current_vehicle_speed,
            driven_wheel_speed,
            driven_wheel_radius,
        );
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

        for wheel in &mut self.wheels {
            if wheel.engine_force.abs() > 0.0 {
                chassis.wake_up(true);
            }

            // apply suspension force
            let mut suspension_force = wheel.wheel_suspension_force;

            if suspension_force > wheel.max_suspension_force {
                suspension_force = wheel.max_suspension_force;
            }

            let impulse = wheel.raycast_info.contact_normal_ws * suspension_force * dt;
            chassis.apply_impulse_at_point(impulse, wheel.raycast_info.contact_point_ws, false);
        }

        self.update_friction(bodies, colliders, dt);

        let rolling_angular_velocities: Vec<Real> = self
            .wheels
            .iter()
            .enumerate()
            .map(|(wheel_id, wheel)| {
                let Some(ground_object) = wheel.raycast_info.ground_object else {
                    return wheel.delta_rotation / dt.max(Real::EPSILON);
                };
                let contact_velocity = relative_velocity_at_contact(
                    bodies,
                    colliders,
                    self.chassis,
                    Some(ground_object),
                    &wheel.raycast_info.contact_point_ws,
                );
                contact_rolling_angular_velocity(
                    &contact_velocity,
                    &self.forward_ws[wheel_id],
                    wheel.radius,
                )
            })
            .collect();

        for (wheel_id, wheel) in self.wheels.iter_mut().enumerate() {
            update_wheel_rotation(wheel, rolling_angular_velocities[wheel_id], dt);
        }
        let chassis = &bodies[self.chassis];
        self.update_output_state(chassis);
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

    #[profiling::function]
    fn update_friction(&mut self, bodies: &mut RigidBodySet, colliders: &ColliderSet, dt: Real) {
        let num_wheels = self.wheels.len();
        let steering_input = self.powertrain.input().steering;
        self.powertrain.state_mut().esc_activity = 0.0;
        if num_wheels == 0 {
            return;
        }

        self.forward_ws.resize(num_wheels, Default::default());
        self.axle.resize(num_wheels, Default::default());
        let mut contacts = vec![WheelContactState::default(); num_wheels];

        let (esc_intervention, esc_side_axis, chassis_forward, body_speed) = {
            let chassis = &bodies[self.chassis];
            let intervention = self.esc_intervention(chassis);
            let rotation = chassis.position().rotation;
            let chassis_forward = rotation * Vector::ith(self.index_forward_axis, 1.0);
            let chassis_up = rotation * Vector::ith(self.index_up_axis, 1.0);
            (
                intervention,
                self.side_axis(),
                chassis_forward,
                planar_speed(chassis.linvel(), &chassis_up),
            )
        };
        self.powertrain.state_mut().esc_activity = esc_intervention.activity;

        for wheel in &mut self.wheels {
            wheel.brake_impulse = 0.0;
            wheel.side_impulse = 0.0;
            wheel.forward_impulse = 0.0;
            wheel.is_anti_lock_brake = false;
            wheel.ground_friction = 1.0;
            wheel.ground_type = String::new();
            wheel.lock = false;
            wheel.skid_info = 0.0;
            wheel.engine_force_feedback = 0.0;
            wheel.drive_slip_demand = 0.0;
            wheel.contact_damping = wheel.base_contact_damping;
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
            wheel.ground_friction = self
                .tire_types
                .get(&wheel.tire_type)
                .map(|tire_type| tire_type.get_friction(&colliders[ground_object].material.name))
                .unwrap_or(wheel.friction_slip);

            contacts[wheel_id] = WheelContactState {
                is_grounded: true,
                ground_object: Some(ground_object),
                forward_dir,
                side_dir,
                forward_speed: wheel.contact_forward_speed,
                side_speed: side_dir.dot(&contact_velocity),
                friction_limit: wheel.wheel_suspension_force
                    * dt
                    * wheel.ground_friction
                    * wheel.friction_slip,
            };
        }

        for wheel_id in 0..num_wheels {
            let contact = &contacts[wheel_id];
            let wheel = &mut self.wheels[wheel_id];

            if !contact.is_grounded {
                wheel.last_skid_info = wheel.skid_info;
                continue;
            }

            let drive_direction = wheel.target_rotation.signum();
            let wheel_surface_speed = wheel.delta_rotation / dt.max(Real::EPSILON) * wheel.radius;
            let maximum_speed = theoretical_max_speed(&self.powertrain.config, wheel.radius);
            wheel.contact_damping = speed_adjusted_contact_damping(
                wheel.base_contact_damping,
                wheel.role.axle,
                body_speed,
                maximum_speed,
            );
            let (lateral_grip_scale, forward_grip_scale) = if wheel.role.driven {
                let longitudinal_slip = driven_wheel_longitudinal_slip(
                    wheel_surface_speed,
                    contact.forward_speed,
                    drive_direction,
                );
                longitudinal_slip_grip_scales(longitudinal_slip)
            } else {
                (1.0, 1.0)
            };

            let rolling_friction = resolve_ground_impulse(
                bodies,
                colliders,
                self.chassis,
                contact.ground_object,
                &wheel.raycast_info.contact_point_ws,
                &contact.forward_dir,
                wheel.contact_damping,
            );
            wheel.engine_force_feedback = rolling_friction;

            wheel.side_impulse = resolve_ground_impulse(
                bodies,
                colliders,
                self.chassis,
                contact.ground_object,
                &wheel.raycast_info.contact_point_ws,
                &contact.side_dir,
                wheel.contact_damping,
            );

            // if contact.side_speed.abs() < 1.0 {
            //     let side_hold = resolve_ground_impulse(
            //         bodies,
            //         colliders,
            //         self.chassis,
            //         contact.ground_object,
            //         &wheel.raycast_info.contact_point_ws,
            //         &contact.side_dir,
            //         1.0,
            //     );

            //     if side_hold.abs() > wheel.side_impulse.abs() {
            //         wheel.side_impulse = side_hold;
            //     }
            // }

            wheel.side_impulse *= wheel.side_friction_stiffness * lateral_grip_scale;
            let side_total = wheel.side_impulse * wheel.side_factor;
            let forward_friction_limit = contact.friction_limit * forward_grip_scale;
            let side_utilization_squared = if contact.friction_limit > Real::EPSILON {
                (side_total / contact.friction_limit).powi(2)
            } else if side_total.abs() > Real::EPSILON {
                Real::MAX
            } else {
                0.0
            };
            let remaining_forward_limit =
                forward_friction_limit * (1.0 - side_utilization_squared).max(0.0).sqrt();

            let powered_acceleration = wheel.role.driven
                && wheel.drivetrain_connected
                && wheel.drive_throttle > 0.1
                && wheel.engine_force * drive_direction > Real::EPSILON;
            let raw_drive_impulse = wheel.engine_force * dt * (1.0 - esc_intervention.engine_cut);
            let raw_drive_demand = (raw_drive_impulse * wheel.fwd_factor).abs();
            let capacity_excess = if powered_acceleration && raw_drive_demand > Real::EPSILON {
                drive_capacity_excess(raw_drive_demand, remaining_forward_limit)
            } else {
                0.0
            };
            let resistance_confirmed = wheel.powered_slip_timer >= POWERED_SLIP_RESISTANCE_DELAY
                && contact.forward_speed.abs() < POWERED_SLIP_RESISTANCE_MAX_ROAD_SPEED;
            let opposing_motion =
                contact.forward_speed * drive_direction < -TRACTION_CONTROL_MIN_BODY_SPEED;
            let traction_control_bypass = resistance_confirmed || opposing_motion;
            let effective_strength = effective_traction_control_strength(
                wheel.traction_control,
                body_speed,
                steering_input,
            );
            let target_traction_cut = if powered_acceleration {
                traction_control_target(
                    effective_strength,
                    wheel_surface_speed,
                    contact.forward_speed,
                    drive_direction,
                    capacity_excess,
                    traction_control_bypass,
                )
            } else {
                0.0
            };
            if effective_strength <= Real::EPSILON {
                wheel.traction_control_cut = 0.0;
            } else {
                let traction_response = if target_traction_cut > wheel.traction_control_cut {
                    TRACTION_CONTROL_ENGAGE_RESPONSE
                } else {
                    TRACTION_CONTROL_RELEASE_RESPONSE
                };
                let traction_blend = 1.0 - (-traction_response * dt).exp();
                wheel.traction_control_cut +=
                    (target_traction_cut - wheel.traction_control_cut) * traction_blend;
            }
            let traction_cut = wheel.traction_control_cut;

            wheel.forward_impulse = raw_drive_impulse * (1.0 - traction_cut);
            wheel.drive_slip_demand = if powered_acceleration && traction_control_bypass {
                1.0
            } else if powered_acceleration {
                traction_controlled_drive_slip_demand(
                    raw_drive_demand,
                    remaining_forward_limit,
                    effective_strength,
                )
            } else {
                0.0
            };

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
            // Make the brake response quadratic to make it less sensitive at low values and more sensitive at high values.
            let brake = (wheel.brake.powi(2) + esc_brake).clamp(0.0, 1.0);

            wheel.brake_impulse = rolling_friction - wheel.forward_impulse;
            if brake > 0.0 && contact.forward_speed.abs() < 1.0 {
                let hold_friction = resolve_ground_impulse(
                    bodies,
                    colliders,
                    self.chassis,
                    contact.ground_object,
                    &wheel.raycast_info.contact_point_ws,
                    &contact.forward_dir,
                    1.0,
                );
                wheel.brake_impulse = hold_friction - wheel.forward_impulse;
            }

            let mut max_brake_impulse = wheel.max_brake_force * brake;
            if max_brake_impulse > 0.0 {
                if wheel.brake >= 1.0 && max_brake_impulse >= wheel.brake_impulse.abs() {
                    wheel.lock = true;
                }
                if wheel.last_skid_info < 0.2 {
                    wheel.lock = true;
                }

                let anti_lock_brake = wheel.anti_lock_brake.clamp(0.0, 1.0);
                if anti_lock_brake > 0.0 && self.current_vehicle_speed.abs() > 1.0 {
                    let slip = ((0.3 - wheel.last_skid_info) / 0.3).clamp(0.0, 1.0);
                    let speed_factor =
                        ((self.current_vehicle_speed.abs() - 1.0) / 4.0).clamp(0.0, 1.0);
                    let slip_release = anti_lock_brake * slip * speed_factor * 1.15;
                    let steering_factor = (wheel.steering.abs() / 0.55).clamp(0.0, 1.0);
                    let lateral_factor = (contact.side_speed.abs() / 3.0).clamp(0.0, 1.0);
                    let lateral_release = anti_lock_brake
                        * brake
                        * speed_factor
                        * steering_factor.max(lateral_factor)
                        * 0.55
                        * slip;
                    let brake_release = (slip_release + lateral_release).clamp(0.0, 0.92);
                    max_brake_impulse *= 1.0 - brake_release;

                    if brake_release > 0.01 {
                        wheel.lock = false;
                        wheel.is_anti_lock_brake = true;
                    }
                }
            }

            wheel.brake_impulse = wheel
                .brake_impulse
                .clamp(-max_brake_impulse, max_brake_impulse);

            if brake > 0.0
                && self.current_vehicle_speed.abs() < 1.0
                && max_brake_impulse >= wheel.brake_impulse.abs()
            {
                wheel.lock = true;
            }

            let forward_total =
                wheel.forward_impulse * wheel.fwd_factor + wheel.brake_impulse * wheel.brake_factor;
            let forward_utilization_squared = if forward_friction_limit > Real::EPSILON {
                (forward_total / forward_friction_limit).powi(2)
            } else if forward_total.abs() > Real::EPSILON {
                Real::MAX
            } else {
                0.0
            };
            let impulse_utilization_squared =
                forward_utilization_squared + side_utilization_squared;
            let (longitudinal_slip, lateral_slip_angle) = tire_slip(
                wheel_surface_speed,
                contact.forward_speed,
                contact.side_speed,
            );
            let grip_ratio = tire_grip_ratio(
                impulse_utilization_squared,
                longitudinal_slip,
                lateral_slip_angle,
            );
            let available_utilization_squared = grip_ratio * grip_ratio;

            wheel.skid_info = 1.0;

            if impulse_utilization_squared > available_utilization_squared {
                let factor = grip_ratio * crate::utils::inv(impulse_utilization_squared.sqrt());
                wheel.skid_info = factor;
                wheel.forward_impulse *= factor;
                wheel.brake_impulse *= factor;
                wheel.side_impulse *= factor;
                wheel.engine_force_feedback *= factor;
            }

            wheel.last_skid_info = wheel.skid_info;
            wheel.forward_impulse += wheel.brake_impulse;
        }

        // apply the impulses
        {
            let chassis = bodies
                .get_mut_internal_with_modification_tracking(self.chassis)
                .unwrap();

            for wheel_id in 0..num_wheels {
                let wheel = &self.wheels[wheel_id];
                let mut impulse_point = wheel.raycast_info.contact_point_ws;

                if wheel.forward_impulse != 0.0 {
                    chassis.apply_impulse_at_point(
                        self.forward_ws[wheel_id] * wheel.forward_impulse,
                        impulse_point,
                        false,
                    );
                }
                if wheel.side_impulse != 0.0 {
                    let side_impulse = self.axle[wheel_id] * wheel.side_impulse;

                    let v_chassis_world_up =
                        chassis.position().rotation * Vector::ith(self.index_up_axis, 1.0);
                    impulse_point -= v_chassis_world_up
                        * (v_chassis_world_up.dot(&(impulse_point - chassis.center_of_mass()))
                            * wheel.anti_roll);

                    chassis.apply_impulse_at_point(side_impulse, impulse_point, false);

                    // TODO: apply friction impulse on the ground
                    // let ground_object = self.wheels[wheel_id].raycast_info.ground_object;
                    // ground_object.apply_impulse_at_point(
                    //     -side_impulse,
                    //     wheels.raycast_info.contact_point_ws,
                    //     false,
                    // );
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dynamics::RigidBodyBuilder;

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
    fn contact_damping_increases_smoothly_with_speed_and_axle_target() {
        let base = 0.15;
        let maximum_speed = 60.0;
        let midpoint_speed = (CONTACT_DAMPING_SPEED_START + maximum_speed) * 0.5;
        let expected_front_midpoint = base + (CONTACT_DAMPING_FRONT_MAX - base) * 0.5;
        let expected_rear_midpoint = base + (CONTACT_DAMPING_REAR_MAX - base) * 0.5;

        assert_eq!(
            speed_adjusted_contact_damping(
                base,
                WheelAxle::Rear,
                CONTACT_DAMPING_SPEED_START,
                maximum_speed,
            ),
            base
        );
        assert!(
            (speed_adjusted_contact_damping(base, WheelAxle::Front, midpoint_speed, maximum_speed,)
                - expected_front_midpoint)
                .abs()
                < 1.0e-6
        );
        assert!(
            (speed_adjusted_contact_damping(base, WheelAxle::Rear, midpoint_speed, maximum_speed,)
                - expected_rear_midpoint)
                .abs()
                < 1.0e-6
        );
        assert!(
            (speed_adjusted_contact_damping(base, WheelAxle::Front, maximum_speed, maximum_speed,)
                - CONTACT_DAMPING_FRONT_MAX)
                .abs()
                < 1.0e-6
        );
        assert!(
            (speed_adjusted_contact_damping(base, WheelAxle::Rear, maximum_speed, maximum_speed,)
                - CONTACT_DAMPING_REAR_MAX)
                .abs()
                < 1.0e-6
        );
    }

    #[test]
    fn contact_damping_maximum_speed_matches_highest_gear_redline() {
        let config = VehicleControllerConfig::default();
        let maximum_speed = theoretical_max_speed(&config, 0.35);

        assert!(maximum_speed > 75.0 && maximum_speed < 76.0);
    }

    #[test]
    fn contact_damping_speed_adjustment_never_reduces_a_higher_base() {
        assert_eq!(
            speed_adjusted_contact_damping(0.9, WheelAxle::Rear, 100.0, 60.0),
            0.9
        );
    }

    #[test]
    fn low_surface_grip_does_not_trigger_dynamic_friction_without_tire_slip() {
        assert_eq!(tire_grip_ratio(12.0, 0.0, 0.0), 1.0);
    }

    #[test]
    fn tire_slip_uses_wheel_and_contact_motion() {
        assert_eq!(tire_slip(10.0, 10.0, 0.0), (0.0, 0.0));

        let (longitudinal_slip, lateral_slip_angle) = tire_slip(12.0, 10.0, 1.0);
        assert!((longitudinal_slip - 0.2).abs() < 1.0e-6);
        assert!((lateral_slip_angle - 0.099_668_65).abs() < 1.0e-6);
    }

    #[test]
    fn saturated_tire_switches_to_dynamic_grip_after_actual_slip_begins() {
        assert_eq!(
            tire_grip_ratio(DYNAMIC_FRICTION_ENTER_UTILIZATION_SQUARED, 0.12, 0.0),
            1.0
        );
        assert_eq!(
            tire_grip_ratio(DYNAMIC_FRICTION_ENTER_UTILIZATION_SQUARED + 0.01, 0.12, 0.0,),
            DYNAMIC_FRICTION_RATIO
        );
        assert_eq!(
            tire_grip_ratio(
                DYNAMIC_FRICTION_ENTER_UTILIZATION_SQUARED + 0.01,
                0.0,
                (5.0 as Real).to_radians(),
            ),
            DYNAMIC_FRICTION_RATIO
        );
    }

    #[test]
    fn tire_regains_static_grip_when_actual_slip_ends() {
        assert_eq!(
            tire_grip_ratio(DYNAMIC_FRICTION_ENTER_UTILIZATION_SQUARED + 1.0, 0.0, 0.0),
            1.0
        );
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
        let dt = 0.1;
        let physical_angular_velocity = -5.0;
        controller.wheels[0].delta_rotation =
            physical_angular_velocity * dt * WHEEL_ROTATION_RETENTION;

        let (speed, radius) = controller.driven_wheel_speed_and_radius(dt);

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
        assert_eq!(powered_test_wheel().anti_roll, 0.0);
    }

    fn powered_test_wheel() -> Wheel {
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
            side_friction_stiffness: 1.0,
            tire_type: "default".to_string(),
            role: WheelRole::new(WheelAxle::Rear, true, false),
        });
        wheel.raycast_info.is_in_contact = true;
        wheel.skid_info = 1.0;
        wheel.target_rotation = 20.0;
        wheel.wheel_limit_velocity = 50.0;
        wheel.wheel_coupling_torque = 500.0;
        wheel.drive_throttle = 1.0;
        wheel.drivetrain_connected = true;
        wheel
    }

    #[test]
    fn accelerating_gripped_wheel_keeps_legacy_road_rotation() {
        let mut wheel = powered_test_wheel();
        let dt = 1.0 / 60.0;

        for step in 0..120 {
            let rolling_angular_velocity = step as Real * 0.15;
            let rotation = update_powered_wheel_rotation(&mut wheel, rolling_angular_velocity, dt);
            assert!((rotation - rolling_angular_velocity).abs() < 1.0e-5);
        }

        assert!(wheel.powered_slip < 1.0e-5);
        assert_eq!(wheel.skid_info, 1.0);
    }

    #[test]
    fn resisted_powered_wheel_transitions_into_spin_without_changing_tire_forces() {
        let mut wheel = powered_test_wheel();
        wheel.forward_impulse = 12.0;
        wheel.side_impulse = 7.0;
        let dt = 1.0 / 60.0;
        let mut rotation = 0.0;

        for _ in 0..120 {
            rotation = update_powered_wheel_rotation(&mut wheel, 0.0, dt);
        }

        assert!(wheel.powered_slip > 0.99);
        assert!(rotation > 10.0);
        assert_eq!(wheel.forward_impulse, 12.0);
        assert_eq!(wheel.side_impulse, 7.0);
        assert_eq!(wheel.skid_info, 1.0);
    }

    #[test]
    fn saturated_powered_wheel_enters_spin_without_the_resistance_delay() {
        let mut wheel = powered_test_wheel();
        wheel.drive_slip_demand = 1.0;
        let dt = 1.0 / 60.0;

        let rotation = update_powered_wheel_rotation(&mut wheel, 2.0, dt);

        assert!(wheel.powered_slip > 0.0);
        assert!(rotation > 2.0);
        assert_eq!(wheel.powered_slip_timer, 0.0);
    }

    #[test]
    fn powered_wheel_without_excess_drive_keeps_road_rotation_at_any_speed() {
        let mut wheel = powered_test_wheel();
        wheel.target_rotation = 250.0;
        wheel.wheel_limit_velocity = 250.0;
        let dt = 1.0 / 60.0;
        let rolling_angular_velocity = 30.0 / wheel.radius;

        for _ in 0..120 {
            let rotation = update_powered_wheel_rotation(&mut wheel, rolling_angular_velocity, dt);
            assert!((rotation - rolling_angular_velocity).abs() < 1.0e-5);
        }

        assert_eq!(wheel.powered_slip, 0.0);
    }

    #[test]
    fn traction_control_cut_does_not_double_suppress_powered_spin() {
        let mut wheel = powered_test_wheel();
        wheel.drive_slip_demand = 1.0;
        wheel.traction_control_cut = 1.0;
        let dt = 1.0 / 60.0;
        let mut rotation = 0.0;

        for _ in 0..120 {
            rotation = update_powered_wheel_rotation(&mut wheel, 5.0, dt);
        }

        assert!(wheel.powered_slip > 0.99);
        assert!(rotation > 10.0);
    }

    #[test]
    fn gear_change_does_not_artificially_clear_powered_spin() {
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
        let wheel = &mut controller.wheels[0];
        wheel.powered_angular_velocity = 40.0;
        wheel.powered_slip = 1.0;
        wheel.powered_slip_timer = POWERED_SLIP_RESISTANCE_DELAY;

        controller.powertrain.state_mut().current_gear = 2;
        controller.apply_powertrain_output(super::super::vehicle_powertrain::PowertrainOutput {
            drive_torque: 0.0,
            engine_brake_torque: 0.0,
            wheel_coupling_torque: 0.0,
            wheel_target_velocity: 10.0,
            wheel_limit_velocity: 50.0,
            drive_throttle: 1.0,
            drivetrain_connected: true,
            service_brake: 0.0,
        });

        let wheel = &controller.wheels[0];
        assert_eq!(wheel.powered_angular_velocity, 40.0);
        assert_eq!(wheel.powered_slip, 1.0);
        assert_eq!(wheel.powered_slip_timer, POWERED_SLIP_RESISTANCE_DELAY);
    }

    #[test]
    fn traction_control_uses_body_speed_for_straight_line_wheelspin() {
        let matching_speed = traction_control_target(0.8, 10.5, 10.0, 1.0, 0.0, false);
        let excessive_speed = traction_control_target(0.8, 15.0, 10.0, 1.0, 0.0, false);

        assert_eq!(matching_speed, 0.0);
        assert!(excessive_speed > 0.65);
        assert!(excessive_speed <= 0.8);
    }

    #[test]
    fn traction_control_supports_reverse_body_speed() {
        let cut = traction_control_target(0.8, -16.0, -10.0, -1.0, 0.0, false);

        assert_eq!(cut, 0.8);
    }

    #[test]
    fn traction_control_strength_scales_intervention_monotonically() {
        let strengths = [0.0, 0.25, 0.5, 0.75, 1.0];
        let cuts =
            strengths.map(|strength| traction_control_target(strength, 5.0, 5.0, 1.0, 0.6, false));

        assert_eq!(cuts[0], 0.0);
        assert!((cuts[1] - 0.15).abs() < 1.0e-5);
        assert!((cuts[2] - 0.3).abs() < 1.0e-5);
        assert!((cuts[3] - 0.45).abs() < 1.0e-5);
        assert!((cuts[4] - 0.6).abs() < 1.0e-5);
    }

    #[test]
    fn zero_traction_control_never_applies_electronic_intervention() {
        for body_speed in [0.0, 2.0, 5.0, 8.0, 20.0] {
            for steering_input in [-1.0, 0.0, 0.5, 1.0] {
                let strength = effective_traction_control_strength(0.0, body_speed, steering_input);
                let cut = traction_control_target(strength, 30.0, body_speed, 1.0, 1.0, false);

                assert_eq!(strength, 0.0);
                assert_eq!(cut, 0.0);
            }
        }
    }

    #[test]
    fn low_speed_override_recovers_assigned_traction_control_between_zero_and_ten_mps() {
        let strengths = [0.0, 2.5, 5.0, 7.5, 10.0, 12.0]
            .map(|speed| effective_traction_control_strength(0.8, speed, 0.0));

        assert_eq!(strengths[0], 0.0);
        assert!((strengths[1] - 0.125).abs() < 1.0e-5);
        assert!((strengths[2] - 0.4).abs() < 1.0e-5);
        assert!((strengths[3] - 0.675).abs() < 1.0e-5);
        assert_eq!(strengths[4], 0.8);
        assert_eq!(strengths[5], 0.8);
    }

    #[test]
    fn steering_override_uses_the_existing_input_range() {
        assert_eq!(traction_control_steering_factor(0.2), 0.0);
        assert!((traction_control_steering_factor(0.5) - 0.5).abs() < 1.0e-5);
        assert_eq!(traction_control_steering_factor(0.8), 1.0);
        assert_eq!(traction_control_steering_factor(-1.0), 1.0);
    }

    #[test]
    fn low_speed_and_steering_overrides_combine_independently() {
        let no_steering_at_midpoint = effective_traction_control_strength(1.0, 5.0, 0.0);
        let full_steering_at_midpoint = effective_traction_control_strength(1.0, 5.0, 1.0);
        let full_steering_at_eight = effective_traction_control_strength(1.0, 8.0, 1.0);
        let no_steering_at_ten = effective_traction_control_strength(1.0, 10.0, 0.0);
        let full_steering_at_ten = effective_traction_control_strength(1.0, 10.0, 1.0);

        assert!((no_steering_at_midpoint - 0.5).abs() < 1.0e-5);
        assert!(full_steering_at_midpoint > 0.0);
        assert!(full_steering_at_midpoint < no_steering_at_midpoint);
        assert!(full_steering_at_eight > 0.0 && full_steering_at_eight < 1.0);
        assert_eq!(no_steering_at_ten, 1.0);
        assert_eq!(full_steering_at_ten, 1.0);
    }

    #[test]
    fn full_steering_traction_control_recovers_smoothly_with_body_speed() {
        let strengths = [0.0, 1.0, 5.0, 8.0, 10.0]
            .map(|speed| effective_traction_control_strength(1.0, speed, 1.0));

        assert_eq!(strengths[0], 0.0);
        assert_eq!(strengths[1], 0.0);
        assert!(strengths.windows(2).all(|values| values[0] <= values[1]));
        assert_eq!(strengths[4], 1.0);
    }

    #[test]
    fn speed_recovery_preserves_the_assigned_strength() {
        assert!((effective_traction_control_strength(0.8, 5.0, 0.0) - 0.4).abs() < 1.0e-5);
        assert_eq!(effective_traction_control_strength(0.8, 10.0, 1.0), 0.8);
        assert_eq!(effective_traction_control_strength(0.8, 20.0, -1.0), 0.8);
    }

    #[test]
    fn traction_control_does_not_feed_its_cut_back_into_powered_slip_demand() {
        let raw_drive_demand = 12.0;
        let forward_capacity = 6.0;

        let no_tc = traction_controlled_drive_slip_demand(raw_drive_demand, forward_capacity, 0.0);
        let partial_tc =
            traction_controlled_drive_slip_demand(raw_drive_demand, forward_capacity, 0.5);
        let full_tc =
            traction_controlled_drive_slip_demand(raw_drive_demand, forward_capacity, 1.0);

        assert!((no_tc - 1.0).abs() < 1.0e-5);
        assert!((partial_tc - 0.5).abs() < 1.0e-5);
        assert_eq!(full_tc, 0.0);
    }

    #[test]
    fn steered_contact_speed_drives_rolling_rotation() {
        let steering_angle: Real = 30.0;
        let steering_angle = steering_angle.to_radians();
        let wheel_forward =
            Vector::ith(0, steering_angle.cos()) + Vector::ith(2, steering_angle.sin());
        let contact_velocity = Vector::ith(0, 20.0);
        let radius = 0.4;

        let rolling_speed =
            contact_rolling_angular_velocity(&contact_velocity, &wheel_forward, radius);
        let wheel_surface_speed = rolling_speed * radius;
        let expected_contact_speed = wheel_forward.dot(&contact_velocity);

        assert!((wheel_surface_speed - expected_contact_speed).abs() < 1.0e-5);
        assert!(wheel_surface_speed < contact_velocity[0]);
    }

    #[test]
    fn longitudinal_slip_uses_wheel_speed_relative_to_the_contact() {
        assert_eq!(driven_wheel_longitudinal_slip(10.0, 10.0, 1.0), 0.0);
        assert_eq!(driven_wheel_longitudinal_slip(11.5, 10.0, 1.0), 0.0);

        let partial_slip = driven_wheel_longitudinal_slip(12.0, 10.0, 1.0);
        assert!(partial_slip > 0.0 && partial_slip < 1.0);
        assert_eq!(driven_wheel_longitudinal_slip(13.0, 10.0, 1.0), 1.0);
    }

    #[test]
    fn longitudinal_slip_supports_reverse_and_low_speed_wheelspin() {
        assert_eq!(driven_wheel_longitudinal_slip(-13.0, -10.0, -1.0), 1.0);
        assert_eq!(driven_wheel_longitudinal_slip(3.0, 0.0, 1.0), 1.0);
        assert_eq!(driven_wheel_longitudinal_slip(5.0, 10.0, 1.0), 0.0);
        assert_eq!(driven_wheel_longitudinal_slip(10.0, 0.0, 0.0), 0.0);
    }

    #[test]
    fn longitudinal_slip_scales_with_ground_relative_overspeed() {
        let first_gear_limit = 21.8;
        assert_eq!(driven_wheel_longitudinal_slip(0.5, 0.0, 1.0), 0.0);
        assert_eq!(driven_wheel_longitudinal_slip(3.0, 0.0, 1.0), 1.0);
        assert_eq!(driven_wheel_longitudinal_slip(15.75, 15.0, 1.0), 0.0);
        assert_eq!(driven_wheel_longitudinal_slip(19.5, 15.0, 1.0), 1.0);
        assert_eq!(
            driven_wheel_longitudinal_slip(first_gear_limit, 15.0, 1.0),
            1.0
        );

        let near_first_gear_limit = driven_wheel_longitudinal_slip(first_gear_limit, 18.0, 1.0);
        assert!(near_first_gear_limit > 0.0 && near_first_gear_limit < 1.0);
    }

    #[test]
    fn longitudinal_slip_activation_reduces_only_forward_grip_quadratically() {
        let (full_lateral, full_forward) = longitudinal_slip_grip_scales(1.0);
        assert_eq!(full_lateral, 1.0);
        assert!((full_forward - LONGITUDINAL_SLIP_FORWARD_GRIP_MIN).abs() < 1.0e-6);

        let (low_slip_lateral, low_slip_forward) = longitudinal_slip_grip_scales(0.25);
        assert_eq!(low_slip_lateral, 1.0);
        assert!((low_slip_forward - 0.984_375).abs() < 1.0e-6);

        let (partial_lateral, partial_forward) = longitudinal_slip_grip_scales(0.5);
        assert_eq!(partial_lateral, 1.0);
        assert!((partial_forward - 0.9375).abs() < 1.0e-6);

        assert_eq!(longitudinal_slip_grip_scales(0.0), (1.0, 1.0));
    }

    #[test]
    fn wheel_based_grip_loss_requires_only_relative_overspeed() {
        let grip_scales = |wheel_surface_speed: Real, ground_speed: Real| {
            let slip = driven_wheel_longitudinal_slip(wheel_surface_speed, ground_speed, 1.0);
            longitudinal_slip_grip_scales(slip)
        };

        assert_eq!(grip_scales(21.0, 20.0), (1.0, 1.0));
        assert_eq!(grip_scales(21.0, 21.0), (1.0, 1.0));

        let (lateral, forward) = grip_scales(3.0, 0.0);
        assert_eq!(lateral, 1.0);
        assert!((forward - LONGITUDINAL_SLIP_FORWARD_GRIP_MIN).abs() < 1.0e-6);
    }

    #[test]
    fn cornering_capacity_increases_drive_excess() {
        assert_eq!(drive_capacity_excess(8.0, 10.0), 0.0);
        assert_eq!(drive_capacity_excess(8.0, 2.0), 0.75);
    }

    #[test]
    fn zero_traction_control_preserves_real_high_speed_longitudinal_wheelspin() {
        let raw_drive_demand = 12.0;
        let forward_capacity = 2.0;
        let mut wheel = powered_test_wheel();
        wheel.drive_slip_demand =
            residual_drive_slip_demand(raw_drive_demand, forward_capacity, raw_drive_demand);

        let rotation = update_powered_wheel_rotation(&mut wheel, 20.0, 1.0 / 60.0);

        assert!(wheel.drive_slip_demand > 0.0);
        assert!(wheel.powered_slip > 0.0);
        assert!(rotation > 20.0);
    }

    #[test]
    fn low_speed_lateral_capacity_still_supports_donuts() {
        let remaining_cornering_capacity = 2.0;
        let capacity = remaining_cornering_capacity;

        assert_eq!(capacity, remaining_cornering_capacity);
        assert!(residual_drive_slip_demand(8.0, capacity, 8.0) > 0.0);
    }

    #[test]
    fn traction_control_does_not_block_confirmed_resistance_or_opposing_burnout() {
        let resisted = traction_control_target(0.8, 20.0, 0.0, 1.0, 1.0, true);
        let opposing = traction_control_target(0.8, 20.0, -5.0, 1.0, 1.0, true);

        assert_eq!(resisted, 0.0);
        assert_eq!(opposing, 0.0);
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
            wheel_limit_velocity: 50.0,
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
    fn powertrain_updates_do_not_modify_configured_contact_damping() {
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
        let wheel = &mut controller.wheels[0];
        wheel.contact_damping = 0.15;
        wheel.base_contact_damping = 0.15;
        wheel.delta_rotation = 2.0;

        controller.apply_powertrain_output(super::super::vehicle_powertrain::PowertrainOutput {
            drive_torque: 500.0,
            engine_brake_torque: 0.0,
            wheel_coupling_torque: 500.0,
            wheel_target_velocity: 50.0,
            wheel_limit_velocity: 100.0,
            drive_throttle: 1.0,
            drivetrain_connected: true,
            service_brake: 0.0,
        });

        assert_eq!(controller.wheels[0].contact_damping, 0.15);
    }

    #[test]
    fn powered_spin_preserves_reverse_rotation_direction() {
        let mut wheel = powered_test_wheel();
        wheel.target_rotation = -20.0;
        wheel.wheel_limit_velocity = -50.0;
        wheel.wheel_coupling_torque = -500.0;
        let dt = 1.0 / 60.0;
        let mut rotation = 0.0;

        for _ in 0..120 {
            rotation = update_powered_wheel_rotation(&mut wheel, 0.0, dt);
        }

        assert!(wheel.powered_slip > 0.99);
        assert!(rotation < -10.0);
    }

    #[test]
    fn locked_wheel_clears_powered_spin_state() {
        let mut wheel = powered_test_wheel();
        wheel.powered_angular_velocity = 30.0;
        wheel.powered_slip = 1.0;
        wheel.powered_slip_timer = POWERED_SLIP_RESISTANCE_DELAY;
        wheel.lock = true;

        let rotation = update_powered_wheel_rotation(&mut wheel, 0.0, 1.0 / 60.0);

        assert_eq!(rotation, 0.0);
        assert_eq!(wheel.powered_angular_velocity, 0.0);
        assert_eq!(wheel.powered_slip, 0.0);
        assert_eq!(wheel.powered_slip_timer, 0.0);
    }

    #[test]
    fn locked_wheel_rotation_update_clears_powered_spin_state() {
        let mut wheel = powered_test_wheel();
        wheel.rotation = 4.0;
        wheel.delta_rotation = 0.5;
        wheel.visual_angular_velocity = 30.0;
        wheel.powered_angular_velocity = 30.0;
        wheel.powered_slip = 1.0;
        wheel.powered_slip_timer = POWERED_SLIP_RESISTANCE_DELAY;
        wheel.lock = true;

        update_wheel_rotation(&mut wheel, 0.0, 1.0 / 60.0);

        assert!(wheel.rotation > 4.0);
        assert!(wheel.visual_angular_velocity > 0.0);
        assert!(wheel.visual_angular_velocity < 30.0);
        assert_eq!(wheel.delta_rotation, 0.0);
        assert_eq!(wheel.powered_angular_velocity, 0.0);
        assert_eq!(wheel.powered_slip, 0.0);
        assert_eq!(wheel.powered_slip_timer, 0.0);

        for _ in 0..30 {
            update_wheel_rotation(&mut wheel, 0.0, 1.0 / 60.0);
        }
        assert_eq!(wheel.visual_angular_velocity, 0.0);
    }

    #[test]
    fn powered_spin_smoothly_releases_when_drive_fits_available_traction() {
        let mut wheel = powered_test_wheel();
        wheel.powered_angular_velocity = 30.0;
        wheel.powered_slip = 1.0;
        wheel.drive_slip_demand = 1.0;
        let dt = 1.0 / 60.0;
        let rolling_angular_velocity = 5.0;

        update_powered_wheel_rotation(&mut wheel, rolling_angular_velocity, dt);
        assert_eq!(wheel.powered_slip, 1.0);

        wheel.drive_slip_demand = 0.0;
        update_powered_wheel_rotation(&mut wheel, rolling_angular_velocity, dt);
        assert!(wheel.powered_slip > 0.9);

        for _ in 0..180 {
            update_powered_wheel_rotation(&mut wheel, rolling_angular_velocity, dt);
        }
        assert!(wheel.powered_slip < 2.0e-4);
    }

    #[test]
    fn released_powered_spin_smoothly_returns_to_road_rotation() {
        let mut wheel = powered_test_wheel();
        wheel.powered_angular_velocity = 30.0;
        wheel.powered_slip = 1.0;
        wheel.powered_slip_timer = POWERED_SLIP_RESISTANCE_DELAY;
        wheel.drive_throttle = 0.0;
        wheel.wheel_coupling_torque = 0.0;
        let dt = 1.0 / 60.0;
        let rolling_angular_velocity = 5.0;
        let mut rotation = 0.0;

        for _ in 0..300 {
            rotation = update_powered_wheel_rotation(&mut wheel, rolling_angular_velocity, dt);
        }

        assert!(wheel.powered_slip < 1.0e-5);
        assert!((rotation - rolling_angular_velocity).abs() < 1.0e-4);
    }

    #[test]
    fn visual_wheel_regrips_before_legacy_powered_slip_finishes_releasing() {
        let mut wheel = powered_test_wheel();
        wheel.powered_angular_velocity = 30.0;
        wheel.powered_slip = 1.0;
        wheel.powered_slip_timer = POWERED_SLIP_RESISTANCE_DELAY;
        wheel.powered_spin_requested = true;
        wheel.visual_angular_velocity = 30.0;
        wheel.drive_throttle = 0.0;
        wheel.wheel_coupling_torque = 0.0;
        let dt = 1.0 / 60.0;
        let rolling_angular_velocity = 5.0;

        for _ in 0..20 {
            update_wheel_rotation(&mut wheel, rolling_angular_velocity, dt);
        }

        let physical_angular_velocity = wheel.delta_rotation / (dt * 0.99);
        assert_eq!(wheel.visual_angular_velocity, rolling_angular_velocity);
        assert!(!wheel.visual_regrip_active);
        assert!(physical_angular_velocity > rolling_angular_velocity);
        assert!(wheel.powered_slip > 0.0);
    }

    #[test]
    fn powered_wheel_spin_raises_clutch_coupled_engine_rpm() {
        let mut config = VehicleControllerConfig::default();
        config.transmission.automatic = false;
        config.transmission.auto_clutch = true;
        let idle_rpm = config.engine.idle_rpm;
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
        controller.set_gear(1);
        controller.set_input(VehicleInput {
            throttle: 1.0,
            ..VehicleInput::default()
        });
        let dt = 1.0 / 60.0;

        for _ in 0..180 {
            let (wheel_speed, wheel_radius) = controller.driven_wheel_speed_and_radius(dt);
            let output = controller
                .powertrain
                .update(dt, 0.0, wheel_speed, wheel_radius);
            controller.apply_powertrain_output(output);
            let wheel = &mut controller.wheels[0];
            wheel.raycast_info.is_in_contact = true;
            wheel.skid_info = 1.0;
            wheel.delta_rotation = update_powered_wheel_rotation(wheel, 0.0, dt) * dt * 0.99;
        }

        assert!(controller.wheels[0].powered_slip > 0.99);
        assert!(controller.state().engine_rpm > idle_rpm * 2.0);
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

        controller.set_steering_assist(false);
        controller.update_steering(&chassis, 1.0 / 60.0);
        assert!((controller.state().steering_angle - max_angle).abs() < 1.0e-5);
    }

    #[test]
    fn steering_assist_speed_activation_blends_from_five_to_ten_meters_per_second() {
        assert_eq!(drift_assist_speed_activation(-10.0), 0.0);
        assert_eq!(drift_assist_speed_activation(5.0), 0.0);
        assert!((drift_assist_speed_activation(7.5) - 0.5).abs() < 1.0e-5);
        assert_eq!(drift_assist_speed_activation(10.0), 1.0);
        assert_eq!(drift_assist_speed_activation(20.0), 1.0);
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
        assert_eq!(controller.drift_assist_offset, 0.0);

        let just_above_minimum_chassis = RigidBodyBuilder::dynamic()
            .linvel(Vector::z() * 5.001 + Vector::x() * 5.0)
            .build();
        controller.current_vehicle_speed = 5.001;
        controller.update_steering(&just_above_minimum_chassis, 1.0 / 60.0);

        assert!(controller.drift_assist_offset.abs() > 0.0);
        assert!(controller.drift_assist_offset.abs() < 1.0e-6);
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
        assert!(!controller.drift_assist_active);
        assert_eq!(controller.drift_assist_offset, 0.0);
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
        assert!(controller.drift_assist_offset > 0.0);
        assert!(controller.drift_assist_offset < correction_angle - player_angle);
    }

    #[test]
    fn drift_correction_strength_blends_between_player_and_full_correction() {
        fn controller(strength: Real) -> DynamicRayCastVehicleController {
            let mut config = VehicleControllerConfig::default();
            config.steering.assist = true;
            config.steering.drift_correction = strength;
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
        assert_eq!(controller.drift_assist_offset, 0.0);
    }

    #[test]
    fn opposite_input_releases_drift_correction_immediately() {
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
        assert!(controller.drift_assist_offset > 0.1);

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
        assert_eq!(controller.drift_assist_offset, 0.0);
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
    fn drift_correction_setter_clamps_to_normalized_range() {
        let mut controller = DynamicRayCastVehicleController::new(
            RigidBodyHandle::invalid(),
            VehicleControllerConfig::default(),
        );

        controller.set_drift_correction(2.0);
        assert_eq!(controller.powertrain.config.steering.drift_correction, 1.0);
        controller.set_drift_correction(-1.0);
        assert_eq!(controller.powertrain.config.steering.drift_correction, 0.0);
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
        controller.drift_assist_active = true;
        controller.drift_assist_offset = 0.3;
        controller.drift_assist_direction = 1.0;
        controller.timer = 5.0;
        let wheel = &mut controller.wheels[0];
        wheel.rotation = 10.0;
        wheel.delta_rotation = 2.0;
        wheel.target_rotation = 3.0;
        wheel.powered_angular_velocity = 20.0;
        wheel.powered_slip = 1.0;
        wheel.powered_slip_timer = 1.0;
        wheel.powered_spin_requested = true;
        wheel.visual_angular_velocity = 15.0;
        wheel.visual_regrip_active = true;
        wheel.drive_slip_demand = 1.0;
        wheel.traction_control = 0.35;
        wheel.engine_force = 100.0;
        wheel.brake = 0.5;
        wheel.steering = 0.4;
        wheel.skid_info = 0.2;
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
        assert!(!controller.drift_assist_active);
        assert_eq!(controller.drift_assist_offset, 0.0);
        assert_eq!(controller.timer, 0.0);
        let wheel = &controller.wheels[0];
        assert_eq!(wheel.rotation, 0.0);
        assert_eq!(wheel.delta_rotation, 0.0);
        assert_eq!(wheel.target_rotation, 0.0);
        assert_eq!(wheel.powered_angular_velocity, 0.0);
        assert_eq!(wheel.powered_slip, 0.0);
        assert_eq!(wheel.powered_slip_timer, 0.0);
        assert!(!wheel.powered_spin_requested);
        assert_eq!(wheel.visual_angular_velocity, 0.0);
        assert!(!wheel.visual_regrip_active);
        assert_eq!(wheel.drive_slip_demand, 0.0);
        assert_eq!(wheel.traction_control, 0.35);
        assert_eq!(wheel.engine_force, 0.0);
        assert_eq!(wheel.brake, 0.0);
        assert_eq!(wheel.steering, 0.0);
        assert_eq!(wheel.skid_info, 0.0);
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

fn resolve_single_bilateral(
    body1: &RigidBody,
    pt1: &Point<Real>,
    body2: &RigidBody,
    pt2: &Point<Real>,
    normal: &Vector<Real>,
    contact_damping: Real,
) -> Real {
    let vel1 = body1.velocity_at_point(pt1);
    let vel2 = body2.velocity_at_point(pt2);
    let dvel = vel1 - vel2;

    let dpt1 = pt1 - body1.center_of_mass();
    let dpt2 = pt2 - body2.center_of_mass();
    let aj = dpt1.gcross(*normal);
    let bj = dpt2.gcross(-*normal);
    let iaj = body1.mprops.effective_world_inv_inertia_sqrt * aj;
    let ibj = body2.mprops.effective_world_inv_inertia_sqrt * bj;

    // TODO: take the effective_inv_mass into account?
    let im1 = body1.mprops.local_mprops.inv_mass;
    let im2 = body2.mprops.local_mprops.inv_mass;

    let jac_diag_ab = im1 + im2 + iaj.gdot(iaj) + ibj.gdot(ibj);
    let jac_diag_ab_inv = crate::utils::inv(jac_diag_ab);
    let rel_vel = normal.dot(&dvel);

    //todo: move this into proper structure
    -contact_damping * rel_vel * jac_diag_ab_inv
}

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
        .filter(|b| b.is_dynamic())
        .map(|b| b.velocity_at_point(point))
        .unwrap_or_else(Vector::zeros);

    chassis_velocity - ground_velocity
}

fn resolve_ground_impulse(
    bodies: &RigidBodySet,
    colliders: &ColliderSet,
    chassis: RigidBodyHandle,
    ground_object: Option<ColliderHandle>,
    point: &Point<Real>,
    direction: &Vector<Real>,
    contact_damping: Real,
) -> Real {
    if let Some(ground_body) = ground_object
        .and_then(|h| colliders[h].parent())
        .map(|h| &bodies[h])
        .filter(|b| b.is_dynamic())
    {
        resolve_single_bilateral(
            &bodies[chassis],
            point,
            ground_body,
            point,
            direction,
            contact_damping,
        )
    } else {
        resolve_single_unilateral(&bodies[chassis], point, direction, contact_damping)
    }
}

fn resolve_single_unilateral(
    body1: &RigidBody,
    pt1: &Point<Real>,
    normal: &Vector<Real>,
    contact_damping: Real,
) -> Real {
    let vel1 = body1.velocity_at_point(pt1);
    let dvel = vel1;
    let dpt1 = pt1 - body1.center_of_mass();
    let aj = dpt1.gcross(*normal);
    let iaj = body1.mprops.effective_world_inv_inertia_sqrt * aj;

    // TODO: take the effective_inv_mass into account?
    let im1 = body1.mprops.local_mprops.inv_mass;
    let jac_diag_ab = im1 + iaj.gdot(iaj);
    let jac_diag_ab_inv = crate::utils::inv(jac_diag_ab);
    let rel_vel = normal.dot(&dvel);

    //todo: move this into proper structure
    -contact_damping * rel_vel * jac_diag_ab_inv
}

#[derive(Clone, Debug, PartialEq)]
/// Dynamic tire type with configurable friction coefficients for different surfaces
pub struct TireType {
    /// Name of the tire type
    pub name: String,
    /// Default friction coefficient
    pub default_friction: Real,
    /// Map of surface material names to friction coefficients
    pub surface_friction: HashMap<String, Real>,
}

impl TireType {
    /// Creates a new tire type with the given name and default friction
    pub fn new(name: &str, default_friction: Real) -> Self {
        Self {
            name: name.to_string(),
            default_friction,
            surface_friction: HashMap::new(),
        }
    }

    /// Adds a surface material and its friction coefficient
    pub fn add_surface(&mut self, surface_name: &str, friction: Real) {
        self.surface_friction
            .insert(surface_name.to_string(), friction);
    }

    /// Gets the friction coefficient for a given surface material
    pub fn get_friction(&self, surface_name: &str) -> Real {
        self.surface_friction
            .get(surface_name)
            .copied()
            .unwrap_or(self.default_friction)
    }

    /// Removes a surface material
    pub fn remove_surface(&mut self, surface_name: &str) -> Option<Real> {
        self.surface_friction.remove(surface_name)
    }

    /// Lists all configured surface materials
    pub fn list_surfaces(&self) -> Vec<&String> {
        self.surface_friction.keys().collect()
    }
}
