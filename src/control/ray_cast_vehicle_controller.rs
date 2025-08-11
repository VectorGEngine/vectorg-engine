
use crate::dynamics::{RigidBody, RigidBodyHandle, RigidBodySet};
use crate::geometry::{ColliderHandle, ColliderSet, Ray};
use crate::math::{Point, Real, Rotation, Vector};
use crate::pipeline::{QueryFilter, QueryPipeline};
use crate::utils::{SimdCross, SimdDot};
use std::collections::HashMap;

/// A character controller to simulate vehicles using ray-casting for the wheels.
pub struct DynamicRayCastVehicleController {
    wheels: Vec<Wheel>,
    forward_ws: Vec<Vector<Real>>,
    axle: Vec<Vector<Real>>,
    /// The current forward speed of the vehicle.
    pub current_vehicle_speed: Real,

    /// Handle of the vehicle’s chassis.
    pub chassis: RigidBodyHandle,
    /// The chassis’ local _up_ direction (`0 = x, 1 = y, 2 = z`)
    pub index_up_axis: usize,
    /// The chassis’ local _forward_ direction (`0 = x, 1 = y, 2 = z`)
    pub index_forward_axis: usize,
    /// Available tire types
    pub tire_types: HashMap<String, TireType>,
    
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
    /// The target rotation of the wheel.
    pub target_rotation: Real,
    /// The influence of the roll on the wheel’s suspension.
    pub anti_roll: Real,
    /// The maximum force applied by the suspension.
    pub max_suspension_force: Real,

    /// The forward impulses applied by the wheel on the chassis.
    pub forward_impulse: Real,
    /// The side impulses applied by the wheel on the chassis.
    pub side_impulse: Real,

    /// The steering angle for this wheel.
    pub steering: Real,
    /// The forward force applied by this wheel on the chassis.
    pub engine_force: Real,
    /// The maximum brakking multiplier applied to this wheel.
    pub brake: Real,
    /// The maximum amount of braking impulse applied to slow down the vehicle.
    pub max_brake_force: Real,
    /// The anti-lock braking system force applied to this wheel.
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
    /// The force applied by the suspension.
    pub wheel_suspension_force: Real,
    /// The amount of skid information for this wheel.
    pub skid_info: Real,
    last_skid_info: Real,
    /// The ground friction multiplier for this wheel.
    pub ground_friction: Real,
    /// The suspension compression ratio, where 1.0 means the suspension is at its rest length.
    pub suspension_compression_rate: Real,
    /// The type of tire for friction calculations
    pub tire_type: String,
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
            brake: 0.0,
            max_brake_force: 1000.0,
            anti_lock_brake: 0.0,
            is_anti_lock_brake: false,
            traction_control: 0.0,
            engine_force_feedback: 0.0,
            anti_roll: 0.8,
            clipped_inv_contact_dot_suspension: 0.0,
            suspension_relative_velocity: 0.0,
            wheel_suspension_force: 0.0,
            max_suspension_force: info.max_suspension_force,
            skid_info: 0.0,
            last_skid_info: 0.0,
            side_impulse: 0.0,
            forward_impulse: 0.0,
            side_friction_stiffness: info.side_friction_stiffness,
            lock: false,
            tire_type: info.tire_type,
            suspension_compression_rate: 0.0,
            ground_friction: 1.0,
        }
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

impl DynamicRayCastVehicleController {
    /// Creates a new vehicle represented by the given rigid-body.
    ///
    /// Wheels have to be attached afterwards calling [`Self::add_wheel`].
    pub fn new(chassis: RigidBodyHandle) -> Self {
        let mut tire_types = HashMap::new();
        
        // Create default tire types        
        tire_types.insert("default".to_string(), TireType::new("default", 1.0));
        
        Self {
            wheels: vec![],
            forward_ws: vec![],
            axle: vec![],
            current_vehicle_speed: 0.0,
            chassis,
            index_up_axis: 1,
            index_forward_axis: 0,
            tire_types,
            timer: 0.0,
        }
    }

    /// Adds a new tire type to the controller
    pub fn add_tire_type(&mut self, tire_type: &str, friction: Real) {
        self.tire_types.insert(tire_type.to_string(), TireType::new(tire_type, friction));
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

    /// Adds a surface to an existing tire type
    pub fn add_surface_to_tire_type(&mut self, tire_type_name: &str, surface_name: &str,friction: Real) {
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
        };

        let wheel_id = self.wheels.len();
        self.wheels.push(Wheel::new(ci));

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

        for i in 0..num_wheels {
            self.update_wheel_transform(chassis, i);
        }

        self.current_vehicle_speed = chassis.linvel().norm();

        let forward_w = chassis.position() * Vector::ith(self.index_forward_axis, 1.0);

        if forward_w.dot(chassis.linvel()) < 0.0 {
            self.current_vehicle_speed *= -1.0;
        }

        //
        // simulate suspension
        //

        for wheel_id in 0..self.wheels.len() {
            self.ray_cast(bodies, colliders, queries, filter, chassis, wheel_id);
        }

        let chassis_mass = chassis.mass();
        self.update_suspension(chassis_mass);

        let chassis = bodies
            .get_mut_internal_with_modification_tracking(self.chassis)
            .unwrap();

        for wheel in &mut self.wheels {
            if wheel.engine_force > 0.0 {
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

        let chassis = bodies
            .get_mut_internal_with_modification_tracking(self.chassis)
            .unwrap();

        for wheel in &mut self.wheels {
            let vel = chassis.velocity_at_point(&wheel.raycast_info.hard_point_ws);
            let target_rotation = wheel.target_rotation * dt;
            if wheel.lock {
                wheel.delta_rotation = 0.0;
            }
            else {
                if wheel.raycast_info.is_in_contact {
                    let mut fwd = chassis.position() * Vector::ith(self.index_forward_axis, 1.0);
                    let proj = fwd.dot(&wheel.raycast_info.contact_normal_ws);
                    fwd -= wheel.raycast_info.contact_normal_ws * proj;

                    let proj2 = fwd.dot(&vel);
                    wheel.delta_rotation = (proj2 * dt) / (wheel.radius);
                    if wheel.skid_info < 0.8 && wheel.engine_force.abs() > 0.0 {
                        let mut speed_factor = 0.0;
                        if self.current_vehicle_speed.abs() > 1.0 {
                            speed_factor = (self.current_vehicle_speed.abs() / 20.0).powi(2);
                            if speed_factor > 1.0 {
                                speed_factor = 1.0;
                            }
                        }
                        // Apply custom rotation when accelerating and sliding
                        wheel.delta_rotation = wheel.delta_rotation * wheel.traction_control * speed_factor + (wheel.delta_rotation + ((target_rotation - wheel.delta_rotation) * (1.0 - wheel.skid_info.powi(2)))) * (1.0 - wheel.traction_control * speed_factor);
                    }
                } else {
                    if wheel.engine_force.abs() > 0.0 && target_rotation != 0.0 {
                        wheel.delta_rotation = target_rotation;
                    }
                    wheel.delta_rotation *= 1.0 - wheel.brake; // Apply brake
                }
            }

            wheel.rotation += wheel.delta_rotation;
            wheel.delta_rotation *= 0.99; //damping of rotation when not in contact
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
                    wheels.suspension_compression_rate =  1.0 - (current_length / rest_length);

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

    #[profiling::function]
    fn update_friction(&mut self, bodies: &mut RigidBodySet, colliders: &ColliderSet, dt: Real) {
        let num_wheels = self.wheels.len();

        if num_wheels == 0 {
            return;
        }

        self.forward_ws.resize(num_wheels, Default::default());
        self.axle.resize(num_wheels, Default::default());

        let mut num_wheels_on_ground = 0;

        //TODO: collapse all those loops into one!
        for wheel in &mut self.wheels {
            let ground_object = wheel.raycast_info.ground_object;

            if ground_object.is_some() {
                num_wheels_on_ground += 1;
            }

            wheel.side_impulse = 0.0;
            wheel.forward_impulse = 0.0;
            wheel.is_anti_lock_brake = false;
            wheel.ground_friction = 1.0;
            wheel.lock = false;
            wheel.skid_info = 0.0;
            wheel.engine_force_feedback = 0.0;
        }

        {
            for i in 0..num_wheels {
                let wheel = &mut self.wheels[i];
                let ground_object = wheel.raycast_info.ground_object;

                if ground_object.is_some() {
                    self.axle[i] = wheel.wheel_axle_ws;

                    let surf_normal_ws = wheel.raycast_info.contact_normal_ws;
                    let proj = self.axle[i].dot(&surf_normal_ws);
                    self.axle[i] -= surf_normal_ws * proj;
                    self.axle[i] = self.axle[i]
                        .try_normalize(1.0e-5)
                        .unwrap_or_else(Vector::zeros);
                    self.forward_ws[i] = surf_normal_ws
                        .cross(&self.axle[i])
                        .try_normalize(1.0e-5)
                        .unwrap_or_else(Vector::zeros);

                    if let Some(ground_body) = ground_object
                        .and_then(|h| colliders[h].parent())
                        .map(|h| &bodies[h])
                        .filter(|b| b.is_dynamic())
                    {
                        wheel.side_impulse = resolve_single_bilateral(
                            &bodies[self.chassis],
                            &wheel.raycast_info.contact_point_ws,
                            ground_body,
                            &wheel.raycast_info.contact_point_ws,
                            &self.axle[i],
                        );
                    } else {
                        wheel.side_impulse = resolve_single_unilateral(
                            &bodies[self.chassis],
                            &wheel.raycast_info.contact_point_ws,
                            &self.axle[i],
                        );
                    }

                    wheel.side_impulse *= wheel.side_friction_stiffness;
                }
            }
        }

        let side_factor = 0.9;
        let fwd_factor = 1.6;

        let mut sliding = false;
        {
            for wheel_id in 0..num_wheels {
                let wheel = &mut self.wheels[wheel_id];
                let ground_object = wheel.raycast_info.ground_object;

                let mut rolling_friction = 0.0;

                if ground_object.is_some() {
                    // let default_rolling_friction_impulse = 0.0;
                    // let max_impulse = if wheel.brake != 0.0 {
                    //     wheel.brake * wheel.max_brake_force
                    // } else {
                    //     default_rolling_friction_impulse
                    // };
                    // let contact_pt = WheelContactPoint::new(
                    //     &bodies[self.chassis],
                    //     ground_object
                    //         .and_then(|h| colliders[h].parent())
                    //         .map(|h| &bodies[h]),
                    //     wheel.raycast_info.contact_point_ws,
                    //     self.forward_ws[wheel_id],
                    //     max_impulse,
                    // );
                    // assert!(num_wheels_on_ground > 0);
                    // rolling_friction = contact_pt.calc_rolling_friction(num_wheels_on_ground);

                    assert!(num_wheels_on_ground > 0);
                    if let Some(ground_body) = ground_object
                        .and_then(|h| colliders[h].parent())
                        .map(|h| &bodies[h])
                        .filter(|b| b.is_dynamic())
                    {
                        rolling_friction = resolve_single_bilateral(
                            &bodies[self.chassis],
                            &wheel.raycast_info.contact_point_ws,
                            ground_body,
                            &wheel.raycast_info.contact_point_ws,
                            &self.forward_ws[wheel_id],
                        )
                    } else {
                        rolling_friction = resolve_single_unilateral(
                            &bodies[self.chassis],
                            &wheel.raycast_info.contact_point_ws,
                            &self.forward_ws[wheel_id],
                        )
                    }
                    wheel.engine_force_feedback = rolling_friction;

                    let engine_force = if wheel.last_skid_info < 0.8 && wheel.engine_force.abs() > 0.0 {
                        let mut speed_factor = 0.0;
                        if self.current_vehicle_speed.abs() > 1.0 {
                            speed_factor = (self.current_vehicle_speed.abs() / 20.0).powi(2);
                            if speed_factor > 1.0 {
                                speed_factor = 1.0;
                            }
                        }
                        wheel.engine_force * dt * (1.0 - wheel.traction_control * speed_factor)
                    } else {
                        wheel.engine_force * dt
                    };

                    let mut brake_friction = rolling_friction - engine_force;
                    let mut max_impulse = wheel.max_brake_force * wheel.brake;

                    if wheel.last_skid_info < 0.2 && max_impulse.abs() > 0.0 {
                        wheel.lock = true;
                        if wheel.last_skid_info < wheel.anti_lock_brake * 0.2 && self.current_vehicle_speed.abs() > 1.0 {
                            if self.timer % (dt * 4.0) < dt * 2.0 {
                                max_impulse = 0.0;
                                wheel.lock = false;
                                wheel.is_anti_lock_brake = true;
                            }
                        }
                    }

                    if max_impulse < brake_friction {
                        brake_friction = max_impulse;
                    }
                    if -max_impulse > brake_friction {
                        brake_friction = -max_impulse;
                    }

                    brake_friction += engine_force;

                    // wheel.lock = wheel.brake != 0.0 && brake_friction.abs() > rolling_friction.abs() * 0.5;

                    // if wheel.lock && wheel.anti_lock_brake > 0.0 && self.current_vehicle_speed.abs() > 1.0 {
                    //     if self.timer % (dt * 4.0) < dt * 3.0 {
                    //         brake_friction *= 1.0 - wheel.anti_lock_brake;
                    //         wheel.lock = false;
                    //         wheel.is_anti_lock_brake = true;
                    //     }
                    // }
                    rolling_friction = brake_friction;
                }

                //switch between active rolling (throttle), braking and non-active rolling friction (no throttle/break)

                if ground_object.is_some() {
                    wheel.skid_info = 1.0;
                    // Get ground friction from collider material name
                    wheel.ground_friction = ground_object
                        .map(|h| &colliders[h])
                        .and_then(|collider| {
                            // Get tire type and calculate friction
                            self.tire_types.get(&wheel.tire_type)
                                .map(|tire_type| tire_type.get_friction(&collider.material.name))
                        })
                        .unwrap_or(wheel.friction_slip); // Default friction if no tire type or unknown material

                    let max_imp = wheel.wheel_suspension_force * dt * wheel.ground_friction * wheel.friction_slip;
                    let max_imp_side = max_imp;
                    let max_imp_squared = max_imp * max_imp_side;
                    assert!(max_imp_squared >= 0.0);

                    wheel.forward_impulse = rolling_friction;

                    let x = wheel.forward_impulse * fwd_factor;
                    let y = wheel.side_impulse * side_factor;

                    let impulse_squared = x * x + y * y;

                    if impulse_squared > max_imp_squared {
                        sliding = true;

                        let factor = max_imp * crate::utils::inv((impulse_squared).sqrt());
                        wheel.skid_info *= factor;
                    }
                }
                 wheel.last_skid_info = wheel.skid_info;
            }
        }

        if sliding {
            for wheel in &mut self.wheels {
                if wheel.skid_info < 1.0 {
                    wheel.forward_impulse *= wheel.skid_info;
                    wheel.side_impulse *= wheel.skid_info;
                    wheel.engine_force_feedback *= wheel.skid_info;
                }
            }
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

                    let v_chassis_world_up = chassis.position().rotation * Vector::ith(self.index_up_axis, 1.0);
                    impulse_point -= v_chassis_world_up * (v_chassis_world_up.dot(&(impulse_point - chassis.center_of_mass())) * wheel.anti_roll);

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
    let contact_damping = 0.6;
    -contact_damping * rel_vel * jac_diag_ab_inv
}

fn resolve_single_unilateral(body1: &RigidBody, pt1: &Point<Real>, normal: &Vector<Real>) -> Real {
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
    let contact_damping = 0.6;
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
        self.surface_friction.insert(surface_name.to_string(), friction);
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
