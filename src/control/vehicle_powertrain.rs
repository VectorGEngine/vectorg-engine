use crate::math::Real;

const TAU: Real = 6.283_185_307_179_586 as Real;
const TRANSMISSION_PEDAL_ENGAGE: Real = 0.1;
const TRANSMISSION_PEDAL_RELEASE: Real = 0.05;
const TRANSMISSION_MANUAL_OVERRIDE: Real = 1.5;

/// Engine parameters used by the vehicle powertrain.
#[derive(Clone, Debug)]
pub struct EngineConfig {
    /// Rated engine horsepower, used to generate a fallback torque curve.
    pub horsepower: Real,
    /// Minimum running engine speed.
    pub idle_rpm: Real,
    /// Maximum supported engine speed.
    pub max_rpm: Real,
    /// Engine speed where combustion torque is cut.
    pub rev_limit_rpm: Real,
    /// Rotational inertia controlling free-rev response.
    pub inertia: Real,
    /// Optional fixed friction-torque scale.
    pub friction_torque: Option<Real>,
    /// Engine-braking strength.
    pub engine_braking: Real,
    /// Fraction of engine torque delivered through the drivetrain.
    pub drivetrain_efficiency: Real,
    /// Final gameplay scaling applied to drive and engine-braking torque.
    pub force_scale: Real,
    /// Exponent applied to gear ratios when calculating wheel torque.
    pub gear_force_exponent: Real,
    /// Ordered pairs of engine speed and torque.
    pub torque_curve: Vec<(Real, Real)>,
}

impl Default for EngineConfig {
    fn default() -> Self {
        Self {
            horsepower: 150.0,
            idle_rpm: 900.0,
            max_rpm: 6500.0,
            rev_limit_rpm: 6400.0,
            inertia: 0.9,
            friction_torque: None,
            engine_braking: 0.2,
            drivetrain_efficiency: 0.9,
            force_scale: 1.0,
            gear_force_exponent: 1.0,
            torque_curve: vec![],
        }
    }
}

/// Gearbox and clutch parameters.
#[derive(Clone, Debug)]
pub struct TransmissionConfig {
    /// Reverse gear ratio. Positive values are normalized to negative.
    pub reverse_ratio: Real,
    /// Forward gear ratios ordered from first to highest gear.
    pub forward_ratios: Vec<Real>,
    /// Final-drive differential ratio.
    pub final_drive_ratio: Real,
    /// Whether road-speed-based automatic shifting is enabled.
    pub automatic: bool,
    /// Whether stopped pedal input automatically selects forward or reverse.
    pub auto_reverse: bool,
    /// Rate at which clutch engagement couples engine and wheel RPM.
    pub clutch_response: Real,
    /// Minimum time between gear changes.
    pub shift_cooldown: Real,
    /// Position within adjacent gears' speed overlap used for upshifts.
    pub upshift_range_position: Real,
    /// Position within adjacent gears' speed overlap used for downshifts.
    pub downshift_range_position: Real,
    /// Absolute vehicle speed considered stopped.
    pub stopped_speed: Real,
}

impl Default for TransmissionConfig {
    fn default() -> Self {
        Self {
            reverse_ratio: -3.2,
            forward_ratios: vec![3.2, 2.1, 1.5, 1.1, 0.85],
            final_drive_ratio: 3.7,
            automatic: true,
            auto_reverse: true,
            clutch_response: 12.0,
            shift_cooldown: 0.73,
            upshift_range_position: 0.9,
            downshift_range_position: 0.7,
            stopped_speed: 0.1,
        }
    }
}

/// Turbocharger spool and release parameters.
#[derive(Clone, Debug)]
pub struct TurboConfig {
    /// Whether turbo boost is simulated.
    pub enabled: bool,
    /// Maximum engine-torque multiplier at full turbo load.
    pub max_boost: Real,
    /// Turbo-load increase per second at full throttle.
    pub spool_rate: Real,
    /// Turbo-load decrease per second after throttle release.
    pub release_rate: Real,
}

impl Default for TurboConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            max_boost: 1.0,
            spool_rate: 1.0,
            release_rate: 3.0,
        }
    }
}

/// Braking, stability, aerodynamics, and chassis-damping parameters.
#[derive(Clone, Debug)]
pub struct VehicleDynamicsConfig {
    /// Fraction of service braking assigned to the front axle.
    pub brake_bias: Real,
    /// Anti-lock braking strength.
    pub abs_strength: Real,
    /// Driven-wheel traction-control strength.
    pub traction_control_strength: Real,
    /// Electronic stability-control strength.
    pub esc_strength: Real,
    /// Aerodynamic drag coefficient.
    pub drag_coefficient: Real,
    /// Vehicle frontal area in square meters.
    pub frontal_area: Real,
    /// Rolling-resistance coefficient.
    pub rolling_resistance: Real,
    /// Downforce in newtons per squared meter-per-second.
    pub downforce_coefficient: Real,
    /// Chassis linear damping at rest.
    pub base_linear_damping: Real,
    /// Additional linear damping per meter-per-second.
    pub linear_damping_per_speed: Real,
    /// Chassis angular damping at rest.
    pub base_angular_damping: Real,
    /// Additional angular damping per meter-per-second.
    pub angular_damping_per_speed: Real,
}

impl Default for VehicleDynamicsConfig {
    fn default() -> Self {
        Self {
            brake_bias: 0.6,
            abs_strength: 1.0,
            traction_control_strength: 0.8,
            esc_strength: 0.8,
            drag_coefficient: 0.35,
            frontal_area: 2.0,
            rolling_resistance: 0.015,
            downforce_coefficient: 0.0,
            base_linear_damping: 0.02,
            linear_damping_per_speed: 0.002,
            base_angular_damping: 0.02,
            angular_damping_per_speed: 0.02,
        }
    }
}

/// Steering geometry and speed-assist parameters.
#[derive(Clone, Debug)]
pub struct SteeringConfig {
    /// Maximum central steering angle in radians.
    pub max_angle: Real,
    /// Speed where steering reaches its minimum multiplier.
    pub speed_sensitivity: Real,
    /// Steering multiplier retained at and above the sensitivity speed.
    pub minimum_speed_factor: Real,
    /// Whether velocity-based counter-steering assistance is enabled.
    pub assist: bool,
    /// Drift-correction strength in the range `0.0` through `1.0`.
    ///
    /// A value of `0.0` disables correction while `1.0` applies the full
    /// calculated correction whenever steering assistance is active.
    pub drift_correction: Real,
}

impl Default for SteeringConfig {
    fn default() -> Self {
        Self {
            max_angle: (35.0 as Real).to_radians(),
            speed_sensitivity: 35.0,
            minimum_speed_factor: 0.25,
            assist: false,
            drift_correction: 1.0,
        }
    }
}

/// Complete mandatory configuration for a ray-cast vehicle controller.
#[derive(Clone, Debug, Default)]
pub struct VehicleControllerConfig {
    /// Engine configuration.
    pub engine: EngineConfig,
    /// Transmission configuration.
    pub transmission: TransmissionConfig,
    /// Turbocharger configuration.
    pub turbo: TurboConfig,
    /// Chassis and driver-assistance configuration.
    pub dynamics: VehicleDynamicsConfig,
    /// Steering configuration.
    pub steering: SteeringConfig,
}

/// Normalized driver input for one vehicle update.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct VehicleInput {
    /// Forward accelerator input in the range 0 through 1.
    pub throttle: Real,
    /// Brake/reverse-pedal input in the range 0 through 1.
    pub brake: Real,
    /// Clutch-disengagement input in the range 0 through 1.
    pub clutch: Real,
    /// Rear-wheel handbrake input in the range 0 through 1.
    pub handbrake: Real,
    /// Steering input in the range -1 through 1.
    pub steering: Real,
}

/// Read-only runtime state produced by the vehicle controller.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct VehicleState {
    /// Current engine speed.
    pub engine_rpm: Real,
    /// Current gear, where -1 is reverse and 0 is neutral.
    pub current_gear: i32,
    /// Whether reverse gear is currently selected.
    pub reverse_direction: bool,
    /// Signed chassis speed along the configured forward axis.
    pub vehicle_speed: Real,
    /// Fastest driven-wheel surface speed.
    pub driven_wheel_speed: Real,
    /// Current central steering angle in radians.
    pub steering_angle: Real,
    /// Normalized engine load.
    pub engine_load: Real,
    /// Normalized proximity to or activity of the rev limiter.
    pub rev_limiter_amount: Real,
    /// Normalized turbocharger load.
    pub turbo_load: Real,
    /// Sequence incremented whenever a loaded turbo is released.
    pub turbo_release_sequence: u32,
    /// Number of wheels currently contacting the ground.
    pub wheels_in_contact: usize,
    /// Fraction of wheels currently receiving ABS intervention.
    pub abs_activity: Real,
    /// Normalized traction-control intervention estimate.
    pub traction_control_activity: Real,
    /// Normalized steering force-feedback output.
    pub force_feedback: Real,
    /// Normalized steering-wheel friction output.
    pub steering_friction: Real,
}

/// Logical axle used for brake and handbrake distribution.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum WheelAxle {
    /// Front axle.
    Front,
    /// Rear axle.
    Rear,
}

/// Permanent drivetrain and steering role assigned to one wheel.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct WheelRole {
    /// Axle used for brake distribution.
    pub axle: WheelAxle,
    /// Whether engine torque is delivered to this wheel.
    pub driven: bool,
    /// Whether Ackermann steering is applied to this wheel.
    pub steered: bool,
}

impl WheelRole {
    /// Creates a wheel role.
    pub const fn new(axle: WheelAxle, driven: bool, steered: bool) -> Self {
        Self {
            axle,
            driven,
            steered,
        }
    }
}

pub(crate) struct PowertrainOutput {
    pub drive_torque: Real,
    pub engine_brake_torque: Real,
    pub wheel_target_velocity: Real,
    pub service_brake: Real,
}

pub(crate) struct VehiclePowertrain {
    pub config: VehicleControllerConfig,
    input: VehicleInput,
    state: VehicleState,
    peak_torque: Real,
    shift_cooldown: Real,
    reverse_cooldown: Real,
    manual_override: Real,
    shift_target: i32,
    reverse_brake_armed: bool,
    turbo_load: Real,
    previous_throttle: Real,
}

impl VehiclePowertrain {
    pub fn new(mut config: VehicleControllerConfig) -> Self {
        sanitize_config(&mut config);
        let peak_torque = prepare_torque_curve(&mut config.engine);

        Self {
            state: VehicleState {
                engine_rpm: config.engine.idle_rpm,
                current_gear: 0,
                ..VehicleState::default()
            },
            config,
            input: VehicleInput::default(),
            peak_torque,
            shift_cooldown: 0.0,
            reverse_cooldown: 0.0,
            manual_override: 0.0,
            shift_target: 0,
            reverse_brake_armed: false,
            turbo_load: 0.0,
            previous_throttle: 0.0,
        }
    }

    pub fn set_input(&mut self, input: VehicleInput) {
        self.input = VehicleInput {
            throttle: input.throttle.clamp(0.0, 1.0),
            brake: input.brake.clamp(0.0, 1.0),
            clutch: input.clutch.clamp(0.0, 1.0),
            handbrake: input.handbrake.clamp(0.0, 1.0),
            steering: input.steering.clamp(-1.0, 1.0),
        };
    }

    pub fn input(&self) -> VehicleInput {
        self.input
    }

    pub fn state(&self) -> VehicleState {
        self.state
    }

    pub fn state_mut(&mut self) -> &mut VehicleState {
        &mut self.state
    }

    pub fn shift_up(&mut self) {
        if self.shift_cooldown > 0.0 {
            return;
        }

        let max_gear = self.config.transmission.forward_ratios.len() as i32;
        self.set_manual_shift_target((self.shift_target + 1).min(max_gear));
    }

    pub fn shift_down(&mut self) {
        if self.shift_cooldown > 0.0 {
            return;
        }

        self.set_manual_shift_target((self.shift_target - 1).max(-1));
    }

    pub fn set_gear(&mut self, gear: i32) {
        let max_gear = self.config.transmission.forward_ratios.len() as i32;
        self.set_manual_shift_target(gear.clamp(-1, max_gear));
    }

    fn set_manual_shift_target(&mut self, gear: i32) {
        self.state.reverse_direction = false;
        self.reverse_brake_armed = false;
        self.shift_target = gear;
        self.manual_override = TRANSMISSION_MANUAL_OVERRIDE;
    }

    pub fn update(
        &mut self,
        dt: Real,
        vehicle_speed: Real,
        driven_wheel_speed: Real,
        driven_wheel_radius: Real,
    ) -> PowertrainOutput {
        let dt = dt.clamp(0.0, 0.05);
        self.state.vehicle_speed = vehicle_speed;
        self.state.driven_wheel_speed = driven_wheel_speed;

        let (drive_throttle, service_brake) = self.effective_pedals();
        self.update_turbo(dt, drive_throttle);

        let ratio = self.current_ratio();
        let clutch_engagement = if ratio == 0.0 {
            0.0
        } else {
            1.0 - self.input.clutch
        };
        let available_torque = self.torque_at(self.state.engine_rpm);
        let rpm_span = (self.config.engine.max_rpm - self.config.engine.idle_rpm).max(1.0);
        let rpm_rate =
            ((self.state.engine_rpm - self.config.engine.idle_rpm) / rpm_span).clamp(0.0, 1.0);
        let boost = if self.config.turbo.enabled {
            1.0 + (self.config.turbo.max_boost - 1.0) * self.turbo_load
        } else {
            1.0
        };
        let friction_scale = self
            .config
            .engine
            .friction_torque
            .unwrap_or(self.peak_torque * 0.12);
        let pumping_loss = 2.5 + (1.0 - 2.5) * drive_throttle;
        let combustion_torque = available_torque * drive_throttle * boost;
        let friction_torque = friction_scale * (0.2 + 2.0 * rpm_rate * rpm_rate) * pumping_loss;
        let angular_acceleration =
            (combustion_torque - friction_torque) / self.config.engine.inertia;
        let rpm_acceleration = angular_acceleration * (60.0 / TAU);
        let mut next_rpm = self.state.engine_rpm + rpm_acceleration * dt;

        if clutch_engagement > 0.0 && ratio != 0.0 {
            let wheel_rpm =
                (driven_wheel_speed.abs() / (TAU * driven_wheel_radius.max(0.01))) * 60.0;
            let drivetrain_rpm =
                (wheel_rpm * ratio.abs() * self.config.transmission.final_drive_ratio)
                    .max(self.config.engine.idle_rpm);
            let coupling =
                1.0 - (-self.config.transmission.clutch_response * clutch_engagement * dt).exp();
            next_rpm += (drivetrain_rpm - next_rpm) * coupling;
        }

        let limit = self
            .config
            .engine
            .rev_limit_rpm
            .min(self.config.engine.max_rpm);
        self.state.engine_rpm = next_rpm.clamp(self.config.engine.idle_rpm, limit);
        let at_limit = self.state.engine_rpm >= limit - 0.5;
        self.state.rev_limiter_amount =
            ((self.state.engine_rpm - limit * 0.97) / (limit * 0.03).max(1.0)).clamp(0.0, 1.0);

        let available_torque = self.torque_at(self.state.engine_rpm);
        let gear_factor = ratio.abs().powf(self.config.engine.gear_force_exponent);
        let drive_torque = if at_limit {
            0.0
        } else {
            available_torque
                * drive_throttle
                * boost
                * gear_factor
                * self.config.transmission.final_drive_ratio
                * self.config.engine.drivetrain_efficiency
                * clutch_engagement
                * ratio.signum()
                * self.config.engine.force_scale
        };
        let engine_brake_torque = self.peak_torque
            * self.config.engine.engine_braking
            * rpm_rate
            * (1.0 - drive_throttle)
            * gear_factor
            * self.config.transmission.final_drive_ratio
            * clutch_engagement
            * self.config.engine.force_scale;
        let wheel_target_velocity = if ratio == 0.0 {
            0.0
        } else {
            (self.state.engine_rpm * TAU / 60.0)
                / (ratio * self.config.transmission.final_drive_ratio)
        };
        let gear_engaged = if ratio == 0.0 { 0.0 } else { clutch_engagement };
        let torque_load = ((available_torque / self.peak_torque) * boost).clamp(0.0, 1.5) / 1.5;
        let driven_load = drive_throttle * (0.35 + torque_load * 0.65) * gear_engaged;
        let free_rev_load = drive_throttle * (1.0 - gear_engaged) * 0.75;
        let engine_brake_load = rpm_rate
            * gear_engaged
            * (1.0 - drive_throttle)
            * (vehicle_speed.abs() / 8.0).clamp(0.0, 1.0);
        self.state.engine_load = (drive_throttle * 0.25 + driven_load + free_rev_load
            - engine_brake_load * 0.55)
            .clamp(0.0, 1.0);
        self.state.turbo_load = self.turbo_load;

        self.update_transmission(dt, vehicle_speed, driven_wheel_radius);

        PowertrainOutput {
            drive_torque,
            engine_brake_torque,
            wheel_target_velocity,
            service_brake,
        }
    }

    fn effective_pedals(&self) -> (Real, Real) {
        if self.state.reverse_direction {
            (self.input.brake, self.input.throttle)
        } else {
            (self.input.throttle, self.input.brake)
        }
    }

    fn update_transmission(&mut self, dt: Real, speed: Real, wheel_radius: Real) {
        self.shift_cooldown = (self.shift_cooldown - dt).max(0.0);
        self.reverse_cooldown = (self.reverse_cooldown - dt).max(0.0);
        self.manual_override = (self.manual_override - dt).max(0.0);

        if self.config.transmission.automatic && self.manual_override <= Real::EPSILON {
            self.update_automatic_transmission(speed, wheel_radius);
        }

        if self.shift_cooldown > Real::EPSILON || self.state.current_gear == self.shift_target {
            return;
        }

        self.state.current_gear = self.shift_target;
        if self.state.current_gear >= 0 {
            self.state.reverse_direction = false;
        }
        self.shift_cooldown = self.config.transmission.shift_cooldown;
    }

    fn update_automatic_transmission(&mut self, speed: Real, wheel_radius: Real) {
        if self.reverse_cooldown > Real::EPSILON {
            return;
        }

        let stopped = speed.abs() < self.config.transmission.stopped_speed;
        let throttle_pressed = self.input.throttle > TRANSMISSION_PEDAL_ENGAGE;
        let brake_pressed = self.input.brake > TRANSMISSION_PEDAL_ENGAGE;
        let brake_released = self.input.brake <= TRANSMISSION_PEDAL_RELEASE;

        if self.state.reverse_direction {
            self.reverse_brake_armed = false;
        } else if self.shift_target != 0 {
            self.reverse_brake_armed = false;
        } else if brake_released {
            self.reverse_brake_armed = true;
        }

        if self.shift_cooldown > Real::EPSILON {
            return;
        }

        if self.config.transmission.auto_reverse {
            if self.state.reverse_direction && stopped && throttle_pressed {
                self.state.reverse_direction = false;
                self.shift_target = 1;
                self.reverse_cooldown = self.config.transmission.shift_cooldown;
                self.reverse_brake_armed = false;
                return;
            }

            if !self.state.reverse_direction
                && self.shift_target == 0
                && self.reverse_brake_armed
                && brake_pressed
            {
                self.state.reverse_direction = true;
                self.shift_target = -1;
                self.reverse_cooldown = self.config.transmission.shift_cooldown;
                self.reverse_brake_armed = false;
                return;
            }
        }

        let drive_pedal = if self.state.reverse_direction {
            self.input.brake
        } else {
            self.input.throttle
        };

        if self.shift_target != 0 && stopped && drive_pedal <= TRANSMISSION_PEDAL_RELEASE {
            self.shift_target = 0;
            return;
        }

        if self.shift_target == 0 && drive_pedal > TRANSMISSION_PEDAL_ENGAGE {
            self.shift_target = if self.state.reverse_direction { -1 } else { 1 };
            return;
        }

        self.select_forward_gear(speed.abs(), wheel_radius);
    }

    fn select_forward_gear(&mut self, road_speed: Real, wheel_radius: Real) {
        let max_gear = self.config.transmission.forward_ratios.len() as i32;
        let mut target = if self.shift_target > 0 {
            self.shift_target
        } else {
            self.state.current_gear
        };

        if target <= 0 {
            return;
        }

        while target < max_gear
            && road_speed
                >= self.shift_speed(
                    target,
                    self.config.transmission.upshift_range_position,
                    wheel_radius,
                )
        {
            target += 1;
        }

        while target > 1
            && road_speed
                <= self.shift_speed(
                    target - 1,
                    self.config.transmission.downshift_range_position,
                    wheel_radius,
                )
        {
            target -= 1;
        }

        self.shift_target = target;
    }

    fn shift_speed(&self, lower_gear: i32, range_position: Real, wheel_radius: Real) -> Real {
        let lower_ratio = self.config.transmission.forward_ratios[(lower_gear - 1) as usize];
        let upper_ratio = self.config.transmission.forward_ratios[lower_gear as usize];
        let lower_max = self.rpm_to_speed(self.config.engine.max_rpm, lower_ratio, wheel_radius);
        let upper_min = self.rpm_to_speed(self.config.engine.idle_rpm, upper_ratio, wheel_radius);
        let overlap_start = lower_max.min(upper_min);
        let overlap_end = lower_max.max(upper_min);
        overlap_start + (overlap_end - overlap_start) * range_position
    }

    fn rpm_to_speed(&self, rpm: Real, ratio: Real, wheel_radius: Real) -> Real {
        rpm / (ratio.abs() * self.config.transmission.final_drive_ratio) / 60.0
            * TAU
            * wheel_radius.max(0.01)
    }

    fn update_turbo(&mut self, dt: Real, throttle: Real) {
        if !self.config.turbo.enabled {
            self.turbo_load = 0.0;
            self.previous_throttle = throttle;
            return;
        }

        if throttle > 0.05 {
            self.turbo_load =
                (self.turbo_load + throttle * self.config.turbo.spool_rate * dt).clamp(0.0, 1.0);
        } else {
            if self.previous_throttle > 0.2 && self.turbo_load > 0.2 {
                self.state.turbo_release_sequence =
                    self.state.turbo_release_sequence.wrapping_add(1);
            }
            self.turbo_load = (self.turbo_load - self.config.turbo.release_rate * dt).max(0.0);
        }
        self.previous_throttle = throttle;
    }

    fn current_ratio(&self) -> Real {
        match self.state.current_gear {
            -1 => self.config.transmission.reverse_ratio,
            0 => 0.0,
            gear => self
                .config
                .transmission
                .forward_ratios
                .get((gear - 1) as usize)
                .copied()
                .unwrap_or(0.0),
        }
    }

    fn torque_at(&self, rpm: Real) -> Real {
        let curve = &self.config.engine.torque_curve;
        if rpm <= curve[0].0 {
            return curve[0].1;
        }

        for pair in curve.windows(2) {
            if rpm <= pair[1].0 {
                let span = (pair[1].0 - pair[0].0).max(Real::EPSILON);
                let amount = (rpm - pair[0].0) / span;
                return pair[0].1 + (pair[1].1 - pair[0].1) * amount;
            }
        }

        curve[curve.len() - 1].1
    }
}

fn sanitize_config(config: &mut VehicleControllerConfig) {
    config.engine.idle_rpm = config.engine.idle_rpm.max(1.0);
    config.engine.max_rpm = config.engine.max_rpm.max(config.engine.idle_rpm + 1.0);
    config.engine.rev_limit_rpm = config
        .engine
        .rev_limit_rpm
        .clamp(config.engine.idle_rpm, config.engine.max_rpm);
    config.engine.inertia = config.engine.inertia.max(0.01);
    config.engine.drivetrain_efficiency = config.engine.drivetrain_efficiency.clamp(0.0, 1.0);
    config.transmission.final_drive_ratio = config.transmission.final_drive_ratio.abs().max(0.01);
    config.transmission.reverse_ratio = -config.transmission.reverse_ratio.abs().max(0.01);
    config.transmission.shift_cooldown = config.transmission.shift_cooldown.max(0.0);
    config.transmission.upshift_range_position =
        config.transmission.upshift_range_position.clamp(0.0, 1.0);
    config.transmission.downshift_range_position =
        config.transmission.downshift_range_position.clamp(0.0, 1.0);
    config.transmission.stopped_speed = config.transmission.stopped_speed.abs();
    config
        .transmission
        .forward_ratios
        .retain(|ratio| ratio.is_finite() && *ratio > 0.0);
    if config.transmission.forward_ratios.is_empty() {
        config.transmission.forward_ratios.push(1.0);
    }
    config.dynamics.brake_bias = config.dynamics.brake_bias.clamp(0.0, 1.0);
    config.dynamics.abs_strength = config.dynamics.abs_strength.clamp(0.0, 1.0);
    config.dynamics.traction_control_strength =
        config.dynamics.traction_control_strength.clamp(0.0, 1.0);
    config.dynamics.esc_strength = config.dynamics.esc_strength.clamp(0.0, 1.0);
    config.steering.max_angle = config.steering.max_angle.abs();
    config.steering.minimum_speed_factor = config.steering.minimum_speed_factor.clamp(0.0, 1.0);
    config.steering.drift_correction = config.steering.drift_correction.clamp(0.0, 1.0);
}

fn prepare_torque_curve(engine: &mut EngineConfig) -> Real {
    engine
        .torque_curve
        .retain(|(rpm, torque)| rpm.is_finite() && torque.is_finite() && *torque >= 0.0);
    engine.torque_curve.sort_by(|a, b| a.0.total_cmp(&b.0));
    engine
        .torque_curve
        .dedup_by(|a, b| (a.0 - b.0).abs() <= Real::EPSILON);

    if engine.torque_curve.is_empty() {
        let peak_rpm = engine.max_rpm * 0.65;
        let peak_torque = engine.horsepower.max(1.0) * 745.7 / (peak_rpm * TAU / 60.0);
        engine.torque_curve = vec![
            (engine.idle_rpm, peak_torque * 0.7),
            (peak_rpm, peak_torque),
            (engine.max_rpm, peak_torque * 0.75),
        ];
    }

    engine
        .torque_curve
        .iter()
        .map(|(_, torque)| *torque)
        .fold(Real::EPSILON, Real::max)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn creates_a_usable_fallback_torque_curve() {
        let powertrain = VehiclePowertrain::new(VehicleControllerConfig::default());
        assert!(powertrain.peak_torque > 0.0);
        assert_eq!(powertrain.config.engine.torque_curve.len(), 3);
    }

    #[test]
    fn interpolates_torque_curve() {
        let mut config = VehicleControllerConfig::default();
        config.engine.torque_curve = vec![(1000.0, 100.0), (3000.0, 200.0)];
        let powertrain = VehiclePowertrain::new(config);
        assert!((powertrain.torque_at(2000.0) - 150.0).abs() < 1.0e-4);
    }

    fn automatic_powertrain() -> VehiclePowertrain {
        let mut config = VehicleControllerConfig::default();
        config.transmission.shift_cooldown = 0.0;
        VehiclePowertrain::new(config)
    }

    #[test]
    fn automatic_transmission_starts_in_neutral_and_engages_from_the_drive_pedal() {
        let mut powertrain = automatic_powertrain();
        assert_eq!(powertrain.state().current_gear, 0);

        powertrain.set_input(VehicleInput {
            throttle: 1.0,
            ..VehicleInput::default()
        });
        powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
        assert_eq!(powertrain.state().current_gear, 1);

        powertrain.set_input(VehicleInput::default());
        powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
        assert_eq!(powertrain.state().current_gear, 0);
    }

    #[test]
    fn automatic_reverse_requires_neutral_brake_release_before_brake_press() {
        let mut powertrain = automatic_powertrain();
        powertrain.set_input(VehicleInput {
            brake: 1.0,
            ..VehicleInput::default()
        });
        powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
        assert_eq!(powertrain.state().current_gear, 0);

        powertrain.set_input(VehicleInput::default());
        powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);

        powertrain.set_input(VehicleInput {
            brake: 1.0,
            ..VehicleInput::default()
        });
        powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
        assert_eq!(powertrain.state().current_gear, -1);
        assert!(powertrain.state().reverse_direction);
        assert_eq!(powertrain.effective_pedals(), (1.0, 0.0));
    }

    #[test]
    fn throttle_returns_automatic_reverse_to_first_gear_when_stopped() {
        let mut powertrain = automatic_powertrain();
        powertrain.set_input(VehicleInput::default());
        powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
        powertrain.set_input(VehicleInput {
            brake: 1.0,
            ..VehicleInput::default()
        });
        powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
        assert_eq!(powertrain.state().current_gear, -1);

        powertrain.set_input(VehicleInput {
            throttle: 1.0,
            ..VehicleInput::default()
        });
        powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
        assert_eq!(powertrain.state().current_gear, 1);
        assert!(!powertrain.state().reverse_direction);
    }

    #[test]
    fn automatic_transmission_selects_gears_from_road_speed_ranges() {
        let mut config = VehicleControllerConfig::default();
        config.engine.idle_rpm = 1000.0;
        config.engine.max_rpm = 6000.0;
        config.transmission.forward_ratios = vec![4.0, 2.0, 1.0];
        config.transmission.final_drive_ratio = 4.0;
        config.transmission.shift_cooldown = 0.0;
        let mut powertrain = VehiclePowertrain::new(config);
        powertrain.set_input(VehicleInput {
            throttle: 1.0,
            ..VehicleInput::default()
        });

        powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.3);
        assert_eq!(powertrain.state().current_gear, 1);
        powertrain.update(1.0 / 60.0, 12.0, 0.0, 0.3);
        assert_eq!(powertrain.state().current_gear, 2);
        powertrain.update(1.0 / 60.0, 8.0, 0.0, 0.3);
        assert_eq!(powertrain.state().current_gear, 1);
    }

    #[test]
    fn manual_shift_temporarily_overrides_automatic_gear_selection() {
        let mut powertrain = automatic_powertrain();
        powertrain.set_input(VehicleInput {
            throttle: 1.0,
            ..VehicleInput::default()
        });
        powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
        assert_eq!(powertrain.state().current_gear, 1);

        powertrain.shift_up();
        powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
        assert_eq!(powertrain.state().current_gear, 2);

        for _ in 0..60 {
            powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
        }
        assert_eq!(powertrain.state().current_gear, 2);

        for _ in 0..31 {
            powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
        }
        assert_eq!(powertrain.state().current_gear, 1);
    }

    #[test]
    fn clamps_runtime_input() {
        let mut powertrain = VehiclePowertrain::new(VehicleControllerConfig::default());
        powertrain.set_input(VehicleInput {
            throttle: 2.0,
            brake: -1.0,
            clutch: 3.0,
            handbrake: -2.0,
            steering: 4.0,
        });
        assert_eq!(
            powertrain.input(),
            VehicleInput {
                throttle: 1.0,
                brake: 0.0,
                clutch: 1.0,
                handbrake: 0.0,
                steering: 1.0,
            }
        );
    }
}
