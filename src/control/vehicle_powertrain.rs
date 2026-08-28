use crate::math::Real;

const TAU: Real = 6.283_185_307_179_586 as Real;
const TRANSMISSION_PEDAL_ENGAGE: Real = 0.1;
const TRANSMISSION_PEDAL_RELEASE: Real = 0.05;
const TRANSMISSION_MANUAL_OVERRIDE: Real = 1.5;
const MANUAL_SHIFT_CLUTCH_DISENGAGEMENT: Real = 0.8;
const STALL_RPM_RATIO: Real = 0.55;
const STALL_PROTECTION_RPM_MARGIN: Real = 1.0;
const ENGINE_START_DURATION: Real = 1.0;
const ENGINE_START_TIME_EPSILON: Real = 1.0e-5;
const ENGINE_START_CRANK_END: Real = 0.5;
const ENGINE_START_CATCH_END: Real = 0.72;
const ENGINE_START_CRANK_RPM_RATIO: Real = 0.35;
const ENGINE_START_FLARE_RPM_RATIO: Real = 1.8;
const CLUTCH_CAPACITY_MULTIPLIER: Real = 2.0;
const AUTO_BLIP_CLUTCH_DISENGAGE_DURATION: Real = 0.05;
const AUTO_BLIP_CLUTCH_REENGAGE_DURATION: Real = 0.12;
const AUTO_BLIP_RPM_TOLERANCE: Real = 75.0;
const AUTO_BLIP_THROTTLE: Real = 1.0;
const AUTO_BLIP_MIN_RPM_GAP: Real = 100.0;
const AUTO_BLIP_OVERSHOOT_FACTOR: Real = 0.1;
const AUTO_CLUTCH_LAUNCH_RPM_RATIO: Real = 0.7;
const AUTO_CLUTCH_ANTISTALL_ENTER_RPM_RATIO: Real = 0.6;
const AUTO_CLUTCH_ANTISTALL_EXIT_RPM_RATIO: Real = 0.65;
const AUTO_CLUTCH_ANTISTALL_RELEASE_RESPONSE_MULTIPLIER: Real = 2.0;
const AUTO_CLUTCH_RPM_CONTROL_FRACTION: Real = 0.35;
const AUTO_CLUTCH_RPM_CONTROL_BAND_FRACTION: Real = 0.2;
const AUTO_CLUTCH_MAX_SLIP_ENGAGEMENT: Real = 0.75;
const AUTO_CLUTCH_LOCK_RPM_TOLERANCE: Real = 5.0;
const AUTO_CLUTCH_DISENGAGED_EPSILON: Real = 0.01;

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
    /// Whether anti-stall clutch management is enabled for a manual transmission.
    ///
    /// Automatic transmissions always use clutch management regardless of this setting.
    pub auto_clutch: bool,
    /// Whether stopped pedal input automatically selects forward or reverse.
    pub auto_reverse: bool,
    /// Rate at which clutch engagement couples engine and wheel RPM.
    pub clutch_response: Real,
    /// Minimum time between gear changes.
    pub shift_cooldown: Real,
    /// Whether sequential and automatic downshifts perform a clutch-assisted rev match.
    pub auto_blip: bool,
    /// Maximum time allowed for open-clutch RPM matching, in seconds.
    pub auto_blip_duration: Real,
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
            auto_clutch: false,
            auto_reverse: true,
            clutch_response: 12.0,
            shift_cooldown: 0.73,
            auto_blip: true,
            auto_blip_duration: 0.2,
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
    /// Linear-to-cubic driver steering blend applied before steering assistance.
    ///
    /// A value of `0.0` is linear while `1.0` is fully cubic.
    pub road_wheel_curve: Real,
    /// Speed where assisted steering reaches its minimum multiplier.
    pub speed_sensitivity: Real,
    /// Assisted steering multiplier retained at and above the sensitivity speed.
    pub minimum_speed_factor: Real,
    /// Whether speed-sensitive range reduction and counter-steering are enabled.
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
            road_wheel_curve: 0.0,
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
    /// Whether the engine is currently producing combustion torque.
    pub engine_running: bool,
    /// Whether the electric starter is currently cranking the engine.
    pub engine_starting: bool,
    /// Normalized progress through the active starter sequence.
    pub engine_start_progress: Real,
    /// Sequence incremented whenever the engine lifecycle state changes.
    pub engine_state_sequence: u32,
    /// Sequence incremented for every driver-requested shift.
    pub gear_shift_sequence: u32,
    /// Sequence incremented whenever a driver-requested shift is accepted.
    pub gear_shift_accepted_sequence: u32,
    /// Sequence incremented whenever a driver-requested shift is ignored.
    pub gear_shift_ignored_sequence: u32,
    /// Sequence incremented whenever a driver-requested shift is rejected by the clutch gate.
    pub gear_shift_rejected_sequence: u32,
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
    /// Uncurved, speed-adjusted driver steering angle in road-wheel radians.
    pub driver_steering_angle: Real,
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
    /// Normalized electronic stability-control intervention.
    pub esc_activity: Real,
    /// Normalized traction-control intervention estimate.
    pub traction_control_activity: Real,
    /// Normalized steering force-feedback output.
    pub force_feedback: Real,
    /// Normalized steering-wheel friction output.
    pub steering_friction: Real,
}

/// Result of a driver-requested gear change.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum VehicleShiftOutcome {
    /// The request was accepted by the transmission controller.
    Accepted,
    /// The request was ignored because another shift is active or cooling down.
    Ignored,
    /// The request failed because a manual clutch was not sufficiently disengaged.
    ClutchRejected,
}

/// Discrete lifecycle state of the combustion engine.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum VehicleEngineState {
    /// The engine is not producing torque and the starter is inactive.
    Stopped,
    /// The starter sequence is actively cranking the engine.
    Starting,
    /// The engine is producing combustion torque.
    Running,
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
    pub wheel_coupling_torque: Real,
    pub wheel_target_velocity: Real,
    pub wheel_limit_velocity: Real,
    pub drive_throttle: Real,
    pub drivetrain_connected: bool,
    pub service_brake: Real,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum ShiftPhase {
    #[default]
    Idle,
    Disengaging,
    Blipping,
    Settling,
    Reengaging,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum AutomaticClutchPhase {
    #[default]
    Open,
    Launch,
    AntiStall,
    Locked,
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
    shift_target_allows_blip: bool,
    shift_phase: ShiftPhase,
    shift_phase_timer: Real,
    shift_to: i32,
    shift_overshoot_rpm: Real,
    reverse_brake_armed: bool,
    turbo_load: Real,
    previous_throttle: Real,
    restart_armed: bool,
    engine_start_elapsed: Real,
    automatic_clutch_engagement: Real,
    automatic_clutch_phase: AutomaticClutchPhase,
}

impl VehiclePowertrain {
    pub fn new(mut config: VehicleControllerConfig) -> Self {
        sanitize_config(&mut config);
        let peak_torque = prepare_torque_curve(&mut config.engine);

        Self {
            state: VehicleState {
                engine_rpm: config.engine.idle_rpm,
                engine_running: true,
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
            shift_target_allows_blip: false,
            shift_phase: ShiftPhase::Idle,
            shift_phase_timer: 0.0,
            shift_to: 0,
            shift_overshoot_rpm: 0.0,
            reverse_brake_armed: false,
            turbo_load: 0.0,
            previous_throttle: 0.0,
            restart_armed: true,
            engine_start_elapsed: 0.0,
            automatic_clutch_engagement: 0.0,
            automatic_clutch_phase: AutomaticClutchPhase::Open,
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

    pub fn reset(&mut self) {
        self.input = VehicleInput::default();
        self.state = VehicleState {
            engine_rpm: self.config.engine.idle_rpm,
            engine_running: true,
            current_gear: 0,
            ..VehicleState::default()
        };
        self.shift_cooldown = 0.0;
        self.reverse_cooldown = 0.0;
        self.manual_override = 0.0;
        self.shift_target = 0;
        self.shift_target_allows_blip = false;
        self.shift_phase = ShiftPhase::Idle;
        self.shift_phase_timer = 0.0;
        self.shift_to = 0;
        self.shift_overshoot_rpm = 0.0;
        self.reverse_brake_armed = false;
        self.turbo_load = 0.0;
        self.previous_throttle = 0.0;
        self.restart_armed = true;
        self.engine_start_elapsed = 0.0;
        self.automatic_clutch_engagement = 0.0;
        self.automatic_clutch_phase = AutomaticClutchPhase::Open;
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

    pub fn shift_up(&mut self) -> VehicleShiftOutcome {
        if self.shift_phase != ShiftPhase::Idle {
            return self.record_shift_outcome(VehicleShiftOutcome::Ignored);
        }

        let max_gear = self.config.transmission.forward_ratios.len() as i32;
        let gear = (self.shift_target + 1).min(max_gear);
        if self.reject_manual_shift_without_clutch(gear) {
            return self.record_shift_outcome(VehicleShiftOutcome::ClutchRejected);
        }

        if self.shift_cooldown > 0.0 {
            return self.record_shift_outcome(VehicleShiftOutcome::Ignored);
        }

        self.set_manual_shift_target(gear, false);
        self.record_shift_outcome(VehicleShiftOutcome::Accepted)
    }

    pub fn shift_down(&mut self) -> VehicleShiftOutcome {
        if self.shift_phase != ShiftPhase::Idle {
            return self.record_shift_outcome(VehicleShiftOutcome::Ignored);
        }

        let gear = (self.shift_target - 1).max(-1);
        if self.reject_manual_shift_without_clutch(gear) {
            return self.record_shift_outcome(VehicleShiftOutcome::ClutchRejected);
        }

        if self.shift_cooldown > 0.0 {
            return self.record_shift_outcome(VehicleShiftOutcome::Ignored);
        }

        self.set_manual_shift_target(gear, true);
        self.record_shift_outcome(VehicleShiftOutcome::Accepted)
    }

    pub fn set_gear(&mut self, gear: i32) -> VehicleShiftOutcome {
        let max_gear = self.config.transmission.forward_ratios.len() as i32;
        let gear = gear.clamp(-1, max_gear);
        if self.reject_manual_shift_without_clutch(gear) {
            return self.record_shift_outcome(VehicleShiftOutcome::ClutchRejected);
        }

        self.set_manual_shift_target(gear, false);
        self.record_shift_outcome(VehicleShiftOutcome::Accepted)
    }

    fn reject_manual_shift_without_clutch(&mut self, gear: i32) -> bool {
        let requires_disengaged_clutch = !self.uses_automatic_clutch()
            && gear != 0
            && gear != self.state.current_gear
            && self.input.clutch < MANUAL_SHIFT_CLUTCH_DISENGAGEMENT;
        if !requires_disengaged_clutch {
            return false;
        }

        self.set_manual_shift_target(0, false);
        self.shift_cooldown = 0.0;
        true
    }

    fn record_shift_outcome(&mut self, outcome: VehicleShiftOutcome) -> VehicleShiftOutcome {
        self.state.gear_shift_sequence = self.state.gear_shift_sequence.wrapping_add(1);
        let sequence = match outcome {
            VehicleShiftOutcome::Accepted => &mut self.state.gear_shift_accepted_sequence,
            VehicleShiftOutcome::Ignored => &mut self.state.gear_shift_ignored_sequence,
            VehicleShiftOutcome::ClutchRejected => &mut self.state.gear_shift_rejected_sequence,
        };
        *sequence = sequence.wrapping_add(1);
        outcome
    }

    pub fn engine_state(&self) -> VehicleEngineState {
        if self.state.engine_starting {
            VehicleEngineState::Starting
        } else if self.state.engine_running {
            VehicleEngineState::Running
        } else {
            VehicleEngineState::Stopped
        }
    }

    fn set_manual_shift_target(&mut self, gear: i32, allows_blip: bool) {
        self.state.reverse_direction = false;
        self.reverse_brake_armed = false;
        self.shift_target = gear;
        self.shift_target_allows_blip = allows_blip;
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
        let previous_engine_state = self.engine_state();
        self.state.vehicle_speed = vehicle_speed;
        self.state.driven_wheel_speed = driven_wheel_speed.abs();
        self.update_shift_sequence(dt, driven_wheel_speed.abs(), driven_wheel_radius);

        let (requested_drive_throttle, service_brake) = self.effective_pedals();
        self.update_engine_start_state(dt, requested_drive_throttle);
        let mut drive_throttle = if self.state.engine_starting {
            0.0
        } else {
            requested_drive_throttle
        };
        if !self.state.engine_starting {
            drive_throttle = drive_throttle.max(self.shift_throttle_override());
        }
        if self.state.engine_running {
            self.update_turbo(dt, drive_throttle);
        } else {
            self.turbo_load = 0.0;
            self.previous_throttle = drive_throttle;
        }

        let ratio = self.current_ratio();
        let signed_wheel_rpm = (driven_wheel_speed / (TAU * driven_wheel_radius.max(0.01))) * 60.0;
        let signed_drivetrain_rpm =
            signed_wheel_rpm * ratio * self.config.transmission.final_drive_ratio;
        let available_torque = self.torque_at(self.state.engine_rpm);
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
        let rpm_span = (self.config.engine.max_rpm - self.config.engine.idle_rpm).max(1.0);
        let rpm_rate =
            ((self.state.engine_rpm - self.config.engine.idle_rpm) / rpm_span).clamp(0.0, 1.0);
        let pumping_loss = 2.5 + (1.0 - 2.5) * drive_throttle;
        let idle_combustion_torque = friction_scale * 0.2 * 2.5;
        let combustion_torque = if self.state.engine_running {
            idle_combustion_torque * (1.0 - drive_throttle)
                + available_torque * drive_throttle * boost
        } else {
            0.0
        };
        let friction_torque = if self.state.engine_running {
            let friction_curve = 0.2 + 1.2 * rpm_rate + 0.8 * rpm_rate * rpm_rate;
            friction_scale * friction_curve * pumping_loss
        } else {
            0.0
        };
        let engine_braking_torque = self.peak_torque
            * self.config.engine.engine_braking
            * rpm_rate
            * (1.0 - drive_throttle);
        let free_engine_torque = combustion_torque - friction_torque;
        let drivetrain_engine_torque = if self.state.engine_running {
            combustion_torque - engine_braking_torque
        } else {
            0.0
        };
        let clutch_engagement = self.clutch_engagement(
            dt,
            ratio,
            vehicle_speed,
            signed_drivetrain_rpm,
            driven_wheel_radius,
            drive_throttle,
            service_brake,
            drivetrain_engine_torque,
        );

        let drivetrain_connected = clutch_engagement > 0.0 && ratio != 0.0;
        let engine_source_torque = if drivetrain_connected {
            drivetrain_engine_torque
        } else {
            free_engine_torque
        };
        let clutch_torque = self.clutch_torque(
            dt,
            signed_drivetrain_rpm,
            clutch_engagement,
            engine_source_torque,
        );
        let angular_acceleration =
            (engine_source_torque - clutch_torque) / self.config.engine.inertia;
        let rpm_acceleration = angular_acceleration * (60.0 / TAU);
        let mut next_rpm = self.state.engine_rpm + rpm_acceleration * dt;

        let bump_cranking =
            !self.state.engine_running && drivetrain_connected && signed_drivetrain_rpm > 0.0;
        if self.state.engine_running && !drivetrain_connected {
            next_rpm = next_rpm.max(self.config.engine.idle_rpm);
        }

        let limit = self
            .config
            .engine
            .rev_limit_rpm
            .min(self.config.engine.max_rpm);
        let stall_rpm = self.config.engine.idle_rpm * STALL_RPM_RATIO;
        if self.state.engine_starting {
            // The starter owns engine RPM and the clutch stays open until ignition completes.
        } else if self.state.engine_running && drivetrain_connected && next_rpm < stall_rpm {
            self.state.engine_running = false;
            self.state.engine_rpm = 0.0;
            self.turbo_load = 0.0;
            self.restart_armed = drive_throttle <= TRANSMISSION_PEDAL_RELEASE;
        } else if self.state.engine_running {
            self.state.engine_rpm = next_rpm.clamp(0.0, limit);
        } else if bump_cranking {
            self.state.engine_rpm = next_rpm.clamp(0.0, limit);
            if self.state.engine_rpm >= stall_rpm {
                self.state.engine_running = true;
                self.restart_armed = drive_throttle <= TRANSMISSION_PEDAL_RELEASE;
            }
        } else {
            self.state.engine_rpm = 0.0;
        }

        let at_limit = self.state.engine_rpm >= limit - 0.5;
        self.state.rev_limiter_amount = if self.state.engine_running {
            ((self.state.engine_rpm - limit * 0.97) / (limit * 0.03).max(1.0)).clamp(0.0, 1.0)
        } else {
            0.0
        };

        let available_torque = self.torque_at(self.state.engine_rpm);
        let rpm_rate =
            ((self.state.engine_rpm - self.config.engine.idle_rpm) / rpm_span).clamp(0.0, 1.0);
        let gear_factor = ratio.abs().powf(self.config.engine.gear_force_exponent);
        let mechanical_wheel_torque_scale = gear_factor
            * self.config.transmission.final_drive_ratio
            * self.config.engine.drivetrain_efficiency;
        let wheel_torque_scale = mechanical_wheel_torque_scale * self.config.engine.force_scale;
        let wheel_coupling_torque = if clutch_torque < 0.0
            || (self.state.engine_running && !at_limit && clutch_torque > 0.0)
        {
            clutch_torque * mechanical_wheel_torque_scale * ratio.signum()
        } else {
            0.0
        };
        let drive_torque = if self.state.engine_running && !at_limit && clutch_torque > 0.0 {
            clutch_torque * wheel_torque_scale * ratio.signum()
        } else {
            0.0
        };
        let engine_brake_torque = if clutch_torque < 0.0 {
            -clutch_torque * wheel_torque_scale
        } else {
            0.0
        };
        let wheel_target_velocity = if !self.state.engine_running || ratio == 0.0 {
            0.0
        } else {
            (self.state.engine_rpm * TAU / 60.0)
                / (ratio * self.config.transmission.final_drive_ratio)
        };
        let wheel_limit_velocity = if ratio == 0.0 {
            0.0
        } else {
            (limit * TAU / 60.0) / (ratio * self.config.transmission.final_drive_ratio)
        };
        let gear_engaged = if ratio == 0.0 { 0.0 } else { clutch_engagement };
        let torque_load = ((available_torque / self.peak_torque) * boost).clamp(0.0, 1.5) / 1.5;
        let combustion_load =
            (combustion_torque / (self.peak_torque * boost).max(Real::EPSILON)).clamp(0.0, 1.0);
        let driven_load = combustion_load * (0.35 + torque_load * 0.65) * gear_engaged;
        let free_rev_load = combustion_load * (1.0 - gear_engaged) * 0.75;
        let engine_brake_load = rpm_rate
            * gear_engaged
            * (1.0 - drive_throttle)
            * (vehicle_speed.abs() / 8.0).clamp(0.0, 1.0);
        self.state.engine_load = if self.state.engine_running {
            (combustion_load * 0.25 + driven_load + free_rev_load - engine_brake_load * 0.55)
                .clamp(0.0, 1.0)
        } else {
            0.0
        };
        self.state.turbo_load = self.turbo_load;

        self.update_transmission(
            dt,
            vehicle_speed,
            driven_wheel_speed.abs(),
            driven_wheel_radius,
        );
        if self.engine_state() != previous_engine_state {
            self.state.engine_state_sequence = self.state.engine_state_sequence.wrapping_add(1);
        }

        PowertrainOutput {
            drive_torque,
            engine_brake_torque,
            wheel_coupling_torque,
            wheel_target_velocity,
            wheel_limit_velocity,
            drive_throttle,
            drivetrain_connected,
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

    fn shift_clutch_override(&self) -> Real {
        match self.shift_phase {
            ShiftPhase::Idle => 0.0,
            ShiftPhase::Disengaging => {
                1.0 - self.shift_phase_timer / AUTO_BLIP_CLUTCH_DISENGAGE_DURATION
            }
            ShiftPhase::Blipping => 1.0,
            ShiftPhase::Settling => 1.0,
            ShiftPhase::Reengaging => self.shift_phase_timer / AUTO_BLIP_CLUTCH_REENGAGE_DURATION,
        }
        .clamp(0.0, 1.0)
    }

    fn shift_throttle_override(&self) -> Real {
        if self.shift_phase == ShiftPhase::Blipping {
            AUTO_BLIP_THROTTLE
        } else {
            0.0
        }
    }

    fn clutch_torque(
        &self,
        dt: Real,
        signed_drivetrain_rpm: Real,
        clutch_engagement: Real,
        engine_source_torque: Real,
    ) -> Real {
        if clutch_engagement <= Real::EPSILON {
            return 0.0;
        }

        let engine_angular_velocity = self.state.engine_rpm * TAU / 60.0;
        let drivetrain_angular_velocity = signed_drivetrain_rpm * TAU / 60.0;
        if !self.state.engine_running && drivetrain_angular_velocity <= 0.0 {
            return 0.0;
        }

        let response = self.config.transmission.clutch_response;
        let synchronization_rate = if dt <= Real::EPSILON || response <= Real::EPSILON {
            0.0
        } else {
            (1.0 - (-response * dt).exp()) / dt
        };
        let slip = engine_angular_velocity - drivetrain_angular_velocity;
        let synchronization_torque = self.config.engine.inertia * slip * synchronization_rate;
        let capacity = self.peak_torque * CLUTCH_CAPACITY_MULTIPLIER * clutch_engagement;
        let mut torque = (engine_source_torque + synchronization_torque).clamp(-capacity, capacity);

        let stall_protection_rpm =
            self.config.engine.idle_rpm * STALL_RPM_RATIO + STALL_PROTECTION_RPM_MARGIN;
        if self.uses_automatic_clutch() && self.state.engine_running && torque > 0.0 {
            let stall_angular_velocity = stall_protection_rpm * TAU / 60.0;
            let available_load = engine_source_torque
                + self.config.engine.inertia * (engine_angular_velocity - stall_angular_velocity)
                    / dt.max(Real::EPSILON);
            torque = torque.min(available_load.max(0.0));
        }

        torque
    }

    fn uses_automatic_clutch(&self) -> bool {
        self.config.transmission.automatic || self.config.transmission.auto_clutch
    }

    fn clutch_engagement(
        &mut self,
        dt: Real,
        ratio: Real,
        vehicle_speed: Real,
        signed_drivetrain_rpm: Real,
        driven_wheel_radius: Real,
        drive_throttle: Real,
        service_brake: Real,
        drivetrain_engine_torque: Real,
    ) -> Real {
        if self.state.engine_starting {
            self.automatic_clutch_engagement = 0.0;
            self.automatic_clutch_phase = AutomaticClutchPhase::Open;
            return 0.0;
        }

        if ratio == 0.0 {
            self.automatic_clutch_engagement = 0.0;
            self.automatic_clutch_phase = AutomaticClutchPhase::Open;
            return 0.0;
        }

        let pedal_engagement = 1.0 - self.input.clutch.max(self.shift_clutch_override());
        if !self.uses_automatic_clutch() {
            return pedal_engagement.clamp(0.0, 1.0);
        }

        let ground_wheel_rpm = (vehicle_speed / (TAU * driven_wheel_radius.max(0.01))) * 60.0;
        let ground_drivetrain_rpm =
            ground_wheel_rpm * ratio * self.config.transmission.final_drive_ratio;
        let target_rpm = self.automatic_clutch_target_rpm();
        self.automatic_clutch_phase = self.next_automatic_clutch_phase(
            signed_drivetrain_rpm,
            ground_drivetrain_rpm,
            target_rpm,
            drive_throttle,
            service_brake,
        );

        let target = match self.automatic_clutch_phase {
            AutomaticClutchPhase::Open => 0.0,
            AutomaticClutchPhase::Launch => self.automatic_clutch_launch_engagement(
                target_rpm,
                ground_drivetrain_rpm,
                drivetrain_engine_torque,
            ),
            AutomaticClutchPhase::AntiStall if drive_throttle > TRANSMISSION_PEDAL_ENGAGE => self
                .automatic_clutch_launch_engagement(
                    target_rpm,
                    ground_drivetrain_rpm,
                    drivetrain_engine_torque,
                ),
            AutomaticClutchPhase::AntiStall => 0.0,
            AutomaticClutchPhase::Locked => 1.0,
        };
        let response = if self.automatic_clutch_phase == AutomaticClutchPhase::AntiStall {
            self.config.transmission.clutch_response
                * AUTO_CLUTCH_ANTISTALL_RELEASE_RESPONSE_MULTIPLIER
        } else {
            self.config.transmission.clutch_response
        };
        let blend = if response <= Real::EPSILON {
            1.0
        } else {
            1.0 - (-response * dt).exp()
        };
        if self.automatic_clutch_phase == AutomaticClutchPhase::Locked {
            self.automatic_clutch_engagement = 1.0;
        } else {
            self.automatic_clutch_engagement += (target - self.automatic_clutch_engagement) * blend;
        }
        self.automatic_clutch_engagement =
            self.automatic_clutch_engagement
                .min(self.automatic_clutch_stall_limit(
                    dt,
                    signed_drivetrain_rpm,
                    drivetrain_engine_torque,
                ));

        if matches!(
            self.automatic_clutch_phase,
            AutomaticClutchPhase::Open | AutomaticClutchPhase::AntiStall
        ) && self.automatic_clutch_engagement <= AUTO_CLUTCH_DISENGAGED_EPSILON
        {
            self.automatic_clutch_engagement = 0.0;
        }

        (self.automatic_clutch_engagement * pedal_engagement).clamp(0.0, 1.0)
    }

    fn next_automatic_clutch_phase(
        &self,
        signed_drivetrain_rpm: Real,
        ground_drivetrain_rpm: Real,
        target_rpm: Real,
        drive_throttle: Real,
        service_brake: Real,
    ) -> AutomaticClutchPhase {
        if !self.state.engine_running {
            return if signed_drivetrain_rpm > 0.0 {
                AutomaticClutchPhase::Locked
            } else {
                AutomaticClutchPhase::Open
            };
        }

        let anti_stall_enter_rpm =
            self.config.engine.idle_rpm * AUTO_CLUTCH_ANTISTALL_ENTER_RPM_RATIO;
        let anti_stall_exit_rpm =
            self.config.engine.idle_rpm * AUTO_CLUTCH_ANTISTALL_EXIT_RPM_RATIO;
        if self.state.engine_rpm <= anti_stall_enter_rpm
            || self.automatic_clutch_phase == AutomaticClutchPhase::AntiStall
                && self.state.engine_rpm < anti_stall_exit_rpm
        {
            return AutomaticClutchPhase::AntiStall;
        }
        let driven_wheels_stopped_while_braking = (service_brake > Real::EPSILON
            || self.input.handbrake > Real::EPSILON)
            && signed_drivetrain_rpm < anti_stall_enter_rpm
            && ground_drivetrain_rpm >= target_rpm - AUTO_CLUTCH_LOCK_RPM_TOLERANCE;
        if driven_wheels_stopped_while_braking {
            return AutomaticClutchPhase::AntiStall;
        }
        if ground_drivetrain_rpm >= target_rpm - AUTO_CLUTCH_LOCK_RPM_TOLERANCE {
            return AutomaticClutchPhase::Locked;
        }
        if drive_throttle > TRANSMISSION_PEDAL_ENGAGE {
            return AutomaticClutchPhase::Launch;
        }
        if ground_drivetrain_rpm < anti_stall_enter_rpm {
            AutomaticClutchPhase::AntiStall
        } else {
            AutomaticClutchPhase::Open
        }
    }

    fn automatic_clutch_target_rpm(&self) -> Real {
        self.config.engine.idle_rpm * AUTO_CLUTCH_LAUNCH_RPM_RATIO
    }

    fn automatic_clutch_launch_engagement(
        &self,
        target_rpm: Real,
        ground_drivetrain_rpm: Real,
        drivetrain_engine_torque: Real,
    ) -> Real {
        let feed_forward = drivetrain_engine_torque.max(0.0)
            / (self.peak_torque * CLUTCH_CAPACITY_MULTIPLIER).max(Real::EPSILON);
        let control_band = (target_rpm * AUTO_CLUTCH_RPM_CONTROL_BAND_FRACTION).max(100.0);
        let rpm_error = (self.state.engine_rpm - target_rpm) / control_band;
        let rpm_control = rpm_error * AUTO_CLUTCH_RPM_CONTROL_FRACTION;
        let bite_floor = feed_forward * 0.2;
        let lock_progress = (ground_drivetrain_rpm / target_rpm.max(1.0)).clamp(0.0, 1.0);
        let lock_floor = lock_progress * lock_progress * lock_progress;

        (feed_forward + rpm_control)
            .clamp(bite_floor, AUTO_CLUTCH_MAX_SLIP_ENGAGEMENT)
            .max(lock_floor)
            .clamp(0.0, 1.0)
    }

    fn automatic_clutch_stall_limit(
        &self,
        dt: Real,
        signed_drivetrain_rpm: Real,
        drivetrain_engine_torque: Real,
    ) -> Real {
        if !self.state.engine_running
            || dt <= Real::EPSILON
            || signed_drivetrain_rpm >= self.state.engine_rpm
        {
            return 1.0;
        }

        let stall_rpm = self.config.engine.idle_rpm * STALL_RPM_RATIO + STALL_PROTECTION_RPM_MARGIN;
        let available_load = drivetrain_engine_torque
            + self.config.engine.inertia * (self.state.engine_rpm - stall_rpm).max(0.0) * TAU
                / (60.0 * dt);
        let capacity = self.peak_torque * CLUTCH_CAPACITY_MULTIPLIER;
        (available_load / capacity.max(Real::EPSILON)).clamp(0.0, 1.0)
    }

    fn update_engine_start_state(&mut self, dt: Real, drive_throttle: Real) {
        if self.state.engine_running {
            self.state.engine_starting = false;
            self.state.engine_start_progress = 0.0;
            self.engine_start_elapsed = 0.0;
            self.restart_armed = drive_throttle <= TRANSMISSION_PEDAL_RELEASE;
            return;
        }

        if self.state.engine_starting {
            self.engine_start_elapsed = (self.engine_start_elapsed + dt).min(ENGINE_START_DURATION);
            self.state.engine_start_progress = self.engine_start_elapsed / ENGINE_START_DURATION;
            self.state.engine_rpm = self.engine_start_rpm(self.state.engine_start_progress);

            if self.engine_start_elapsed >= ENGINE_START_DURATION - ENGINE_START_TIME_EPSILON {
                self.state.engine_running = true;
                self.state.engine_starting = false;
                self.state.engine_start_progress = 0.0;
                self.state.engine_rpm = self.config.engine.idle_rpm;
                self.engine_start_elapsed = 0.0;
            }
            return;
        }

        if drive_throttle <= TRANSMISSION_PEDAL_RELEASE {
            self.restart_armed = true;
        } else if self.restart_armed && drive_throttle > TRANSMISSION_PEDAL_ENGAGE {
            self.state.engine_starting = true;
            self.state.engine_start_progress = 0.0;
            self.state.engine_rpm = 0.0;
            self.engine_start_elapsed = 0.0;
            self.restart_armed = false;
        }
    }

    fn engine_start_rpm(&self, progress: Real) -> Real {
        let smooth_step = |value: Real| value * value * (3.0 - 2.0 * value);
        let idle_rpm = self.config.engine.idle_rpm;
        let crank_rpm = idle_rpm * ENGINE_START_CRANK_RPM_RATIO;
        let flare_rpm = (idle_rpm * ENGINE_START_FLARE_RPM_RATIO)
            .min(self.config.engine.max_rpm * 0.75)
            .max(idle_rpm);

        if progress <= ENGINE_START_CRANK_END {
            let amount = smooth_step(progress / ENGINE_START_CRANK_END);
            crank_rpm * amount
        } else if progress <= ENGINE_START_CATCH_END {
            let amount = smooth_step(
                (progress - ENGINE_START_CRANK_END)
                    / (ENGINE_START_CATCH_END - ENGINE_START_CRANK_END),
            );
            crank_rpm + (flare_rpm - crank_rpm) * amount
        } else {
            let amount =
                smooth_step((progress - ENGINE_START_CATCH_END) / (1.0 - ENGINE_START_CATCH_END));
            flare_rpm + (idle_rpm - flare_rpm) * amount
        }
    }

    fn update_transmission(
        &mut self,
        dt: Real,
        vehicle_speed: Real,
        driven_wheel_speed: Real,
        wheel_radius: Real,
    ) {
        self.shift_cooldown = (self.shift_cooldown - dt).max(0.0);
        self.reverse_cooldown = (self.reverse_cooldown - dt).max(0.0);
        self.manual_override = (self.manual_override - dt).max(0.0);

        if self.shift_phase != ShiftPhase::Idle {
            return;
        }

        if self.config.transmission.automatic && self.manual_override <= Real::EPSILON {
            self.update_automatic_transmission(vehicle_speed, wheel_radius);
        }

        if self.shift_cooldown > Real::EPSILON || self.state.current_gear == self.shift_target {
            return;
        }

        if self.start_auto_blip_shift(driven_wheel_speed, wheel_radius) {
            return;
        }

        self.state.current_gear = self.shift_target;
        if self.state.current_gear >= 0 {
            self.state.reverse_direction = false;
        }
        self.shift_cooldown = self.config.transmission.shift_cooldown;
        self.shift_target_allows_blip = false;
    }

    fn start_auto_blip_shift(&mut self, speed: Real, wheel_radius: Real) -> bool {
        if !self.config.transmission.auto_blip
            || !self.shift_target_allows_blip
            || !self.state.engine_running
            || self.config.transmission.auto_blip_duration <= Real::EPSILON
        {
            return false;
        }

        let old_gear = self.state.current_gear;
        let new_gear = self.shift_target;
        if old_gear <= new_gear || old_gear <= 1 || new_gear < 1 {
            return false;
        }

        let Some(target_rpm) = self.target_rpm_for_gear(new_gear, speed, wheel_radius) else {
            return false;
        };
        let rpm_gap = target_rpm - self.state.engine_rpm;
        if rpm_gap < AUTO_BLIP_MIN_RPM_GAP {
            return false;
        }

        self.shift_to = new_gear;
        self.shift_overshoot_rpm = (self.config.engine.max_rpm - self.state.engine_rpm).max(0.0)
            * AUTO_BLIP_OVERSHOOT_FACTOR;
        self.shift_phase = ShiftPhase::Disengaging;
        self.shift_phase_timer = AUTO_BLIP_CLUTCH_DISENGAGE_DURATION;
        self.shift_cooldown = self.config.transmission.shift_cooldown;
        true
    }

    fn update_shift_sequence(&mut self, dt: Real, speed: Real, wheel_radius: Real) {
        if self.shift_phase == ShiftPhase::Idle {
            return;
        }

        self.shift_phase_timer = (self.shift_phase_timer - dt).max(0.0);
        match self.shift_phase {
            ShiftPhase::Idle => {}
            ShiftPhase::Disengaging if self.shift_phase_timer <= Real::EPSILON => {
                self.state.current_gear = self.shift_to;
                self.state.reverse_direction = false;
                self.shift_phase = ShiftPhase::Blipping;
                self.shift_phase_timer = self.config.transmission.auto_blip_duration;
            }
            ShiftPhase::Blipping => {
                let overshoot_reached = self
                    .target_rpm_for_gear(self.shift_to, speed, wheel_radius)
                    .is_none_or(|target_rpm| {
                        let limit = self
                            .config
                            .engine
                            .rev_limit_rpm
                            .min(self.config.engine.max_rpm);
                        let overshoot_target = (target_rpm + self.shift_overshoot_rpm).min(limit);
                        self.state.engine_rpm >= overshoot_target - AUTO_BLIP_RPM_TOLERANCE
                    });
                if overshoot_reached {
                    self.shift_phase = ShiftPhase::Settling;
                } else if self.shift_phase_timer <= Real::EPSILON {
                    self.shift_phase = ShiftPhase::Reengaging;
                    self.shift_phase_timer = AUTO_BLIP_CLUTCH_REENGAGE_DURATION;
                }
            }
            ShiftPhase::Settling => {
                let target_reached = self
                    .target_rpm_for_gear(self.shift_to, speed, wheel_radius)
                    .is_none_or(|target_rpm| {
                        self.state.engine_rpm <= target_rpm + AUTO_BLIP_RPM_TOLERANCE
                    });
                if target_reached || self.shift_phase_timer <= Real::EPSILON {
                    self.shift_phase = ShiftPhase::Reengaging;
                    self.shift_phase_timer = AUTO_BLIP_CLUTCH_REENGAGE_DURATION;
                }
            }
            ShiftPhase::Reengaging if self.shift_phase_timer <= Real::EPSILON => {
                self.shift_phase = ShiftPhase::Idle;
                self.shift_phase_timer = 0.0;
                self.shift_to = 0;
                self.shift_overshoot_rpm = 0.0;
                self.shift_target_allows_blip = false;
            }
            _ => {}
        }
    }

    fn target_rpm_for_gear(&self, gear: i32, speed: Real, wheel_radius: Real) -> Option<Real> {
        let ratio = self
            .config
            .transmission
            .forward_ratios
            .get((gear - 1) as usize)
            .copied()?;
        let wheel_rpm = (speed.abs() / (TAU * wheel_radius.max(0.01))) * 60.0;
        Some(wheel_rpm * ratio * self.config.transmission.final_drive_ratio)
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
                self.shift_target_allows_blip = false;
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
                self.shift_target_allows_blip = false;
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
            self.shift_target_allows_blip = false;
            return;
        }

        if self.shift_target == 0 && drive_pedal > TRANSMISSION_PEDAL_ENGAGE {
            self.shift_target = if self.state.reverse_direction { -1 } else { 1 };
            self.shift_target_allows_blip = false;
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

        self.shift_target_allows_blip = target < self.shift_target;
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
    config.transmission.clutch_response = config.transmission.clutch_response.max(0.0);
    config.transmission.shift_cooldown = config.transmission.shift_cooldown.max(0.0);
    config.transmission.auto_blip_duration = config.transmission.auto_blip_duration.max(0.0);
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
    config.steering.road_wheel_curve = config.steering.road_wheel_curve.clamp(0.0, 1.0);
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
    fn clamps_road_wheel_curve_to_normalized_range() {
        let mut config = VehicleControllerConfig::default();
        config.steering.road_wheel_curve = -1.0;
        let linear = VehiclePowertrain::new(config.clone());
        assert_eq!(linear.config.steering.road_wheel_curve, 0.0);

        config.steering.road_wheel_curve = 2.0;
        let cubic = VehiclePowertrain::new(config);
        assert_eq!(cubic.config.steering.road_wheel_curve, 1.0);
    }

    #[test]
    fn interpolates_torque_curve() {
        let mut config = VehicleControllerConfig::default();
        config.engine.torque_curve = vec![(1000.0, 100.0), (3000.0, 200.0)];
        let powertrain = VehiclePowertrain::new(config);
        assert!((powertrain.torque_at(2000.0) - 150.0).abs() < 1.0e-4);
    }

    #[test]
    fn force_scale_does_not_change_mechanical_wheel_coupling_torque() {
        fn output(force_scale: Real) -> PowertrainOutput {
            let mut config = VehicleControllerConfig::default();
            config.engine.force_scale = force_scale;
            config.transmission.automatic = false;
            config.transmission.auto_clutch = true;
            let mut powertrain = VehiclePowertrain::new(config);
            select_first_gear(&mut powertrain);
            powertrain.state.engine_rpm = 3000.0;
            powertrain.set_input(VehicleInput {
                throttle: 1.0,
                ..VehicleInput::default()
            });
            let wheel_radius = 0.35;
            let wheel_speed = powertrain.state.engine_rpm
                / (powertrain.current_ratio() * powertrain.config.transmission.final_drive_ratio)
                / 60.0
                * TAU
                * wheel_radius;
            powertrain.update(1.0 / 60.0, wheel_speed, wheel_speed, wheel_radius)
        }

        let low_scale = output(0.3);
        let full_scale = output(1.0);

        assert!(low_scale.drive_torque < full_scale.drive_torque);
        assert!(
            (low_scale.wheel_coupling_torque - full_scale.wheel_coupling_torque).abs() < 1.0e-4
        );
    }

    #[test]
    fn throttle_holds_engine_rpm_at_fixed_road_speed_in_every_gear() {
        let mut config = VehicleControllerConfig::default();
        config.transmission.automatic = false;
        config.transmission.auto_clutch = true;
        let additional_ratio = config
            .transmission
            .forward_ratios
            .last()
            .copied()
            .unwrap_or(1.0)
            * 0.8;
        config.transmission.forward_ratios.push(additional_ratio);
        let wheel_radius = 0.35;
        let road_rpm = 3000.0;

        for gear_index in 0..config.transmission.forward_ratios.len() {
            let gear = gear_index as i32 + 1;
            let ratio = config.transmission.forward_ratios[gear_index]
                * config.transmission.final_drive_ratio;
            let road_speed = road_rpm / ratio / 60.0 * TAU * wheel_radius;
            let mut powertrain = VehiclePowertrain::new(config.clone());
            powertrain.set_gear(gear);
            powertrain.state.current_gear = gear;
            powertrain.state.engine_rpm = road_rpm;
            powertrain.set_input(VehicleInput {
                throttle: 1.0,
                ..VehicleInput::default()
            });

            for _ in 0..300 {
                powertrain.update(1.0 / 60.0, road_speed, road_speed, wheel_radius);
            }

            assert!(
                (powertrain.state.engine_rpm - road_rpm).abs() < 1.0,
                "gear {gear}: engine rpm {}, road rpm {road_rpm}",
                powertrain.state.engine_rpm
            );
        }
    }

    #[test]
    fn neutral_engine_speed_returns_to_idle_without_a_slow_midrange_tail() {
        let mut config = VehicleControllerConfig::default();
        config.engine.idle_rpm = 1000.0;
        config.engine.max_rpm = 8000.0;
        config.engine.rev_limit_rpm = 7900.0;
        config.engine.inertia = 0.9;
        config.engine.friction_torque = Some(70.0);
        let mut powertrain = VehiclePowertrain::new(config);
        powertrain.state.engine_rpm = 7900.0;

        for _ in 0..300 {
            powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
        }

        assert_eq!(powertrain.state.current_gear, 0);
        assert!(powertrain.state.engine_rpm >= powertrain.config.engine.idle_rpm);
        assert!(powertrain.state.engine_rpm < 2200.0);
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
    fn zero_shift_cooldown_does_not_oscillate_between_forward_and_reverse() {
        let mut powertrain = automatic_powertrain();
        powertrain.set_input(VehicleInput::default());
        powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);

        powertrain.set_input(VehicleInput {
            brake: 1.0,
            ..VehicleInput::default()
        });
        for _ in 0..5 {
            powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
            assert_eq!(powertrain.state().current_gear, -1);
            assert!(powertrain.state().reverse_direction);
        }

        powertrain.set_input(VehicleInput {
            throttle: 1.0,
            ..VehicleInput::default()
        });
        for _ in 0..5 {
            powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
            assert_eq!(powertrain.state().current_gear, 1);
            assert!(!powertrain.state().reverse_direction);
        }
    }

    #[test]
    fn automatic_transmission_selects_gears_from_road_speed_ranges() {
        let mut config = VehicleControllerConfig::default();
        config.engine.idle_rpm = 1000.0;
        config.engine.max_rpm = 6000.0;
        config.transmission.forward_ratios = vec![4.0, 2.0, 1.0];
        config.transmission.final_drive_ratio = 4.0;
        config.transmission.shift_cooldown = 0.0;
        config.transmission.auto_blip = false;
        let mut powertrain = VehiclePowertrain::new(config);
        powertrain.set_input(VehicleInput {
            throttle: 1.0,
            ..VehicleInput::default()
        });

        powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.3);
        assert_eq!(powertrain.state().current_gear, 1);
        powertrain.update(1.0 / 60.0, 12.0, 0.0, 0.3);
        assert_eq!(powertrain.state().current_gear, 2);
        powertrain.update(1.0 / 60.0, 8.0, 8.0, 0.3);
        assert_eq!(powertrain.state().current_gear, 1);
    }

    #[test]
    fn automatic_downshift_triggers_auto_blip() {
        let mut config = VehicleControllerConfig::default();
        config.engine.idle_rpm = 1000.0;
        config.engine.max_rpm = 6000.0;
        config.transmission.forward_ratios = vec![4.0, 2.0, 1.0];
        config.transmission.final_drive_ratio = 4.0;
        config.transmission.shift_cooldown = 0.0;
        let mut powertrain = VehiclePowertrain::new(config);
        powertrain.state.current_gear = 3;
        powertrain.shift_target = 3;
        powertrain.state.engine_rpm = 1000.0;

        powertrain.update(1.0 / 60.0, 8.0, 8.0, 0.3);

        assert_eq!(powertrain.state().current_gear, 3);
        assert_eq!(powertrain.shift_phase, ShiftPhase::Disengaging);
        assert!(powertrain.shift_overshoot_rpm > 0.0);
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

    #[test]
    fn reset_restores_initial_runtime_state_without_changing_config() {
        let mut config = VehicleControllerConfig::default();
        config.engine.idle_rpm = 1100.0;
        config.turbo.enabled = true;
        let mut powertrain = VehiclePowertrain::new(config);

        powertrain.input = VehicleInput {
            throttle: 1.0,
            brake: 0.5,
            clutch: 0.25,
            handbrake: 1.0,
            steering: -0.75,
        };
        powertrain.state = VehicleState {
            engine_rpm: 5000.0,
            engine_running: false,
            engine_starting: true,
            engine_start_progress: 0.5,
            engine_state_sequence: 6,
            gear_shift_sequence: 6,
            gear_shift_accepted_sequence: 7,
            gear_shift_ignored_sequence: 8,
            gear_shift_rejected_sequence: 9,
            current_gear: 3,
            reverse_direction: true,
            vehicle_speed: 30.0,
            driven_wheel_speed: 35.0,
            steering_angle: 0.4,
            driver_steering_angle: 0.3,
            engine_load: 0.8,
            rev_limiter_amount: 0.6,
            turbo_load: 0.9,
            turbo_release_sequence: 7,
            wheels_in_contact: 4,
            abs_activity: 0.5,
            esc_activity: 0.6,
            traction_control_activity: 0.4,
            force_feedback: 0.3,
            steering_friction: 0.2,
        };
        powertrain.shift_cooldown = 0.5;
        powertrain.reverse_cooldown = 0.4;
        powertrain.manual_override = 0.3;
        powertrain.shift_target = 4;
        powertrain.shift_target_allows_blip = true;
        powertrain.shift_phase = ShiftPhase::Blipping;
        powertrain.shift_phase_timer = 0.2;
        powertrain.shift_to = 3;
        powertrain.shift_overshoot_rpm = 600.0;
        powertrain.reverse_brake_armed = true;
        powertrain.turbo_load = 0.9;
        powertrain.previous_throttle = 1.0;
        powertrain.restart_armed = false;
        powertrain.engine_start_elapsed = 0.5;
        powertrain.automatic_clutch_engagement = 0.75;
        powertrain.automatic_clutch_phase = AutomaticClutchPhase::Launch;

        powertrain.reset();

        assert_eq!(powertrain.input(), VehicleInput::default());
        assert_eq!(
            powertrain.state(),
            VehicleState {
                engine_rpm: 1100.0,
                engine_running: true,
                current_gear: 0,
                ..VehicleState::default()
            }
        );
        assert_eq!(powertrain.shift_cooldown, 0.0);
        assert_eq!(powertrain.reverse_cooldown, 0.0);
        assert_eq!(powertrain.manual_override, 0.0);
        assert_eq!(powertrain.shift_target, 0);
        assert!(!powertrain.shift_target_allows_blip);
        assert_eq!(powertrain.shift_phase, ShiftPhase::Idle);
        assert_eq!(powertrain.shift_phase_timer, 0.0);
        assert_eq!(powertrain.shift_to, 0);
        assert_eq!(powertrain.shift_overshoot_rpm, 0.0);
        assert_eq!(powertrain.state().gear_shift_accepted_sequence, 0);
        assert_eq!(powertrain.state().gear_shift_ignored_sequence, 0);
        assert_eq!(powertrain.state().gear_shift_rejected_sequence, 0);
        assert_eq!(powertrain.state().gear_shift_sequence, 0);
        assert_eq!(powertrain.state().engine_state_sequence, 0);
        assert!(!powertrain.reverse_brake_armed);
        assert_eq!(powertrain.turbo_load, 0.0);
        assert_eq!(powertrain.previous_throttle, 0.0);
        assert!(powertrain.restart_armed);
        assert_eq!(powertrain.engine_start_elapsed, 0.0);
        assert_eq!(powertrain.automatic_clutch_engagement, 0.0);
        assert_eq!(
            powertrain.automatic_clutch_phase,
            AutomaticClutchPhase::Open
        );
        assert_eq!(powertrain.config.engine.idle_rpm, 1100.0);
        assert!(powertrain.config.turbo.enabled);
    }

    fn select_first_gear(powertrain: &mut VehiclePowertrain) {
        let input = powertrain.input();
        powertrain.set_input(VehicleInput {
            clutch: 1.0,
            ..input
        });
        powertrain.set_gear(1);
        powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
        assert_eq!(powertrain.state().current_gear, 1);
        powertrain.set_input(input);
    }

    #[test]
    fn automatic_clutch_launch_drops_below_idle_without_stalling() {
        let mut powertrain = automatic_powertrain();
        assert!(!powertrain.config.transmission.auto_clutch);
        select_first_gear(&mut powertrain);
        powertrain.state.engine_rpm = powertrain.config.engine.idle_rpm;
        powertrain.set_input(VehicleInput {
            throttle: 1.0,
            ..VehicleInput::default()
        });

        let idle_rpm = powertrain.config.engine.idle_rpm;
        let mut peak_drive_torque: Real = 0.0;
        let mut minimum_rpm = idle_rpm;
        for _ in 0..60 {
            let output = powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
            peak_drive_torque = peak_drive_torque.max(output.drive_torque.abs());
            minimum_rpm = minimum_rpm.min(powertrain.state().engine_rpm);
        }

        assert!(powertrain.state().engine_running);
        assert_eq!(powertrain.state().current_gear, 1);
        assert!(peak_drive_torque > 0.0);
        assert!(minimum_rpm < idle_rpm * 0.85);
        assert!(minimum_rpm > idle_rpm * STALL_RPM_RATIO);
    }

    #[test]
    fn automatic_clutch_synchronizes_engine_after_a_revved_neutral_shift() {
        let mut powertrain = automatic_powertrain();
        select_first_gear(&mut powertrain);
        powertrain.state.engine_rpm = 5000.0;
        let wheel_radius = 0.35;
        let drivetrain_rpm = 2000.0;
        let wheel_speed = wheel_speed_for_crank_rpm(&powertrain, drivetrain_rpm, wheel_radius);
        let initial_slip = (powertrain.state().engine_rpm - drivetrain_rpm).abs();

        for _ in 0..60 {
            powertrain.update(1.0 / 60.0, wheel_speed, wheel_speed, wheel_radius);
        }

        let final_slip = (powertrain.state().engine_rpm - drivetrain_rpm).abs();
        assert!(powertrain.state().engine_running);
        assert!(final_slip < initial_slip * 0.05);
    }

    #[test]
    fn manual_auto_clutch_matches_direct_clutch_synchronization_above_idle() {
        let mut managed = manual_auto_clutch_powertrain();
        let mut direct = manual_powertrain();
        select_first_gear(&mut managed);
        select_first_gear(&mut direct);
        managed.state.engine_rpm = 5000.0;
        direct.state.engine_rpm = 5000.0;
        let wheel_radius = 0.35;
        let wheel_speed = wheel_speed_for_crank_rpm(&managed, 2000.0, wheel_radius);

        for _ in 0..60 {
            managed.update(1.0 / 60.0, wheel_speed, wheel_speed, wheel_radius);
            direct.update(1.0 / 60.0, wheel_speed, wheel_speed, wheel_radius);
        }

        assert!(
            (managed.state().engine_rpm - direct.state().engine_rpm).abs() < 1.0,
            "managed {}, direct {}",
            managed.state().engine_rpm,
            direct.state().engine_rpm
        );
    }

    #[test]
    fn clutch_synchronization_is_stable_across_timesteps() {
        let mut final_rpms = Vec::new();
        for dt in [1.0 / 30.0, 1.0 / 60.0, 1.0 / 120.0] {
            let mut powertrain = manual_auto_clutch_powertrain();
            select_first_gear(&mut powertrain);
            powertrain.state.engine_rpm = 5000.0;
            let wheel_radius = 0.35;
            let wheel_speed = wheel_speed_for_crank_rpm(&powertrain, 2000.0, wheel_radius);

            for _ in 0..(1.0 / dt) as usize {
                powertrain.update(dt, wheel_speed, wheel_speed, wheel_radius);
            }
            final_rpms.push(powertrain.state().engine_rpm);
        }

        let minimum = final_rpms.iter().copied().fold(Real::MAX, Real::min);
        let maximum = final_rpms.iter().copied().fold(Real::MIN, Real::max);
        assert!(
            maximum - minimum < 5.0,
            "final RPMs {final_rpms:?}, range {}",
            maximum - minimum
        );
    }

    struct LaunchResult {
        target_rpm: Real,
        minimum_launch_rpm: Real,
        maximum_launch_rpm: Real,
        final_speed: Real,
        final_clutch_engagement: Real,
        engine_running: bool,
        locked: bool,
    }

    fn launch_powertrain(idle_rpm: Real) -> VehiclePowertrain {
        let mut config = VehicleControllerConfig::default();
        config.engine.idle_rpm = idle_rpm;
        config.engine.max_rpm = idle_rpm + 6000.0;
        config.engine.rev_limit_rpm = config.engine.max_rpm - 100.0;
        config.engine.torque_curve = vec![
            (idle_rpm, 220.0),
            (idle_rpm + 2500.0, 320.0),
            (config.engine.max_rpm, 210.0),
        ];
        config.transmission.shift_cooldown = 0.0;
        let mut powertrain = VehiclePowertrain::new(config);
        select_first_gear(&mut powertrain);
        powertrain.set_input(VehicleInput {
            throttle: 1.0,
            ..VehicleInput::default()
        });

        for _ in 0..300 {
            powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
            if powertrain.automatic_clutch_phase == AutomaticClutchPhase::Launch {
                break;
            }
        }

        assert_eq!(
            powertrain.automatic_clutch_phase,
            AutomaticClutchPhase::Launch
        );
        powertrain
    }

    #[test]
    fn automatic_clutch_targets_just_above_stall_rpm() {
        let powertrain = launch_powertrain(1500.0);

        let target_rpm = powertrain.automatic_clutch_target_rpm();
        let stall_rpm = powertrain.config.engine.idle_rpm * STALL_RPM_RATIO;
        assert_eq!(target_rpm, 1050.0);
        assert!(target_rpm > stall_rpm);
        assert!(target_rpm < powertrain.config.engine.idle_rpm);
    }

    #[test]
    fn launch_clutch_engagement_responds_to_rpm_error() {
        let mut powertrain = launch_powertrain(1500.0);
        let target_rpm = powertrain.automatic_clutch_target_rpm();
        let drivetrain_engine_torque = powertrain.torque_at(target_rpm);

        powertrain.state.engine_rpm = target_rpm - 500.0;
        let below_target = powertrain.automatic_clutch_launch_engagement(
            target_rpm,
            0.0,
            drivetrain_engine_torque,
        );
        powertrain.state.engine_rpm = target_rpm;
        let at_target = powertrain.automatic_clutch_launch_engagement(
            target_rpm,
            0.0,
            drivetrain_engine_torque,
        );
        powertrain.state.engine_rpm = target_rpm + 500.0;
        let above_target = powertrain.automatic_clutch_launch_engagement(
            target_rpm,
            0.0,
            drivetrain_engine_torque,
        );

        assert!(below_target < at_target);
        assert!(at_target > 0.0 && at_target < 1.0);
        assert!(above_target > at_target);
    }

    #[test]
    fn automatic_clutch_intervenes_only_near_stall() {
        let idle_rpm = 2000.0;
        let mut powertrain = launch_powertrain(idle_rpm);
        powertrain.automatic_clutch_engagement = 1.0;
        powertrain.state.engine_rpm = idle_rpm * (AUTO_CLUTCH_ANTISTALL_ENTER_RPM_RATIO - 0.01);

        powertrain.update(1.0 / 120.0, 0.0, 0.0, 0.35);

        assert_eq!(
            powertrain.automatic_clutch_phase,
            AutomaticClutchPhase::AntiStall
        );
        assert!(powertrain.automatic_clutch_engagement < 1.0);
        assert!(powertrain.state().engine_running);
    }

    #[test]
    fn automatic_clutch_launches_progressively_in_higher_gears() {
        let mut powertrain = manual_auto_clutch_powertrain();
        select_manual_gear(&mut powertrain, 2);
        powertrain.set_input(VehicleInput {
            throttle: 1.0,
            ..VehicleInput::default()
        });

        powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);

        assert_eq!(
            powertrain.automatic_clutch_phase,
            AutomaticClutchPhase::Launch
        );
        assert!(powertrain.automatic_clutch_engagement > 0.0);
        assert!(powertrain.automatic_clutch_engagement < 1.0);
        assert!(powertrain.state().engine_running);
    }

    #[test]
    fn locked_clutch_preserves_engine_braking_in_first_and_reverse() {
        let wheel_radius = 0.35;
        for gear in [-1, 1] {
            let mut powertrain = manual_auto_clutch_powertrain();
            select_manual_gear(&mut powertrain, gear);
            powertrain.state.engine_rpm = 3000.0;
            powertrain.automatic_clutch_phase = AutomaticClutchPhase::Locked;
            powertrain.automatic_clutch_engagement = 1.0;
            let wheel_speed = wheel_speed_for_crank_rpm(&powertrain, 3000.0, wheel_radius);

            let output = powertrain.update(1.0 / 60.0, wheel_speed, wheel_speed, wheel_radius);

            assert_eq!(
                powertrain.automatic_clutch_phase,
                AutomaticClutchPhase::Locked
            );
            assert_eq!(powertrain.automatic_clutch_engagement, 1.0);
            assert!(output.engine_brake_torque > 0.0);
        }
    }

    #[test]
    fn driven_wheel_spin_cannot_finish_launch_before_ground_speed_reaches_target_rpm() {
        let idle_rpm = 2000.0;
        let wheel_radius = 0.35;
        let mut powertrain = launch_powertrain(idle_rpm);
        let target_rpm = powertrain.automatic_clutch_target_rpm();
        powertrain.state.engine_rpm = target_rpm;
        let spinning_wheel_speed =
            wheel_speed_for_crank_rpm(&powertrain, target_rpm * 2.0, wheel_radius);

        powertrain.update(1.0 / 60.0, 0.0, spinning_wheel_speed, wheel_radius);
        assert_eq!(
            powertrain.automatic_clutch_phase,
            AutomaticClutchPhase::Launch
        );

        let target_ground_speed = wheel_speed_for_crank_rpm(&powertrain, target_rpm, wheel_radius);
        powertrain.update(
            1.0 / 60.0,
            target_ground_speed,
            spinning_wheel_speed,
            wheel_radius,
        );
        assert_eq!(
            powertrain.automatic_clutch_phase,
            AutomaticClutchPhase::Locked
        );
        assert_eq!(powertrain.automatic_clutch_engagement, 1.0);
    }

    fn simulate_automatic_launch(
        dt: Real,
        idle_rpm: Real,
        direction: Real,
        throttle: Real,
    ) -> LaunchResult {
        let mut config = VehicleControllerConfig::default();
        config.engine.idle_rpm = idle_rpm;
        config.engine.max_rpm = idle_rpm + 6000.0;
        config.engine.rev_limit_rpm = config.engine.max_rpm - 100.0;
        config.engine.torque_curve = vec![
            (idle_rpm, 220.0),
            (idle_rpm + 2500.0, 320.0),
            (config.engine.max_rpm, 210.0),
        ];
        config.transmission.shift_cooldown = 0.0;
        let mut powertrain = VehiclePowertrain::new(config);
        let input = if direction > 0.0 {
            VehicleInput {
                throttle,
                ..VehicleInput::default()
            }
        } else {
            powertrain.state.reverse_direction = true;
            powertrain.shift_target = -1;
            VehicleInput {
                brake: throttle,
                ..VehicleInput::default()
            }
        };
        powertrain.set_input(input);

        let wheel_radius = 0.35;
        let effective_mass = 1300.0;
        let mut speed = 0.0;
        let mut minimum_launch_rpm = Real::MAX;
        let mut maximum_launch_rpm = Real::MIN;
        let mut target_reached = false;
        let mut final_clutch_engagement = 0.0;
        let steps = (6.0 / dt) as usize;
        let target_rpm = powertrain.automatic_clutch_target_rpm();

        for _ in 0..steps {
            let output = powertrain.update(dt, speed, speed, wheel_radius);
            speed += output.drive_torque / (effective_mass * wheel_radius) * dt;
            final_clutch_engagement = powertrain.automatic_clutch_engagement;
            let rpm = powertrain.state().engine_rpm;
            target_reached |= rpm <= target_rpm * 1.1;
            if target_reached && powertrain.automatic_clutch_phase == AutomaticClutchPhase::Launch {
                minimum_launch_rpm = minimum_launch_rpm.min(rpm);
                maximum_launch_rpm = maximum_launch_rpm.max(rpm);
            }
        }

        LaunchResult {
            target_rpm,
            minimum_launch_rpm,
            maximum_launch_rpm,
            final_speed: speed,
            final_clutch_engagement,
            engine_running: powertrain.state().engine_running,
            locked: powertrain.automatic_clutch_phase == AutomaticClutchPhase::Locked,
        }
    }

    #[test]
    fn automatic_launch_handles_high_idle_reverse_and_partial_throttle() {
        let cases = [
            (2000.0, 1.0, 1.0, 1.0, 0.8, 1.2),
            (1500.0, -1.0, 1.0, 1.0, 0.8, 1.2),
            (
                1500.0,
                1.0,
                0.35,
                0.5,
                STALL_RPM_RATIO / AUTO_CLUTCH_LAUNCH_RPM_RATIO,
                1.25,
            ),
        ];

        for (idle_rpm, direction, throttle, minimum_speed, minimum_rpm, maximum_rpm) in cases {
            let result = simulate_automatic_launch(1.0 / 60.0, idle_rpm, direction, throttle);
            assert!(result.engine_running);
            assert!((result.target_rpm - idle_rpm * AUTO_CLUTCH_LAUNCH_RPM_RATIO).abs() < 1.0);
            assert!(result.minimum_launch_rpm > result.target_rpm * minimum_rpm);
            assert!(result.maximum_launch_rpm < result.target_rpm * maximum_rpm);
            assert!(result.final_speed * direction > minimum_speed);
            assert!(result.final_clutch_engagement > 0.95);
            assert!(result.locked);
        }
    }

    #[test]
    fn automatic_launch_is_stable_across_timesteps() {
        let results = [1.0 / 30.0, 1.0 / 60.0, 1.0 / 120.0]
            .map(|dt| simulate_automatic_launch(dt, 1500.0, 1.0, 1.0));
        let minimum_speed = results
            .iter()
            .map(|result| result.final_speed)
            .fold(Real::MAX, Real::min);
        let maximum_speed = results
            .iter()
            .map(|result| result.final_speed)
            .fold(Real::MIN, Real::max);

        assert!(results.iter().all(|result| result.engine_running));
        assert!(results
            .iter()
            .all(|result| result.maximum_launch_rpm < result.target_rpm * 1.25));
        assert!(maximum_speed - minimum_speed < maximum_speed * 0.08);
    }

    #[test]
    fn stopped_throttle_abort_returns_the_automatic_clutch_to_open() {
        let mut powertrain = automatic_powertrain();
        powertrain.set_input(VehicleInput {
            throttle: 1.0,
            ..VehicleInput::default()
        });
        for _ in 0..90 {
            powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
        }
        assert!(powertrain.automatic_clutch_engagement > 0.0);

        powertrain.set_input(VehicleInput::default());
        powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
        powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);

        assert_eq!(powertrain.state().current_gear, 0);
        assert_eq!(powertrain.automatic_clutch_engagement, 0.0);
    }

    #[test]
    fn zero_clutch_response_still_applies_the_managed_target() {
        let mut config = VehicleControllerConfig::default();
        config.transmission.clutch_response = 0.0;
        config.transmission.shift_cooldown = 0.0;
        let mut powertrain = VehiclePowertrain::new(config);
        powertrain.set_input(VehicleInput {
            throttle: 1.0,
            ..VehicleInput::default()
        });

        let mut peak_drive_torque: Real = 0.0;
        for _ in 0..60 {
            let output = powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
            peak_drive_torque = peak_drive_torque.max(output.drive_torque);
        }

        assert!(peak_drive_torque > 0.0);
        assert!(powertrain.state().engine_running);
    }

    fn manual_powertrain() -> VehiclePowertrain {
        let mut config = VehicleControllerConfig::default();
        config.transmission.automatic = false;
        config.transmission.shift_cooldown = 0.0;
        VehiclePowertrain::new(config)
    }

    fn manual_auto_clutch_powertrain() -> VehiclePowertrain {
        let mut config = VehicleControllerConfig::default();
        config.transmission.automatic = false;
        config.transmission.auto_clutch = true;
        config.transmission.shift_cooldown = 0.0;
        VehiclePowertrain::new(config)
    }

    fn low_inertia_auto_clutch_powertrain() -> VehiclePowertrain {
        let mut config = VehicleControllerConfig::default();
        config.engine.horsepower = 1000.0;
        config.engine.idle_rpm = 4500.0;
        config.engine.max_rpm = 15000.0;
        config.engine.rev_limit_rpm = 15000.0;
        config.engine.inertia = 0.06;
        config.engine.torque_curve = vec![(1000.0, 550.0), (8000.0, 700.0), (15000.0, 545.0)];
        config.transmission.automatic = false;
        config.transmission.auto_clutch = true;
        config.transmission.forward_ratios = vec![3.5];
        config.transmission.final_drive_ratio = 4.0;
        config.transmission.clutch_response = 22.0;
        config.transmission.shift_cooldown = 0.0;
        let mut powertrain = VehiclePowertrain::new(config);
        powertrain.state.current_gear = 1;
        powertrain.shift_target = 1;
        powertrain.state.engine_rpm = 3000.0;
        powertrain.automatic_clutch_phase = AutomaticClutchPhase::Locked;
        powertrain.automatic_clutch_engagement = 1.0;
        powertrain
    }

    fn select_manual_gear(powertrain: &mut VehiclePowertrain, gear: i32) {
        powertrain.set_input(VehicleInput {
            clutch: 1.0,
            ..VehicleInput::default()
        });
        powertrain.set_gear(gear);
        powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
        assert_eq!(powertrain.state().current_gear, gear);
        powertrain.set_input(VehicleInput::default());
    }

    #[test]
    fn sequential_manual_downshift_triggers_auto_blip() {
        let mut powertrain = manual_auto_clutch_powertrain();
        select_manual_gear(&mut powertrain, 3);
        powertrain.config.engine.inertia = 0.2;
        powertrain.config.transmission.auto_blip_duration = 0.5;
        powertrain.state.engine_rpm = 1000.0;
        let wheel_radius = 0.35;
        let target_rpm = 1500.0;
        let wheel_speed = wheel_speed_for_gear_crank_rpm(&powertrain, 2, target_rpm, wheel_radius);

        powertrain.shift_down();
        powertrain.update(1.0 / 60.0, wheel_speed, wheel_speed, wheel_radius);
        let expected_overshoot = (powertrain.config.engine.max_rpm - powertrain.state.engine_rpm)
            * AUTO_BLIP_OVERSHOOT_FACTOR;

        assert_eq!(powertrain.state().current_gear, 3);
        assert_eq!(powertrain.shift_phase, ShiftPhase::Disengaging);
        assert_eq!(powertrain.shift_clutch_override(), 0.0);
        assert_eq!(powertrain.shift_throttle_override(), 0.0);
        assert_eq!(powertrain.shift_overshoot_rpm, expected_overshoot);

        let mut output = powertrain.update(1.0 / 60.0, wheel_speed, wheel_speed, wheel_radius);
        for _ in 0..2 {
            output = powertrain.update(1.0 / 60.0, wheel_speed, wheel_speed, wheel_radius);
        }

        assert_eq!(powertrain.state().current_gear, 2);
        assert_eq!(powertrain.shift_phase, ShiftPhase::Blipping);
        assert_eq!(powertrain.shift_clutch_override(), 1.0);
        assert!(powertrain.shift_throttle_override() > 0.0);
        assert_eq!(output.drive_torque, 0.0);

        let mut peak_blip_rpm = powertrain.state.engine_rpm;
        let mut settled_after_overshoot = false;
        for _ in 0..60 {
            powertrain.update(1.0 / 60.0, wheel_speed, wheel_speed, wheel_radius);
            if matches!(
                powertrain.shift_phase,
                ShiftPhase::Blipping | ShiftPhase::Settling
            ) {
                peak_blip_rpm = peak_blip_rpm.max(powertrain.state.engine_rpm);
            }
            settled_after_overshoot |= powertrain.shift_phase == ShiftPhase::Settling;
        }
        assert!(peak_blip_rpm >= target_rpm + expected_overshoot - AUTO_BLIP_RPM_TOLERANCE);
        assert!(settled_after_overshoot);
        assert_eq!(powertrain.shift_phase, ShiftPhase::Idle);
        assert_eq!(powertrain.shift_clutch_override(), 0.0);
        assert_eq!(powertrain.shift_throttle_override(), 0.0);
    }

    #[test]
    fn direct_manual_gear_selection_does_not_trigger_auto_blip() {
        let mut powertrain = manual_auto_clutch_powertrain();
        select_manual_gear(&mut powertrain, 3);
        powertrain.state.engine_rpm = 1000.0;
        let wheel_radius = 0.35;
        let wheel_speed = wheel_speed_for_gear_crank_rpm(&powertrain, 2, 5000.0, wheel_radius);

        powertrain.set_gear(2);
        powertrain.update(1.0 / 60.0, wheel_speed, wheel_speed, wheel_radius);

        assert_eq!(powertrain.state().current_gear, 2);
        assert_eq!(powertrain.shift_phase, ShiftPhase::Idle);
        assert_eq!(powertrain.shift_throttle_override(), 0.0);
    }

    #[test]
    fn manual_shift_requires_eighty_percent_clutch_disengagement() {
        let mut powertrain = manual_powertrain();
        powertrain.set_input(VehicleInput {
            clutch: MANUAL_SHIFT_CLUTCH_DISENGAGEMENT,
            ..VehicleInput::default()
        });

        let outcome = powertrain.set_gear(1);
        powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);

        assert_eq!(outcome, VehicleShiftOutcome::Accepted);
        assert_eq!(powertrain.state().gear_shift_sequence, 1);
        assert_eq!(powertrain.state().gear_shift_accepted_sequence, 1);
        assert_eq!(powertrain.state().current_gear, 1);
    }

    #[test]
    fn missed_manual_shift_up_falls_back_to_neutral() {
        let mut powertrain = manual_powertrain();
        powertrain.set_input(VehicleInput {
            clutch: 1.0,
            ..VehicleInput::default()
        });
        powertrain.set_gear(1);
        powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);

        powertrain.set_input(VehicleInput {
            clutch: MANUAL_SHIFT_CLUTCH_DISENGAGEMENT - 0.01,
            ..VehicleInput::default()
        });
        let outcome = powertrain.shift_up();
        powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);

        assert_eq!(outcome, VehicleShiftOutcome::ClutchRejected);
        assert_eq!(powertrain.state().gear_shift_rejected_sequence, 1);
        assert_eq!(powertrain.state().current_gear, 0);
    }

    #[test]
    fn missed_manual_shift_down_falls_back_to_neutral() {
        let mut powertrain = manual_powertrain();
        powertrain.set_input(VehicleInput {
            clutch: 1.0,
            ..VehicleInput::default()
        });
        powertrain.set_gear(2);
        powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);

        powertrain.set_input(VehicleInput::default());
        let outcome = powertrain.shift_down();
        powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);

        assert_eq!(outcome, VehicleShiftOutcome::ClutchRejected);
        assert_eq!(powertrain.state().current_gear, 0);
    }

    #[test]
    fn missed_direct_manual_gear_selection_falls_back_to_neutral_during_cooldown() {
        let mut config = VehicleControllerConfig::default();
        config.transmission.automatic = false;
        config.transmission.shift_cooldown = 1.0;
        let mut powertrain = VehiclePowertrain::new(config);
        powertrain.set_input(VehicleInput {
            clutch: 1.0,
            ..VehicleInput::default()
        });
        powertrain.set_gear(1);
        powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
        assert_eq!(powertrain.state().current_gear, 1);

        powertrain.set_input(VehicleInput::default());
        let outcome = powertrain.set_gear(-1);
        powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);

        assert_eq!(outcome, VehicleShiftOutcome::ClutchRejected);
        assert_eq!(powertrain.state().current_gear, 0);
    }

    #[test]
    fn shift_during_cooldown_is_ignored_without_a_clutch_rejection() {
        let mut config = VehicleControllerConfig::default();
        config.transmission.automatic = false;
        config.transmission.shift_cooldown = 1.0;
        let mut powertrain = VehiclePowertrain::new(config);
        powertrain.set_input(VehicleInput {
            clutch: 1.0,
            ..VehicleInput::default()
        });
        powertrain.set_gear(1);
        powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);

        let outcome = powertrain.shift_up();

        assert_eq!(outcome, VehicleShiftOutcome::Ignored);
        assert_eq!(powertrain.state().gear_shift_sequence, 2);
        assert_eq!(powertrain.state().gear_shift_ignored_sequence, 1);
        assert_eq!(powertrain.state().current_gear, 1);
    }

    #[test]
    fn manual_neutral_selection_does_not_require_the_clutch() {
        let mut powertrain = manual_powertrain();
        powertrain.set_input(VehicleInput {
            clutch: 1.0,
            ..VehicleInput::default()
        });
        powertrain.set_gear(1);
        powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);

        powertrain.set_input(VehicleInput::default());
        let outcome = powertrain.set_gear(0);
        powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);

        assert_eq!(outcome, VehicleShiftOutcome::Accepted);
        assert_eq!(powertrain.state().current_gear, 0);
    }

    fn engage_first_gear(powertrain: &mut VehiclePowertrain, input: VehicleInput) {
        powertrain.set_input(VehicleInput {
            clutch: 1.0,
            ..input
        });
        powertrain.set_gear(1);
        powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
        assert_eq!(powertrain.state().current_gear, 1);
        powertrain.set_input(input);
    }

    fn prepare_stalled_powertrain_in_gear(gear: i32, clutch: Real) -> VehiclePowertrain {
        let mut powertrain = manual_powertrain();
        powertrain.set_input(VehicleInput {
            clutch: 1.0,
            ..VehicleInput::default()
        });
        powertrain.set_gear(gear);
        powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
        assert_eq!(powertrain.state().current_gear, gear);
        powertrain.set_input(VehicleInput {
            clutch,
            ..VehicleInput::default()
        });
        powertrain.state.engine_running = false;
        powertrain.state.engine_rpm = 0.0;
        powertrain.restart_armed = false;
        powertrain
    }

    fn wheel_speed_for_crank_rpm(
        powertrain: &VehiclePowertrain,
        crank_rpm: Real,
        wheel_radius: Real,
    ) -> Real {
        crank_rpm
            / (powertrain.current_ratio() * powertrain.config.transmission.final_drive_ratio)
            / 60.0
            * TAU
            * wheel_radius
    }

    fn wheel_speed_for_gear_crank_rpm(
        powertrain: &VehiclePowertrain,
        gear: i32,
        crank_rpm: Real,
        wheel_radius: Real,
    ) -> Real {
        let ratio = powertrain.config.transmission.forward_ratios[(gear - 1) as usize];
        crank_rpm / (ratio * powertrain.config.transmission.final_drive_ratio) / 60.0
            * TAU
            * wheel_radius
    }

    fn update_until_started(
        powertrain: &mut VehiclePowertrain,
        driven_wheel_speed: Real,
        wheel_radius: Real,
    ) {
        for _ in 0..120 {
            powertrain.update(
                1.0 / 60.0,
                driven_wheel_speed.signum(),
                driven_wheel_speed,
                wheel_radius,
            );
            if powertrain.state().engine_running {
                break;
            }
        }
    }

    #[test]
    fn running_engine_holds_idle_while_disconnected_from_the_drivetrain() {
        let mut powertrain = manual_powertrain();

        for _ in 0..120 {
            powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
        }

        assert!(powertrain.state().engine_running);
        assert!((powertrain.state().engine_rpm - powertrain.config.engine.idle_rpm).abs() < 1.0e-4);
    }

    #[test]
    fn engaged_clutch_transmits_idle_combustion_torque_at_matching_wheel_speed() {
        let wheel_radius = 0.35;
        let mut powertrain = manual_powertrain();
        engage_first_gear(&mut powertrain, VehicleInput::default());
        powertrain.state.engine_rpm = powertrain.config.engine.idle_rpm;
        let wheel_speed =
            wheel_speed_for_crank_rpm(&powertrain, powertrain.config.engine.idle_rpm, wheel_radius);

        for _ in 0..120 {
            let output = powertrain.update(1.0 / 60.0, wheel_speed, wheel_speed, wheel_radius);
            assert!(output.drive_torque > 0.0);
            assert!(powertrain.state().engine_running);
        }

        assert!((powertrain.state().engine_rpm - powertrain.config.engine.idle_rpm).abs() < 1.0e-3);
    }

    #[test]
    fn idle_combustion_torque_does_not_drive_in_neutral_or_with_the_clutch_disengaged() {
        let wheel_radius = 0.35;
        let mut neutral = manual_powertrain();
        let neutral_output = neutral.update(1.0 / 60.0, 0.0, 0.0, wheel_radius);
        assert_eq!(neutral_output.drive_torque, 0.0);

        let mut disengaged = manual_powertrain();
        let input = VehicleInput {
            clutch: 1.0,
            ..VehicleInput::default()
        };
        engage_first_gear(&mut disengaged, input);
        let wheel_speed =
            wheel_speed_for_crank_rpm(&disengaged, disengaged.config.engine.idle_rpm, wheel_radius);
        let disengaged_output =
            disengaged.update(1.0 / 60.0, wheel_speed, wheel_speed, wheel_radius);
        assert_eq!(disengaged_output.drive_torque, 0.0);
    }

    #[test]
    fn engaged_clutch_transmits_idle_torque_and_can_stall_the_engine() {
        let mut powertrain = manual_powertrain();
        assert!(!powertrain.config.transmission.auto_clutch);
        engage_first_gear(&mut powertrain, VehicleInput::default());

        let output = powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
        assert!(output.drive_torque > 0.0);
        assert!(powertrain.state().engine_rpm < powertrain.config.engine.idle_rpm);

        for _ in 0..30 {
            powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
            if !powertrain.state().engine_running {
                break;
            }
        }

        assert!(!powertrain.state().engine_running);
        assert_eq!(powertrain.state().engine_rpm, 0.0);
        let output = powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
        assert_eq!(output.drive_torque, 0.0);
        assert_eq!(output.engine_brake_torque, 0.0);
        assert_eq!(output.wheel_target_velocity, 0.0);
    }

    #[test]
    fn disengaged_clutch_prevents_drivetrain_load_from_stalling_the_engine() {
        let mut powertrain = manual_powertrain();
        let input = VehicleInput {
            clutch: 1.0,
            ..VehicleInput::default()
        };
        engage_first_gear(&mut powertrain, input);

        for _ in 0..120 {
            powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
        }

        assert!(powertrain.state().engine_running);
        assert!(powertrain.state().engine_rpm >= powertrain.config.engine.idle_rpm);
    }

    #[test]
    fn enabled_manual_auto_clutch_prevents_stalling() {
        let mut powertrain = manual_auto_clutch_powertrain();
        engage_first_gear(&mut powertrain, VehicleInput::default());
        powertrain.state.engine_rpm = powertrain.config.engine.idle_rpm;

        for _ in 0..60 {
            powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
        }

        assert!(powertrain.state().engine_running);
        assert!(powertrain.state().engine_rpm >= powertrain.config.engine.idle_rpm);
    }

    #[test]
    fn managed_clutch_disengages_before_braking_to_a_stop_can_stall_the_engine() {
        let mut powertrain = manual_auto_clutch_powertrain();
        powertrain.state.current_gear = 2;
        powertrain.state.engine_rpm = 3000.0;
        powertrain.automatic_clutch_phase = AutomaticClutchPhase::Locked;
        powertrain.automatic_clutch_engagement = 1.0;
        powertrain.set_input(VehicleInput {
            brake: 1.0,
            ..VehicleInput::default()
        });

        for _ in 0..30 {
            powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
            assert!(powertrain.state().engine_running);
        }

        assert_ne!(
            powertrain.automatic_clutch_phase,
            AutomaticClutchPhase::Locked
        );
        assert_eq!(powertrain.automatic_clutch_engagement, 0.0);
        assert!(
            powertrain.state().engine_rpm >= powertrain.config.engine.idle_rpm * STALL_RPM_RATIO
        );
    }

    #[test]
    fn low_inertia_auto_clutch_survives_driven_wheel_lock_under_braking() {
        for (dt, steps) in [(1.0 / 30.0, 15), (1.0 / 60.0, 30), (1.0 / 120.0, 60)] {
            for input in [
                VehicleInput {
                    brake: 1.0,
                    ..VehicleInput::default()
                },
                VehicleInput {
                    handbrake: 1.0,
                    ..VehicleInput::default()
                },
            ] {
                let mut powertrain = low_inertia_auto_clutch_powertrain();
                powertrain.set_input(input);

                for _ in 0..steps {
                    powertrain.update(dt, 10.0, 0.0, 0.335);
                    assert!(powertrain.state().engine_running);
                }

                assert_eq!(
                    powertrain.automatic_clutch_phase,
                    AutomaticClutchPhase::AntiStall
                );
                assert_eq!(powertrain.automatic_clutch_engagement, 0.0);
            }
        }
    }

    #[test]
    fn projected_rpm_protection_survives_unbraked_drivetrain_speed_collapse() {
        for dt in [1.0 / 30.0, 1.0 / 60.0, 1.0 / 120.0] {
            let mut powertrain = low_inertia_auto_clutch_powertrain();

            powertrain.update(dt, 10.0, 0.0, 0.335);

            assert!(powertrain.state().engine_running);
            assert!(
                powertrain.state().engine_rpm
                    >= powertrain.config.engine.idle_rpm * STALL_RPM_RATIO
            );
        }
    }

    #[test]
    fn enabled_manual_auto_clutch_transmits_torque_without_stalling() {
        let mut powertrain = manual_auto_clutch_powertrain();
        engage_first_gear(&mut powertrain, VehicleInput::default());
        powertrain.set_input(VehicleInput {
            throttle: 1.0,
            ..VehicleInput::default()
        });

        let idle_rpm = powertrain.config.engine.idle_rpm;
        let mut peak_drive_torque: Real = 0.0;
        let mut minimum_rpm = idle_rpm;
        for _ in 0..60 {
            let output = powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
            peak_drive_torque = peak_drive_torque.max(output.drive_torque);
            minimum_rpm = minimum_rpm.min(powertrain.state().engine_rpm);
            assert!(powertrain.state().engine_running);
        }
        assert!(peak_drive_torque > 0.0);
        assert!(minimum_rpm < idle_rpm * 0.85);
        assert!(minimum_rpm > idle_rpm * STALL_RPM_RATIO);
    }

    #[test]
    fn manual_auto_clutch_configuration_survives_reset() {
        let mut powertrain = manual_auto_clutch_powertrain();

        powertrain.reset();

        assert!(powertrain.config.transmission.auto_clutch);
        assert!(powertrain.uses_automatic_clutch());
    }

    #[test]
    fn stalled_engine_starts_a_timed_sequence_after_a_fresh_drive_pedal_press() {
        let mut powertrain = manual_powertrain();
        engage_first_gear(&mut powertrain, VehicleInput::default());
        for _ in 0..30 {
            powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
            if !powertrain.state().engine_running {
                break;
            }
        }
        assert!(!powertrain.state().engine_running);
        assert_eq!(powertrain.engine_state(), VehicleEngineState::Stopped);
        assert_eq!(powertrain.state().engine_state_sequence, 1);

        powertrain.set_input(VehicleInput {
            throttle: 1.0,
            clutch: 1.0,
            ..VehicleInput::default()
        });
        let output = powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);

        assert!(powertrain.state().engine_starting);
        assert!(!powertrain.state().engine_running);
        assert_eq!(powertrain.engine_state(), VehicleEngineState::Starting);
        assert_eq!(powertrain.state().engine_state_sequence, 2);
        assert_eq!(powertrain.state().engine_rpm, 0.0);
        assert_eq!(output.drive_throttle, 0.0);
        assert_eq!(output.drive_torque, 0.0);

        let mut crank_rpm = 0.0;
        let mut flare_rpm = 0.0;
        for frame in 1..=59 {
            let output = powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
            assert!(powertrain.state().engine_starting);
            assert!(!powertrain.state().engine_running);
            assert_eq!(output.drive_throttle, 0.0);
            assert_eq!(output.drive_torque, 0.0);
            if frame == 30 {
                crank_rpm = powertrain.state().engine_rpm;
            }
            if frame == 43 {
                flare_rpm = powertrain.state().engine_rpm;
            }
        }

        assert!(crank_rpm > 0.0);
        assert!(crank_rpm < powertrain.config.engine.idle_rpm);
        assert!(flare_rpm > powertrain.config.engine.idle_rpm);

        let output = powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
        assert!(powertrain.state().engine_running);
        assert!(!powertrain.state().engine_starting);
        assert_eq!(powertrain.engine_state(), VehicleEngineState::Running);
        assert_eq!(powertrain.state().engine_state_sequence, 3);
        assert!(powertrain.state().engine_rpm >= powertrain.config.engine.idle_rpm);
        assert_eq!(output.drive_throttle, 1.0);
    }

    #[test]
    fn stalled_engine_requires_throttle_release_before_restarting_when_pedal_is_held() {
        let mut powertrain = manual_powertrain();
        powertrain.state.engine_running = false;
        powertrain.state.engine_rpm = 0.0;
        powertrain.restart_armed = false;
        powertrain.set_input(VehicleInput {
            throttle: 1.0,
            clutch: 1.0,
            ..VehicleInput::default()
        });

        powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
        assert!(!powertrain.state().engine_running);

        powertrain.set_input(VehicleInput::default());
        powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
        powertrain.set_input(VehicleInput {
            throttle: 1.0,
            clutch: 1.0,
            ..VehicleInput::default()
        });
        powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);

        assert!(powertrain.state().engine_starting);
        assert!(!powertrain.state().engine_running);

        for _ in 0..60 {
            powertrain.update(1.0 / 60.0, 0.0, 0.0, 0.35);
        }

        assert!(powertrain.state().engine_running);
        assert!(!powertrain.state().engine_starting);
    }

    #[test]
    fn forward_motion_in_forward_gear_bump_starts_the_engine() {
        let wheel_radius = 0.35;
        let mut powertrain = prepare_stalled_powertrain_in_gear(1, 0.0);
        let wheel_speed =
            wheel_speed_for_crank_rpm(&powertrain, powertrain.config.engine.idle_rpm, wheel_radius);

        update_until_started(&mut powertrain, wheel_speed, wheel_radius);

        assert!(powertrain.state().engine_running);
        assert!(
            powertrain.state().engine_rpm >= powertrain.config.engine.idle_rpm * STALL_RPM_RATIO
        );
    }

    #[test]
    fn backward_motion_in_reverse_gear_bump_starts_the_engine() {
        let wheel_radius = 0.35;
        let mut powertrain = prepare_stalled_powertrain_in_gear(-1, 0.0);
        let wheel_speed =
            wheel_speed_for_crank_rpm(&powertrain, powertrain.config.engine.idle_rpm, wheel_radius);
        assert!(wheel_speed < 0.0);

        update_until_started(&mut powertrain, wheel_speed, wheel_radius);

        assert!(powertrain.state().engine_running);
        assert!(
            powertrain.state().engine_rpm >= powertrain.config.engine.idle_rpm * STALL_RPM_RATIO
        );
    }

    #[test]
    fn opposing_motion_and_gear_directions_do_not_crank_the_engine() {
        let wheel_radius = 0.35;
        for (gear, movement_sign) in [(1, -1.0), (-1, 1.0)] {
            let mut powertrain = prepare_stalled_powertrain_in_gear(gear, 0.0);
            let matching_wheel_speed = wheel_speed_for_crank_rpm(
                &powertrain,
                powertrain.config.engine.idle_rpm,
                wheel_radius,
            );
            let opposing_wheel_speed = matching_wheel_speed.abs() * movement_sign;

            update_until_started(&mut powertrain, opposing_wheel_speed, wheel_radius);

            assert!(!powertrain.state().engine_running);
            assert_eq!(powertrain.state().engine_rpm, 0.0);
        }
    }

    #[test]
    fn bump_start_requires_stall_speed_and_an_engaged_clutch() {
        let wheel_radius = 0.35;
        let mut below_threshold = prepare_stalled_powertrain_in_gear(1, 0.0);
        let low_wheel_speed = wheel_speed_for_crank_rpm(
            &below_threshold,
            below_threshold.config.engine.idle_rpm * STALL_RPM_RATIO * 0.9,
            wheel_radius,
        );
        update_until_started(&mut below_threshold, low_wheel_speed, wheel_radius);
        assert!(!below_threshold.state().engine_running);
        assert!(below_threshold.state().engine_rpm > 0.0);

        let mut clutch_disengaged = prepare_stalled_powertrain_in_gear(1, 1.0);
        let starting_wheel_speed = wheel_speed_for_crank_rpm(
            &clutch_disengaged,
            clutch_disengaged.config.engine.idle_rpm,
            wheel_radius,
        );
        update_until_started(&mut clutch_disengaged, starting_wheel_speed, wheel_radius);
        assert!(!clutch_disengaged.state().engine_running);
        assert_eq!(clutch_disengaged.state().engine_rpm, 0.0);
    }
}
