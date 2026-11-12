#![no_std]
#![no_main]
#![forbid(unsafe_code)]

mod dm_mc02;

use core::sync::atomic::{AtomicU32, Ordering};
use dm_mc02::Can1;
use embassy_executor::Spawner;
use embassy_stm32::can::Frame;
use embassy_time::{Duration, Instant, Ticker};
use embedded_can::Id;

// This rig uses GM6020 voltage commands for motor IDs 1 to 4.
const MOTOR_COMMAND_ID: u16 = 0x1FF;
// Ticks without remote input before it counts as lost.
const REMOTE_SILENT_TICKS: u16 = 100;
const TICK: Duration = Duration::from_millis(1);
const YAW_FEEDBACK_ID :Option<u16>= None;
const PITCH_FEEDBACK_ID :Option<u16>= None;
const MOTOR_VOLTAGE_LIMIT:f32 =0.0;
const FEEDBACK_TIME_OUT_TICKS: u16=20;
const PREPARE_ANGLE_TOLERANCE_RAD: f32=0.02;
const PREPARE_SPEED_TOLERANCE_RPM: f32=2.0;
const PREPARE_STABLE_TICKS:u32=200;
const PREPARE_TIMEOUT_TICKS:u32=5000;
const PREPARE_MAX_RPM: f32 = 0.0;
const PREPARE_MAX_VOLTAGE: f32 = 0.0;
const RUNNING_MAX_RPM: f32 = 0.0;
const RUNNING_MAX_VOLTAGE: f32 = 0.0;
const STALL_MIN_TARGET_RPM: f32 = 5.0;
const STALL_MAX_ACTUAL_RPM: f32 = 1.0;
const STALL_MIN_VOLTAGE: f32 = 1000.0;
const STALL_DURATION_TICKS: u32 = 300;
const OSC_MIN_SPEED_RPM: f32 = 1.0;
const OSC_MIN_SWING_RAD: f32 = 0.02;
const OSC_MAX_FREQUENCY_HZ: f32 = 8.0;
const OSC_DURATION_TICKS: u32 = 500;

/// Command frames dropped because the CAN transmit queue was full, for example while no
/// motor acknowledges frames.
pub static TX_FAILED: AtomicU32 = AtomicU32::new(0);

/// Feedback and voltage command for one motor.
#[derive(Default)]
pub struct Motor {
    /// Single-turn motor shaft angle in rad; turns are not counted.
    pub angle_rad: f32,
    pub speed_rpm: f32,
    /// Feedback current as sent on the wire. The motor manual gives no scale to amperes.
    pub current_raw: i16,
    /// Voltage command counts, not volts; protocol range +/-25000.
    pub command_voltage_raw: f32,
}

impl Motor {
    /// Clear the voltage command in RAM. The motor receives zero when
    /// `Control::transmit` next runs, within 1 ms.
    pub fn disable(&mut self) {
        self.command_voltage_raw = 0.0;
    }

    pub fn read(&mut self, data: &[u8; 8]) {
        let angle_raw=u16::from_be_bytes([data[0],data[1]]);

        self.angle_rad=angle_raw as f32 * core::f32::consts::TAU / 8192.0;

        self.speed_rpm=i16::from_be_bytes([data[2],data[3]]) as f32;

        self.current_raw=i16::from_be_bytes([data[4],data[5]]);
    }

    pub fn command_bytes(&self, voltage_limit: f32)-> [u8;2]{
        let limit=voltage_limit.clamp(0.0,25000.0);
        let limit_command=self.command_voltage_raw.clamp(-limit,limit);
        let command=limit_command as i16;
        command.to_be_bytes()
    }
}

/// PID gains, plus the integral and history kept between ticks.
// TODO: add gains, history, an integral limit and a symmetric output limit `maxout`.
#[derive(Default)]
pub struct Pid {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub integral: f32,
    pub integral_limit: f32,
    pub maxout: f32,
    pub previous_error: f32,
    pub has_previous_error: bool,
}

impl Pid {
    // TODO: clear the integral and history.
    pub fn reset(&mut self) {
        self.integral=0.0;
        self.previous_error=0.0;
        self.has_previous_error=false;
    }

    /// `target` and `measured` share a unit, set by the caller. The position loop takes rad
    /// and returns rpm; the speed loop takes rpm and returns voltage counts. `dt_s` is in seconds.
    /// If a position loop returns rad/s, convert it to rpm before passing it to the speed loop.
    pub fn calculate(&mut self, target: f32, measured: f32, dt_s: f32) -> f32 {
        // TODO: compute the PID with dt_s, and limit the integral and output.
        let error = target - measured;
        self.integral+=error*dt_s;
        self.integral=self.integral.clamp(-self.integral_limit,self.integral_limit,);

        let derivative=if self.has_previous_error{
            (error-self.previous_error)/dt_s
        }else{0.0};
        self.previous_error=error;
        self.has_previous_error=true;

        let p_out = self.kp * error;
        let i_out = self.ki * self.integral;
        let d_out = self.kd * derivative;

        let output=p_out+i_out+d_out;
        let limited_output = output.clamp(-self.maxout, self.maxout);
        limited_output
    }
}

/// The gimbal's stage, which decides the allowed operations and outputs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    /// Zero output. Waits for valid remote input and fresh feedback from both motors,
    /// then for a new enable request.
    Waiting,
    /// Recenters both axes to the reference pose with limited speed and output.
    /// Enters `Running` once both axes settle.
    Prepare,
    /// The left stick moves each axis's `target_angle_rad`, and its cascade tracks it.
    Running,
    /// Zero output until a manual recovery.
    Error,
}

/// One gimbal axis: its motor, targets and cascaded PIDs.
#[derive(Default)]
pub struct Axis {
    pub motor: Motor,
    /// Position loop target in rad.
    pub target_angle_rad: f32,
    /// Speed loop target in rpm, set from the position loop output.
    pub target_rpm: f32,
    /// Position loop: rad in, rpm out.
    pub position_pid: Pid,
    /// Speed loop: rpm in, voltage counts out.
    pub speed_pid: Pid,
    pub reference_angle_rad: Option<f32>,
    pub min_target_angle_rad: Option<f32>,
    pub max_target_angle_rad: Option<f32>,
    pub stick_rate_rad_s: f32,
    pub stall_ticks: u32,
    pub previous_motion_direction: i8,
    pub oscillation_clock_ticks: u32,
    pub oscillation_last_turn_angle: Option<f32>,
    pub oscillation_last_cycle_tick: Option<u32>,
    pub oscillation_valid_halves: u8,
    pub oscillation_fast_ticks: u32,
    pub oscillation_frequency_hz: f32,
}

impl Axis{
    pub fn calculate_output(&mut self, dt_s: f32){
        self.target_rpm=self.position_pid.calculate(
            self.target_angle_rad,
            self.motor.angle_rad,
            dt_s,
        );
        self.motor.command_voltage_raw=self.speed_pid.calculate(
            self.target_rpm,
            self.motor.speed_rpm,
            dt_s,
        );
    }

    pub fn update_target_from_stick(&mut self, input: f32, dt_s: f32) -> bool {
        let min_angle = match self.min_target_angle_rad {
            Some(angle) => angle,
            None => return false,
        };
    
        let max_angle = match self.max_target_angle_rad {
            Some(angle) => angle,
            None => return false,
        };
    
        if !min_angle.is_finite() || !max_angle.is_finite() || min_angle >= max_angle {
            return false;
        }
    
        if !self.stick_rate_rad_s.is_finite() {
            return false;
        }
    
        if !(min_angle..=max_angle).contains(&self.target_angle_rad) {
            return false;
        }
    
        let change = input * self.stick_rate_rad_s * dt_s;
        let next_target = self.target_angle_rad + change;
    
        self.target_angle_rad = next_target.clamp(min_angle, max_angle);
    
        true
    }

    pub fn current_position_is_allowed(&self) -> bool {
        let min_angle = match self.min_target_angle_rad {
            Some(angle) => angle,
            None => return false,
        };
    
        let max_angle = match self.max_target_angle_rad {
            Some(angle) => angle,
            None => return false,
        };
    
        min_angle.is_finite()
            && max_angle.is_finite()
            && min_angle < max_angle
            && self.stick_rate_rad_s.is_finite()
            && (min_angle..=max_angle).contains(&self.motor.angle_rad)
    }

    pub fn is_stall_candidate(
        &self,
        min_target_rpm: f32,
        max_actual_rpm: f32,
        min_voltage: f32,
    ) -> bool {
        let wants_to_move = self.target_rpm.abs() > min_target_rpm;
    
        let sent_voltage =
            i16::from_be_bytes(self.motor.command_bytes(MOTOR_VOLTAGE_LIMIT));
    
        let output_large = (sent_voltage as f32).abs() > min_voltage;
        let moving_slowly = self.motor.speed_rpm.abs() < max_actual_rpm;
    
        wants_to_move && output_large && moving_slowly
    }

    pub fn update_stall_detection(&mut self) -> bool {
        let suspicious = self.is_stall_candidate(
            STALL_MIN_TARGET_RPM,
            STALL_MAX_ACTUAL_RPM,
            STALL_MIN_VOLTAGE,
        );
    
        if suspicious {
            self.stall_ticks = self.stall_ticks.saturating_add(1);
        } else {
            self.stall_ticks = 0;
        }
    
        self.stall_ticks >= STALL_DURATION_TICKS
    }

    pub fn reset_control(&mut self){
        self.target_rpm=0.0;
        self.position_pid.reset();
        self.speed_pid.reset();
        self.motor.disable();
        self.stall_ticks = 0;
        self.previous_motion_direction = 0;
        self.reset_oscillation_detection();
        
    }

    pub fn is_settled(&self, angle_tolerance_rad: f32, speed_tolerance_rpm: f32)->bool{
        let angle_error=(self.target_angle_rad-self.motor.angle_rad).abs();
        let actual_speed=self.motor.speed_rpm.abs();
        let angle_ok=angle_error< angle_tolerance_rad;
        let speed_ok=actual_speed< speed_tolerance_rpm;
        angle_ok && speed_ok
    }

    pub fn motion_direction(&self, min_speed_rpm: f32) -> i8 {
        let speed = self.motor.speed_rpm;
    
        if speed > min_speed_rpm {
            1
        } else if speed < -min_speed_rpm {
            -1
        } else {
            0
        }
    }

    pub fn detect_direction_change(&mut self, min_speed_rpm: f32) -> bool {
        let current = self.motion_direction(min_speed_rpm);
    
        if current == 0 {
            return false;
        }
    
        let changed =
            self.previous_motion_direction != 0
            && current != self.previous_motion_direction;
    
        self.previous_motion_direction = current;
    
        changed
    }

    pub fn reset_oscillation_detection(&mut self) {
        self.previous_motion_direction = 0;
        self.oscillation_clock_ticks = 0;
        self.oscillation_last_turn_angle = None;
        self.oscillation_last_cycle_tick = None;
        self.oscillation_valid_halves = 0;
        self.oscillation_fast_ticks = 0;
        self.oscillation_frequency_hz = 0.0;
    }
    
    pub fn update_oscillation_detection(&mut self) -> bool {
        self.oscillation_clock_ticks =
            self.oscillation_clock_ticks.saturating_add(1);
    
        let now = self.oscillation_clock_ticks;
    
        // 太久没有完成一次往返，就不再算连续高频摆动。
        if let Some(last) = self.oscillation_last_cycle_tick {
            let elapsed = now.saturating_sub(last);
            if elapsed as f32 > 1000.0 / OSC_MAX_FREQUENCY_HZ {
                self.oscillation_fast_ticks = 0;
                self.oscillation_frequency_hz = 0.0;
            }
        }
    
        if !self.detect_direction_change(OSC_MIN_SPEED_RPM) {
            return false;
        }
    
        let angle = self.motor.angle_rad;
    
        let amplitude_ok = match self.oscillation_last_turn_angle {
            Some(last) => (angle - last).abs() >= OSC_MIN_SWING_RAD,
            None => false,
        };
    
        self.oscillation_last_turn_angle = Some(angle);
    
        if amplitude_ok {
            self.oscillation_valid_halves =
                self.oscillation_valid_halves.saturating_add(1);
        } else {
            self.oscillation_valid_halves = 0;
            self.oscillation_fast_ticks = 0;
        }
    
        // 只在转为正向时结算周期，避免把半个往返算成一整次。
        if self.previous_motion_direction != 1 {
            return false;
        }
    
        if let Some(last) = self.oscillation_last_cycle_tick {
            let period_ticks = now.saturating_sub(last);
    
            if period_ticks > 0 && self.oscillation_valid_halves >= 2 {
                self.oscillation_frequency_hz = 1000.0 / period_ticks as f32;
    
                if self.oscillation_frequency_hz > OSC_MAX_FREQUENCY_HZ {
                    self.oscillation_fast_ticks =
                        self.oscillation_fast_ticks.saturating_add(period_ticks);
                } else {
                    self.oscillation_fast_ticks = 0;
                }
            } else {
                self.oscillation_fast_ticks = 0;
                self.oscillation_frequency_hz = 0.0;
            }
        }
    
        self.oscillation_last_cycle_tick = Some(now);
        self.oscillation_valid_halves = 0;
    
        self.oscillation_fast_ticks >= OSC_DURATION_TICKS
    }
}

/// State shared by every control tick.
// TODO: add feedback ages and a fault record.
pub struct Control {
    pub state: State,
    pub yaw: Axis,
    pub pitch: Axis,
    pub remote: Option<dm_mc02::Dr16>,
    pub remote_silent_ticks: u16,
    pub yaw_feedback_age_ticks: u16,
    pub pitch_feedback_age_ticks: u16,
    pub previous_right_switch: Option<u8>,
    pub prepare_ticks: u32,
    pub prepare_stable_ticks: u32,
    pub recovery_armed: bool,
}

/// Start in `Waiting` with no remote input.
/// Set the initial value of each field you add here, including the PID gains.
impl Default for Control {
    fn default() -> Self {
        Self {
            state: State::Waiting,
            yaw: Axis::default(),
            pitch: Axis::default(),
            remote: None,
            remote_silent_ticks: REMOTE_SILENT_TICKS,
            yaw_feedback_age_ticks: u16::MAX,
            pitch_feedback_age_ticks: u16::MAX,
            previous_right_switch: None,
            prepare_ticks:0,
            prepare_stable_ticks:0,
            recovery_armed: false,
        }
    }
}

fn fill_motor_command(data:&mut[u8;8], feedback_id:Option<u16>, command: [u8;2]){
    let id=match feedback_id{
        Some(id)=>id,
        None=>return,
    };
    if id<0x205|| id > 0x208{return;}
    let start=((id-0x205)*2) as usize;
    data[start]=command[0];
    data[start+1]=command[1];
}

fn shortest_angle_error(target:f32, actual:f32) ->f32{
    let mut error=target-actual;
    let pi = core::f32::consts::PI;
    let tau = core::f32::consts::TAU;

    if error > pi {
        error -= tau;
    } else if error < -pi {
        error += tau;
    }

    error
}

fn normalize_stick(raw: i16) -> f32 {
    let input = (raw as f32 / 660.0).clamp(-1.0, 1.0);

    if input.abs() < 0.05 {
        0.0
    } else {
        input
    }
}

impl Control {
    /// Clear both motors' voltage commands in RAM.
    pub fn disable_all(&mut self) {
        self.yaw.motor.disable();
        self.pitch.motor.disable();
    }

    pub fn check_stalls(&mut self) {
        if self.state != State::Prepare && self.state != State::Running {
            return;
        }
    
        let yaw_stalled = self.yaw.update_stall_detection();
        let pitch_stalled = self.pitch.update_stall_detection();
    
        if yaw_stalled || pitch_stalled {
            self.disable_all();
            self.state = State::Error;
    
            if yaw_stalled {
                rtt_target::rprintln!("Fault: yaw suspected stall");
            }
    
            if pitch_stalled {
                rtt_target::rprintln!("Fault: pitch suspected stall");
            }
        }
    }

    pub fn check_oscillations(&mut self) {
        if self.state != State::Prepare && self.state != State::Running {
            return;
        }
    
        let yaw_fault = self.yaw.update_oscillation_detection();
        let pitch_fault = self.pitch.update_oscillation_detection();
    
        if yaw_fault || pitch_fault {
            self.disable_all();
            self.state = State::Error;
    
            if yaw_fault {
                rtt_target::rprintln!(
                    "Fault: yaw oscillation {:.1} Hz",
                    self.yaw.oscillation_frequency_hz,
                );
            }
    
            if pitch_fault {
                rtt_target::rprintln!(
                    "Fault: pitch oscillation {:.1} Hz",
                    self.pitch.oscillation_frequency_hz,
                );
            }
        }
    }

    pub fn feedback_is_fresh(&self)->bool{
        let yaw_fresh=self.yaw_feedback_age_ticks < FEEDBACK_TIME_OUT_TICKS;
        let pitch_fresh=self.pitch_feedback_age_ticks < FEEDBACK_TIME_OUT_TICKS;
        yaw_fresh && pitch_fresh
    }

    pub fn take_enable_request(&mut self)->bool{
        let current=match self.remote{
            Some(remote)=>Some(remote.right_switch),
            None=>None,
        };
        let enable_requested=
            self.previous_right_switch==Some(2) && current==Some(3);
        self.previous_right_switch=current;
        enable_requested
    }

    pub fn take_recovery_request(&mut self) -> bool {
        if self.state != State::Error {
            self.recovery_armed = false;
            return false;
        }
    
        let remote = match self.remote {
            Some(remote) => remote,
            None => {
                self.recovery_armed = false;
                return false;
            }
        };
    
        if remote.right_switch != 3 {
            self.recovery_armed = false;
            return false;
        }
    
        if remote.left_switch == 2 {
            self.recovery_armed = true;
        } else if remote.left_switch == 1 {
            let requested = self.recovery_armed;
            self.recovery_armed = false;
    
            let stick_centered =
                normalize_stick(remote.channels[2]) == 0.0
                && normalize_stick(remote.channels[3]) == 0.0;
    
            return requested && stick_centered;
        }
    
        false
    }

    /// Runs every 1 ms: read the remote and motor feedback, update the state, then
    /// compute and send the outputs. Missing remote input stops output; a latched Error is preserved.
    pub fn tick(&mut self, can: &mut Can1) {
        self.read_remote();
        self.read_feedback(can);
        self.update_mode();
        if self.remote.is_none() && self.state != State::Error {
            self.state = State::Waiting;
        }
        self.update();
        self.check_stalls();
        self.check_oscillations();
        self.transmit(can);
    }

    /// Take the latest input published by the remote task. After an invalid frame, or
    /// `REMOTE_SILENT_TICKS` ticks without one, clear the commands; return to Waiting unless already in Error.
    pub fn read_remote(&mut self) {
        self.remote_silent_ticks = self.remote_silent_ticks.saturating_add(1);
        if let Some(remote) = dm_mc02::REMOTE.try_take() {
            self.remote = remote;
            if self.remote.is_some() {
                self.remote_silent_ticks = 0;
            }
        }
        if self.remote_silent_ticks >= REMOTE_SILENT_TICKS {
            self.remote = None;
        }
        if self.remote.is_none() {
            self.disable_all();
            if self.state != State::Error {
                self.state = State::Waiting;
            }
        }
    }

    /// Read every queued CAN frame without waiting for more.
    pub fn read_feedback(&mut self, can: &mut Can1) {
        self.yaw_feedback_age_ticks=self.yaw_feedback_age_ticks.saturating_add(1);
        self.pitch_feedback_age_ticks=self.pitch_feedback_age_ticks.saturating_add(1);
        let receiver = can.reader();
        while let Ok(received) = receiver.try_receive() {
            match received {
                Ok(envelope) => {
                    let frame = envelope.frame;
                    if frame.header().rtr() || frame.data().len()!=8{
                        continue;
                    }
                    let feedback_id=match frame.id(){
                        Id::Standard(id) => id.as_raw(),
                        Id::Extended(_)=>continue,
                    };
                    let mut data=[0u8; 8];
                    data.copy_from_slice(frame.data());
                    let angle_raw=u16::from_be_bytes([data[0],data[1]]);
                    if angle_raw>8191{continue;}
                    if Some(feedback_id)== YAW_FEEDBACK_ID{
                        self.yaw.motor.read(&data);
                        self.yaw_feedback_age_ticks=0;
                    }
                    else if Some(feedback_id)==PITCH_FEEDBACK_ID{
                        self.pitch.motor.read(&data);
                        self.pitch_feedback_age_ticks=0;
                    }

                }
                Err(error) => dm_mc02::log_can_error(error),
            }
        }
    }

    /// Choose the state from the remote, feedback and fault conditions.
    // TODO: the transitions, and initialising targets and PIDs on entering a state.
    pub fn update_mode(&mut self) {
        let enable_request=self.take_enable_request();
        let recovery_request = self.take_recovery_request();
        let enable=match self.remote{
            Some(remote)=>remote.right_switch==3,
            None=>false,
        };

        if !enable || !self.feedback_is_fresh(){
            self.disable_all();
            if self.state!=State::Error{
                self.state=State::Waiting;
            }
            return;
        }

        match self.state {
            State::Waiting => {
                let reference_ready=
                    self.yaw.reference_angle_rad.is_some()
                    && self.pitch.reference_angle_rad.is_some();

                if enable_request && self.feedback_is_fresh() && reference_ready{
                    self.yaw.reset_control();
                    self.pitch.reset_control();

                    if let Some(angle)=self.yaw.reference_angle_rad{
                        self.yaw.target_angle_rad=angle;
                    }
                    if let Some(angle)=self.pitch.reference_angle_rad{
                        self.pitch.target_angle_rad=angle;
                    }
                    self.prepare_ticks=0;
                    self.prepare_stable_ticks=0;
                    self.state=State::Prepare;
                }
            }
            State::Prepare => {
                self.prepare_ticks=self.prepare_ticks.saturating_add(1);
                let yaw_settled=self.yaw.is_settled(
                    PREPARE_ANGLE_TOLERANCE_RAD,
                    PREPARE_SPEED_TOLERANCE_RPM,
                );
                let pitch_settled=self.pitch.is_settled(
                    PREPARE_ANGLE_TOLERANCE_RAD,
                    PREPARE_SPEED_TOLERANCE_RPM,
                );
                if yaw_settled && pitch_settled{
                    self.prepare_stable_ticks=
                        self.prepare_stable_ticks.saturating_add(1);
                }
                else{
                    self.prepare_stable_ticks=0;
                }

                if self.prepare_ticks>=PREPARE_TIMEOUT_TICKS{
                    self.disable_all();
                    self.state=State::Error;
                    rtt_target::rprintln!("Prepare timeout");
                }
                else if self.prepare_stable_ticks>=PREPARE_STABLE_TICKS{
                    self.state=State::Running;
                    rtt_target::rprintln!("Prepare complete");
                }
            }
            State::Running => {
                // TODO: handle disable/feedback loss; stall or sustained high-frequency oscillation enters Error.
            }
            State::Error => {
                if recovery_request {
                    let yaw_allowed = self.yaw.current_position_is_allowed();
                    let pitch_allowed = self.pitch.current_position_is_allowed();
            
                    if yaw_allowed && pitch_allowed {
                        self.yaw.target_angle_rad = self.yaw.motor.angle_rad;
                        self.pitch.target_angle_rad = self.pitch.motor.angle_rad;
            
                        self.yaw.reset_control();
                        self.pitch.reset_control();
            
                        self.prepare_ticks = 0;
                        self.prepare_stable_ticks = 0;
            
                        self.state = State::Running;
            
                        rtt_target::rprintln!(
                            "Recovered: yaw target={:.3}, pitch target={:.3}; histories reset",
                            self.yaw.target_angle_rad,
                            self.pitch.target_angle_rad,
                        );
                    } else {
                        rtt_target::rprintln!(
                            "Recovery refused: check angle limits and current positions"
                        );
                    }
                }
            }
        }
    }

    /// Set targets for the current state, run each axis's PIDs and write the voltage commands.
    // TODO: limited recentering in Prepare, remote cascade control in Running, and
    // `disable_all` in Waiting and Error. Each tick is 0.001 s.
    pub fn update(&mut self) {
        match self.state {
            State::Waiting => {
                self.disable_all();
            }
    
            State::Prepare => {
                self.yaw.position_pid.maxout = PREPARE_MAX_RPM;
                self.pitch.position_pid.maxout = PREPARE_MAX_RPM;
    
                self.yaw.speed_pid.maxout = PREPARE_MAX_VOLTAGE;
                self.pitch.speed_pid.maxout = PREPARE_MAX_VOLTAGE;
    
                self.yaw.calculate_output(0.001);
                self.pitch.calculate_output(0.001);
            }
    
            State::Running => {
                self.yaw.position_pid.maxout = RUNNING_MAX_RPM;
                self.pitch.position_pid.maxout = RUNNING_MAX_RPM;
    
                self.yaw.speed_pid.maxout = RUNNING_MAX_VOLTAGE;
                self.pitch.speed_pid.maxout = RUNNING_MAX_VOLTAGE;

                let remote = match self.remote {
                    Some(remote) => remote,
                    None => {
                        self.disable_all();
                        self.state = State::Waiting;
                        return;
                    }
                };
                
                let yaw_input = normalize_stick(remote.channels[2]);
                let pitch_input = normalize_stick(remote.channels[3]);
                
                let yaw_ok = self.yaw.update_target_from_stick(yaw_input, 0.001);
                let pitch_ok = self.pitch.update_target_from_stick(pitch_input, 0.001);
                
                if !yaw_ok || !pitch_ok {
                    self.disable_all();
                    self.state = State::Waiting;
                    return;
                }
    
                self.yaw.calculate_output(0.001);
                self.pitch.calculate_output(0.001);
            }
    
            State::Error => {
                self.disable_all();
            }
        }
    }



    /// Queue one group command frame on CAN1 without waiting. This runs every tick, so a
    /// cleared command reaches the motor within 1 ms. A full queue drops the frame and
    /// counts it in `TX_FAILED`.
    pub fn transmit(&mut self, can: &mut Can1) {
        let mut data = [0u8; 8];
        let yaw_command=self.yaw.motor.command_bytes(MOTOR_VOLTAGE_LIMIT);
        let pitch_command=self.pitch.motor.command_bytes(MOTOR_VOLTAGE_LIMIT);
        fill_motor_command(&mut data, YAW_FEEDBACK_ID, yaw_command);
        fill_motor_command(&mut data, PITCH_FEEDBACK_ID, pitch_command);
        // TODO: clamp each voltage count output within the bench limit (protocol +/-25000),
        // encode signed i16 big-endian and write it into the
        // bytes for its motor ID as the manual describes. Other motor IDs stay zero.
        let frame = Frame::new_standard(MOTOR_COMMAND_ID, &data).unwrap();
        if can.writer().try_write(frame).is_err() {
            TX_FAILED.fetch_add(1, Ordering::Relaxed);
        }
    }
}

// Set up RTT and the board, then start the remote and control tasks.
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    rtt_target::rtt_init_print!(rtt_target::ChannelMode::NoBlockTrim, 1024);
    let (can, watchdog, remote) = dm_mc02::init();
    rtt_target::rprintln!("Gimbal control started");
    spawner.spawn(dm_mc02::remote_task(remote).unwrap());
    spawner.spawn(control_task(can, watchdog).unwrap());
}

// Run `tick` at 1 kHz and feed the watchdog after each one. If a tick hangs, the watchdog
// resets the MCU after 100 ms.
#[embassy_executor::task]
async fn control_task(mut can: Can1, mut watchdog: dm_mc02::Watchdog) {
    let mut control = Control::default();
    let mut ticker = Ticker::every(TICK);
    let mut last = Instant::now();
    loop {
        ticker.next().await;
        // After a stall, such as the 20 ms halt when a debug probe attaches, start the
        // schedule again instead of running every missed tick back to back.
        let now = Instant::now();
        if now - last > TICK * 2 {
            ticker.reset();
        }
        last = now;
        control.tick(&mut can);
        watchdog.pet();
    }
}
