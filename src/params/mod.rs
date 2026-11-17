//! 云台调参配置：以后需要调整的参数集中在这里。
//! 本次仅整理配置，所有数值保持原样。
//! 修改后需重新编译、烧录；修改前先用遥控器停用输出。
//! 控制周期固定为 1 ms；下方所有 *_TICKS 均以 1 ms 为单位。
//! 电压输出是协议原始值，不是伏特。

use embassy_time::Duration;

// ===== 1. 常用 PID 参数 =====
// 位置环：角度误差 rad → 目标速度 rpm。
// 速度环：速度误差 rpm → 电压指令原始值。

// ===== Pitch（抬头/低头）：目前实际启用的轴 =====
pub const PITCH_POSITION_KP: f32 = 15.0; // rpm/rad
pub const PITCH_SPEED_KP: f32 = 200.0;
pub const PITCH_SPEED_KI: f32 = 6000.0; // output counts / (rpm * second)
pub const PITCH_I_LIMIT_RAW: f32 = 10000.0; // 积分项输出的限幅，不是积分状态限幅
pub const PITCH_MAX_RPM: f32 = 10.0;
pub const PITCH_ACCEL_RPM_S: f32 = 4.0;
pub const PITCH_OUTPUT_RAMP_RAW_S: f32 = 7000.0; // 输出上限每秒增加 1000，9 秒达到 9000

pub const PITCH_POSITION_KI: f32 = 0.0;
pub const PITCH_POSITION_KD: f32 = 0.0;
pub const PITCH_POSITION_I_LIMIT_RPM: f32 = 0.0;
pub const PITCH_SPEED_KD: f32 = 0.0;

// Yaw（左右转动）：目前输出关闭，保留原有增益。
pub const YAW_POSITION_KP: f32 = 85.0;
pub const YAW_POSITION_KI: f32 = 0.0;
pub const YAW_POSITION_KD: f32 = 0.0;
pub const YAW_POSITION_I_LIMIT_RPM: f32 = 0.0;
pub const YAW_SPEED_KP: f32 = 320.0;
pub const YAW_SPEED_KI: f32 = 0.0;
pub const YAW_SPEED_KD: f32 = 0.0;
pub const YAW_SPEED_I_LIMIT_RAW: f32 = 0.0;
// Prepare 和 Running 每周期都使用此值。0 表示 yaw 无输出。
pub const YAW_SPEED_MAXOUT_RAW: f32 = 6000.0;

// ===== 2. 测试模式与阶段限幅 =====
// 仅跳过 yaw 回正完成判定；yaw 是否输出还由下方限幅控制。
//pub const PITCH_ONLY_TEST: bool = false;//
pub const MOTOR_VOLTAGE_LIMIT: f32 = 12000.0;
// yaw 回正目标速度限幅（pitch 使用 PITCH_MAX_RPM）。
pub const PREPARE_MAX_RPM: f32 = 15.0;
pub const PREPARE_MAX_VOLTAGE: f32 = 12000.0;
// yaw 运行目标速度限幅（pitch 使用 PITCH_MAX_RPM）。
pub const RUNNING_MAX_RPM: f32 = 10.0;
pub const RUNNING_MAX_VOLTAGE: f32 = 12000.0;

// ===== 3. 回正判定 =====
pub const PITCH_REST_ANGLE_RAD: f32 = 0.826049;
pub const PITCH_REST_TOLERANCE_RAD: f32 = 2.0 * core::f32::consts::TAU / 8192.0;
pub const PREPARE_ANGLE_TOLERANCE_RAD: f32 = 0.02;
pub const PREPARE_SPEED_TOLERANCE_RPM: f32 = 2.0;
pub const PREPARE_STABLE_TICKS: u32 = 200;
pub const PREPARE_TIMEOUT_TICKS: u32 = 30000;

// ===== 4. 堵转检测 =====
pub const STALL_MIN_TARGET_RPM: f32 = 1.0;
pub const STALL_MAX_ACTUAL_RPM: f32 = 1.0;
pub const STALL_MIN_VOLTAGE: f32 = 8500.0; // 输出较小时不计为堵转
pub const STALL_DURATION_TICKS: u32 = 1000;

// ===== 5. 振荡检测 =====
pub const OSC_MIN_SPEED_RPM: f32 = 1.0;
pub const OSC_MIN_SWING_RAD: f32 = 0.02;
pub const OSC_MAX_FREQUENCY_HZ: f32 = 8.0;
pub const OSC_DURATION_TICKS: u32 = 500;

// ===== 6. 通信配置（一般无需随 PID 调整） =====
pub const MOTOR_COMMAND_ID: u16 = 0x1FF;
pub const YAW_FEEDBACK_ID: Option<u16> = Some(0x205);
pub const PITCH_FEEDBACK_ID: Option<u16> = Some(0x206);
pub const REMOTE_SILENT_TICKS: u16 = 100;
pub const FEEDBACK_TIME_OUT_TICKS: u16 = 20;
pub const TICK: Duration = Duration::from_millis(1);

// ===== 7. 零位和目标角度范围（按实物标定） =====
// 与原代码一致：目标和反馈仍使用电机绝对角度 rad。
pub const YAW_REFERENCE_ANGLE_RAD: f32 = 1508.0 * core::f32::consts::TAU / 8192.0;
pub const YAW_MIN_TARGET_ANGLE_RAD: f32 = YAW_REFERENCE_ANGLE_RAD - core::f32::consts::FRAC_PI_2;
pub const YAW_MAX_TARGET_ANGLE_RAD: f32 = YAW_REFERENCE_ANGLE_RAD + core::f32::consts::FRAC_PI_2;
pub const PITCH_REFERENCE_ANGLE_RAD: f32 = 1480.0 * core::f32::consts::TAU / 8192.0;
pub const PITCH_MIN_TARGET_ANGLE_RAD: f32 = PITCH_REFERENCE_ANGLE_RAD - 0.25;
pub const PITCH_MAX_TARGET_ANGLE_RAD: f32 = PITCH_REFERENCE_ANGLE_RAD + 0.18;

// ===== 8. 遥控目标变化 =====
pub const STICK_DEADZONE: f32 = 0.05;
// 保留原来 Axis::default() 的零值，因此目前摇杆不移动目标。
// 带符号的 rad/s；正负号决定摇杆与目标变化的对应方向。
pub const YAW_STICK_RATE_RAD_S: f32 = 0.0;
pub const PITCH_STICK_RATE_RAD_S: f32 = 0.0;



// ===== 9. Pitch 到位积分死区（只影响 Pitch；角度单位 rad） =====
// 连续 200 ms 位于进入范围且足够慢，才冻结积分；已有积分支撑输出保留。
// 退出范围稍大，避免在边界反复切换；目标角度改变也会退出。
pub const PITCH_HOLD_ENTER_RAD: f32 = 0.015; // 15 mrad，约 0.86 度
pub const PITCH_HOLD_EXIT_RAD: f32 = 0.020; // 20 mrad，约 1.15 度
pub const PITCH_HOLD_ENTER_RPM: f32 = 1.0;
pub const PITCH_HOLD_EXIT_RPM: f32 = 2.0;
pub const PITCH_HOLD_STABLE_TICKS: u32 = 200;
