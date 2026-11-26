package org.firstinspires.ftc.teamcode.biobuzz.subsystems;

import com.acmerobotics.dashboard.config.Config;

/** FTC Dashboard tunable parameters for the ported DECODE turret control algorithm (12 V normalized). */
@Config
public final class DecodeTurretConfig {
    // false: use CalibrationStore.DECODE_TURRET_PARAMS (saved by 02B) if present, else fallback below.
    // true: force using the live Dashboard FF/PB values below instead of saved calibration.
    public static boolean USE_DASHBOARD_FF_OVERRIDE = false;

    // Outer position (deg) -> target velocity (deg/s) PIDF gains (DECODE original defaults).
    public static double TURRET_kP = 20.0;
    public static double TURRET_kI = 0.0;
    public static double TURRET_kD = 0.0;
    public static double TURRET_kF = 0.0;

    // 12 V normalized feedforward & predictive braking defaults (DECODE 13.84 V values scaled to 12 V as initial fallback).
    public static double TURRET_kS = 0.0;
    public static double TURRET_kV = 0.000803;
    public static double TURRET_kA = 0.000031;
    public static double TURRET_kLinearBraking = 0.081114;
    public static double TURRET_kQuadraticFriction = 0.00000;
    public static double TUNING_VOLTAGE = 12.0;

    // DECODE filtering, latency compensation, and unwind tuning.
    public static double TURRET_LATENCY = 0.012;
    public static double TURRET_MAX_POWER = 1.0;
    public static double TURRET_FILTER_ALPHA = 0.8;
    public static double TURRET_VEL_FILTER_ALPHA = 0.8;
    public static double TURRET_CMD_ACCEL_FILTER_ALPHA = 0.15;
    public static double OMEGA_FILTER_ALPHA = 0.7;
    public static double HEADING_CORRECTION_ALPHA = 0.5;

    public static double KS_VEL_DEADBAND_DEG_S = 0.5;
    public static double ANGLE_DEADBAND_DEG = 0.0;
    public static double UNWIND_EXIT_ANGLE_DEG = 15.0;
    public static double UNWIND_EXIT_VEL_DEG_S = 20.0;
    public static double SHOT_AIM_TOLERANCE_DEG = 3.0;
    public static boolean ENABLE_SOFT_LIMIT_GUARD = true;

    private DecodeTurretConfig() {}
}
