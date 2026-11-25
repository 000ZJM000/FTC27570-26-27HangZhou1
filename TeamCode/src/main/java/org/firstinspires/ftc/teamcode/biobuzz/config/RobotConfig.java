package org.firstinspires.ftc.teamcode.biobuzz.config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.ErrorDetectionType;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/** Robot hardware mapping, CAD dimensions, and FTC Dashboard-tunable match parameters. */
@Config
public final class RobotConfig {
    // Expansion Hub 2: m-1 (0), m-2 (1), m-7 (2), m-8 (3); SL (0), ST (1); Pinpoint I2C bus 1.
    // Control Hub: m-3 (0), m-4 (1), m-5 (2), m-6 (3); SB (0), SR (5); internal IMU I2C bus 0.
    public static final String[] DRIVE = {"m-1", "m-2", "m-3", "m-4"};
    public static final Direction[] DRIVE_DIRECTIONS = {
            Direction.REVERSE, Direction.REVERSE, Direction.FORWARD, Direction.FORWARD
    };
    public static final String INTAKE = "m-5";
    public static final Direction INTAKE_DIRECTION = Direction.REVERSE;
    public static final String TURRET_MOTOR = "m-6";
    public static final String FLYWHEEL_LEFT = "m-7", FLYWHEEL_ENCODER_MOTOR = "m-8";
    public static final Direction FLYWHEEL_LEFT_DIRECTION = Direction.FORWARD;
    public static final Direction FLYWHEEL_ENCODER_DIRECTION = Direction.REVERSE;
    public static final String INTAKE_LEFT_SERVO = "SL", INTAKE_RIGHT_SERVO = "SR";
    public static final String BLOCKER_SERVO = "SB", PITCH_SERVO = "ST";
    public static final String TASK_RIGHT_SERVO = "FR", TASK_LEFT_SERVO = "FL";
    // Expansion Hub encoder port 0 shares its configuration name with the left front drive motor.
    public static final String TURRET_ENCODER = "m-1";
    public static final String PINPOINT = "pinpoint";
    public static final String HUB_IMU = "imu";
    public static final String REAR_STATUS_LED = "LED";

    // REV Through Bore Encoder V2 (REV-11-3174), on the shaft after the 5.2:1 gearbox.
    public static final double TURRET_ENCODER_TICKS_PER_REV = 8192.0;
    public static final double TURRET_GEARBOX_RATIO = 5.2;
    public static final int TURRET_PINION_TEETH = 58, TURRET_RING_TEETH = 230;
    public static final double TICKS_PER_TURRET_RADIAN = TURRET_ENCODER_TICKS_PER_REV
            * TURRET_RING_TEETH / TURRET_PINION_TEETH / (2 * Math.PI);
    public static final double TILE_MM = 609.6;
    public static final double ROBOT_WIDTH_MM = 438.6;
    public static final double ROBOT_LENGTH_MM = 440.47468;
    public static final double CENTER_FROM_LEFT_EDGE_MM = 219.3;
    public static final double CENTER_FROM_RIGHT_EDGE_MM = 219.3;
    public static final double CENTER_FROM_REAR_EDGE_MM = 177.47468;
    public static final double CENTER_FROM_FRONT_EDGE_MM = ROBOT_LENGTH_MM - CENTER_FROM_REAR_EDGE_MM;
    // CAD convention: right and forward positive, measured from the mecanum center.
    public static final double X_POD_RIGHT_MM = -34.8;
    public static final double X_POD_FORWARD_MM = -160.37497;
    public static final double Y_POD_RIGHT_MM = 79.82503;
    public static final double Y_POD_FORWARD_MM = 78.0;
    public static final double PINPOINT_RIGHT_MM = 93.2;
    public static final double PINPOINT_FORWARD_MM = 83.0;
    public static final double TURRET_FORWARD_MM = -48.95762;
    public static final double TURRET_RIGHT_MM = 0;

    // X must count up forward and Y must count up left.
    public static final EncoderDirection X_POD_DIRECTION = EncoderDirection.REVERSED;
    public static final EncoderDirection Y_POD_DIRECTION = EncoderDirection.REVERSED;
    public static final double TURRET_MARGIN = Math.toRadians(4);
    public static final double TURRET_MAX_POWER = 1.0;
    public static final double TURRET_MAX_RATE = 10.0;
    public static final double FLYWHEEL_MAX_POWER = 1.0;
    public static final double FLYWHEEL_RAMP = 2.5;
    public static final double FLYWHEEL_TARGET_MAX_DUTY = 0.96;
    public static final double FLYWHEEL_SPEED_TOLERANCE = 0.02;
    public static final double SHOT_AIM_TOLERANCE = Math.toRadians(2.5);
    public static final double SHOT_RPM_TOLERANCE = 0.10;
    public static final double SHOT_TURRET_STILL_RATE = 0.13;
    public static final double SHOT_TURRET_STILL_SECONDS = 0.10;
    public static final double FEED_POWER = 1.0;
    public static final double INTAKE_RAISE_SETTLE_SECONDS = 0.18;

    // Optional measured defaults. Flywheel calibration can fill in the encoder scale on the Hub.
    public static final double FLYWHEEL_TICKS_PER_WHEEL_REV = 0;

    public static final double INTAKE_POWER = 1.0;
    public static final double INTAKE_REVERSE_POWER = 1.0;
    public static final double SL_INTAKE = 0.95, SR_INTAKE = 0.035;
    public static final double SL_UP = 0.825, SR_UP = 0.16;
    public static final double SL_FULL_UP = 0.68, SR_FULL_UP = 0.31; // reserved only; do not wire into behavior yet
    public static final double SB_IDLE = 0.25, SB_FIRE = 0.60;
    public static final double ST_HOME = 0.47, ST_HIGH = 0.05;
    public static final double FR_INIT = 0.855, FR_HOME = 0.98, FR_OTHER = 0.0;
    public static final double FL_INIT = 0.155, FL_HOME = 0.055, FL_OTHER = 1.0;

    // =========================================================================
    // FTC Dashboard (@Config) Tunable Match & SOTM Parameters (Must NOT be final)
    // =========================================================================
    // Pinpoint + Control Hub IMU joint fallback mode (Hub IMU fills heading/yawRate only on Pinpoint bad reads)
    public static boolean ENABLE_HUB_IMU_BACKUP = true;
    public static LogoFacingDirection HUB_IMU_LOGO_DIR = LogoFacingDirection.RIGHT;
    public static UsbFacingDirection HUB_IMU_USB_DIR = UsbFacingDirection.UP;

    public static boolean ENABLE_MOVING_LEAD = true;
    public static double SHOT_RELEASE_SECONDS = 0.140;
    public static double FLIGHT_NEAR_MM = 1000.0;
    public static double FLIGHT_NEAR_SECONDS = 0.34;
    public static double FLIGHT_FAR_MM = 3500.0;
    public static double FLIGHT_FAR_SECONDS = 0.78;
    public static double BRAKE_FORWARD_MM_S2 = 2800.0;
    public static double BRAKE_SIDE_MM_S2 = 2000.0;
    public static double SOTM_LEAD_SCALE = 1.0;
    public static double SOTM_MAX_LEAD_MM = 1350.0;
    public static double TURRET_REWIND_LOOKAHEAD_SCALE = 1.0;

    // Optional per-half shooting offsets (degrees / RPM, added after half-field mirroring)
    public static double SHOT_LEFT_YAW_OFFSET_DEG = 0.0;
    public static double SHOT_RIGHT_YAW_OFFSET_DEG = 0.0;
    public static double SHOT_LEFT_RPM_OFFSET = 0.0;
    public static double SHOT_RIGHT_RPM_OFFSET = 0.0;

    public static double FLYWHEEL_TARGET_RPM = 3100.0;
    public static double FLYWHEEL_RPM_STEP = 100.0;

    // TeleOp drive behavior & full-power options
    public static boolean ENABLE_DRIVE_LOW_VOLT_CAP = false;
    public static boolean DRIVE_SLEW_ENABLED = false;
    public static boolean ENABLE_TURN_LINEARITY = true;

    // Gamepad 2 tuning step sizes
    public static double G2_RPM_FINE_STEP = 25.0;
    public static double G2_YAW_FINE_DEG = 0.5;
    public static double G2_YAW_LARGE_DEG = 5.0;

    // Autonomous timing & Pedro 3.0 Foresight gains
    public static double AUTO_STALL_SECONDS = 1.5;
    public static double AUTO_STALL_SPEED_IPS = 0.5;
    public static double AUTO_STALL_YAW_DEG_S = 4.0;
    public static double AUTO_VOLLEY_FEED_SECONDS = 0.90;
    public static double AUTO_VOLLEY_MAX_EXTRA_SECONDS = 1.50;
    public static double AUTO_STARTUP_SPINUP_SECONDS = 0.60;
    public static double AUTO_RPM_SETTLE_SECONDS = 0.10;
    public static double AUTO_LEG4_SHOOT_DELAY_SECONDS = 0.30;
    public static int AUTO_LEG6_BUMP_CYCLES = 3;
    public static double AUTO_LEG6_BACK_SECONDS = 0.20;
    public static double AUTO_LEG6_PUSH_SECONDS = 1.00;
    public static double AUTO_LEG6_BUMP_POWER = 0.70;
    public static double AUTO_FRONT_LOWER_SETTLE_SECONDS = 0.45;
    public static double AUTO_POS_TOLERANCE_IN = 1.5;
    public static double AUTO_HEADING_TOLERANCE_DEG = 7.0;

    public static double PEDRO_FWD_P = 0.18;
    public static double PEDRO_STRAFE_P = 0.21;
    public static double PEDRO_HEADING_P = 2.0;
    public static double PEDRO_COAST_S = 0.08;
    public static double PEDRO_COAST_P = 0.016;
    public static double PEDRO_BRAKE_P = 0.018;
    public static double PEDRO_MAX_FWD_VEL = 85.0;
    public static double PEDRO_MAX_STRAFE_VEL = 75.0;
    public static double PEDRO_MAX_DECEL = 0.0; // <= 0 uses ForesightConfig.Constraint.NONE for full 1.0 cruise speed
    public static double PEDRO_MAX_BRAKE_POWER = 0.65;
    public static double PEDRO_HEADING_DEV_TOL_DEG = 28.0;
    public static double PEDRO_TRANS_DEV_TOL_IN = 6.0;
    public static double PEDRO_HOLD_TRANS_SCALE = 0.75;
    public static double PEDRO_HOLD_HEADING_SCALE = 0.65;
    public static double PEDRO_HEADING_DRIVE_RATIO = 0.35;

    /** Configures Pinpoint parameters without resetting position or IMU calibration. */
    public static int configurePinpoint(GoBildaPinpointDriver device) {
        int version = device.getDeviceVersion();
        if (version < 1) throw new IllegalStateException("Pinpoint firmware version unavailable");
        device.setErrorDetectionType(version >= 3 ? ErrorDetectionType.CRC : ErrorDetectionType.LOCAL_TEST);
        device.setOffsets(-X_POD_RIGHT_MM, Y_POD_FORWARD_MM, DistanceUnit.MM);
        device.setEncoderDirections(X_POD_DIRECTION, Y_POD_DIRECTION);
        device.setEncoderResolution(GoBildaOdometryPods.goBILDA_4_BAR_POD);
        return version;
    }

    private RobotConfig() {}
}
