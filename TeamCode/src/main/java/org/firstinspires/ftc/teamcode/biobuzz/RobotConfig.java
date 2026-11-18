package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.ErrorDetectionType;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public final class RobotConfig {
    // Expansion Hub 2: m-1 (0), m-2 (1), m-7 (2), m-8 (3); SL (0), ST (1); Pinpoint I2C bus 0.
    // Control Hub: m-3 (0), m-4 (1), m-5 (2), m-6 (3); SB (0), SR (5); internal IMU I2C bus 0.
    public static final String[] DRIVE = {"m-1", "m-2", "m-3", "m-4"};
    public static final Direction[] DRIVE_DIRECTIONS = {
            Direction.REVERSE, Direction.REVERSE, Direction.FORWARD, Direction.FORWARD
    };
    public static final String INTAKE = "m-5";
    public static final Direction INTAKE_DIRECTION = Direction.REVERSE;
    public static final String TURRET_MOTOR = "m-6";
    public static final String FLYWHEEL_LEFT = "m-7", FLYWHEEL_ENCODER_MOTOR = "m-8";
    public static final Direction FLYWHEEL_LEFT_DIRECTION = Direction.REVERSE;
    public static final Direction FLYWHEEL_ENCODER_DIRECTION = Direction.FORWARD;
    public static final String INTAKE_LEFT_SERVO = "SL", INTAKE_RIGHT_SERVO = "SR";
    public static final String BLOCKER_SERVO = "SB", PITCH_SERVO = "ST";
    // Expansion Hub port 1 shares its configuration name with the left rear DRIVE MOTOR.
    public static final String TURRET_ENCODER = "m-2";
    public static final String PINPOINT = "pinpoint";
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
    // goBILDA 4-Bar resolution. X must count up forward and Y must count up left.
    public static final double POD_TICKS_PER_MM = 19.89436789;
    public static final EncoderDirection X_POD_DIRECTION = EncoderDirection.REVERSED;
    public static final EncoderDirection Y_POD_DIRECTION = EncoderDirection.REVERSED;
    public static final double TURRET_MARGIN = Math.toRadians(4);
    public static final double TURRET_MAX_POWER = 1.0;
    public static final double TURRET_MAX_RATE = 10.0;
    public static final double FLYWHEEL_MAX_POWER = 1.0;
    public static final double FLYWHEEL_RAMP = 2.5;
    public static final double FLYWHEEL_IDLE_RPM = 4000;
    public static final double FLYWHEEL_SHOOT_RPM = 5200;
    public static final double FLYWHEEL_SPEED_TOLERANCE = 0.02;
    public static final double SHOT_AIM_TOLERANCE = Math.toRadians(2.5);
    public static final long SHOT_FEED_DELAY_MS = 300;
    public static final double FEED_POWER = 0.8;

    // Optional measured defaults. Flywheel calibration can fill in the encoder scale on the Hub.
    public static final double FLYWHEEL_TICKS_PER_WHEEL_REV = 0;
    public static final double GOAL_ONE_RPM = 0;
    public static final double GOAL_TWO_RPM = 0;

    public static final double INTAKE_POWER = 1.0;
    public static final double INTAKE_REVERSE_POWER = 1.0;
    public static final double SL_INTAKE = 0.97, SR_INTAKE = 0.02;
    public static final double SL_UP = 0.825, SR_UP = 0.16;
    public static final double SB_IDLE = 0.25, SB_FIRE = 0.60;
    public static final double ST_HOME = 0.47, ST_HIGH = 0.05;

    static int configurePinpoint(GoBildaPinpointDriver device) {
        int version = device.getDeviceVersion();
        if (version < 1) throw new IllegalStateException("Pinpoint firmware version unavailable");
        device.setErrorDetectionType(version >= 3 ? ErrorDetectionType.CRC : ErrorDetectionType.LOCAL_TEST);
        device.setOffsets(-X_POD_RIGHT_MM, Y_POD_FORWARD_MM, DistanceUnit.MM);
        device.setEncoderDirections(X_POD_DIRECTION, Y_POD_DIRECTION);
        device.setEncoderResolution(POD_TICKS_PER_MM, DistanceUnit.MM);
        return version;
    }

    private RobotConfig() {}
}
