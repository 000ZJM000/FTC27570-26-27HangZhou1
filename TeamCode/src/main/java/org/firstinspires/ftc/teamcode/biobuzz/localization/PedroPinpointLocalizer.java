package org.firstinspires.ftc.teamcode.biobuzz.localization;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import org.firstinspires.ftc.teamcode.biobuzz.config.RobotConfig;

/** PedroPathing 3.0 Localizer adapter backed by PinpointPose. */
public final class PedroPinpointLocalizer implements Localizer {
    public static final double MM_PER_INCH = 25.4;
    private static final double FIELD_WIDTH_MM = 6 * RobotConfig.TILE_MM;
    private final PinpointPose pinpoint;

    public PedroPinpointLocalizer(PinpointPose pinpoint) {
        this.pinpoint = pinpoint;
    }

    @Override public void setPose(Pose pose) {
        if (!pinpoint.anchor(fieldForwardMm(pose), fieldRightMm(pose), pose.heading())
                || !pinpoint.positionValid) {
            throw new IllegalStateException("Pinpoint 起点未就绪：" + pinpoint.status);
        }
    }

    public static boolean anchorRouteStart(PinpointPose pinpoint, Pose start) {
        if (!pinpoint.isReadyForAnchor()) return false;
        return pinpoint.anchor(fieldForwardMm(start), fieldRightMm(start), start.heading());
    }

    // From the red manual corner, Visualizer +X is forward and +Y is left.
    public static double fieldForwardMm(Pose pose) { return pose.x() * MM_PER_INCH; }

    public static double fieldRightMm(Pose pose) {
        return FIELD_WIDTH_MM - pose.y() * MM_PER_INCH;
    }

    @Override public MotionState state() {
        Pose pose = new Pose(pinpoint.forward / MM_PER_INCH, (FIELD_WIDTH_MM + pinpoint.left) / MM_PER_INCH,
                pinpoint.heading);
        Velocity velocity = new Velocity(pinpoint.forwardVelocity / MM_PER_INCH,
                pinpoint.leftVelocity / MM_PER_INCH, pinpoint.yawRate);
        return MotionState.ofVelocity(pose, velocity);
    }

    @Override public void update() {
        sample();
        if (!pinpoint.positionValid) throw new PositionUnavailable(pinpoint.status);
    }

    public boolean sample() {
        pinpoint.update();
        return pinpoint.positionValid;
    }

    public static final class PositionUnavailable extends IllegalStateException {
        PositionUnavailable(String status) { super("Pinpoint 定位失效：" + status); }
    }

    @Override public void reset() {
        if (!pinpoint.startImu()) throw new IllegalStateException(pinpoint.status);
    }
}
