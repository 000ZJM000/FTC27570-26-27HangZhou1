package org.firstinspires.ftc.teamcode.biobuzz.subsystems;

import org.firstinspires.ftc.teamcode.biobuzz.localization.PinpointPose;

/** Common interface shared by legacy and DECODE turret controllers. */
public interface TurretController {
    boolean calibrated();
    boolean isHomed();
    boolean outOfTravel();
    boolean softHoming();
    boolean encoderReady();
    boolean motorResponding();

    double span();
    int homeTick();
    int lastTick();
    double boreRelative();
    double initRelativeAngle();

    double angle();
    double speed();
    double target();
    double worldError();
    double output();
    double actualDuty();

    boolean isInheritedHomePlausible(int savedHomeTick, boolean hasLastTick, int lastKnownTick);
    void homeAtRight();
    void homeAtRight(int right);
    void teachingMode(boolean free);

    boolean startSoftHome();
    void cancelSoftHome();
    Turret.SoftHomeResult updateSoftHome(double dt);

    void sample(double dt);
    void lockAtForward(double dt);
    boolean atForwardZero();
    void aim(PinpointPose pose, int goalNumber, double bearing, double lineRate, double yawTrim, double stick, double dt);

    boolean aimReady();
    boolean aimReady(double tolerance);
    double shotAimTolerance();
    boolean stableForShot();

    boolean rewinding();
    boolean rewindPredicted();
    boolean targetReachable();

    void stop();
}
