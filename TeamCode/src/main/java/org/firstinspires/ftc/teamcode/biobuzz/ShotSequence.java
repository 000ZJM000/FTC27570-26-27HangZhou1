package org.firstinspires.ftc.teamcode.biobuzz;

final class ShotSequence {
    private static final double IDLE_RPM = 2500;
    private final Turret turret;
    private final Flywheel shooter;
    private final Intake intake;
    private final boolean calibratedMode;

    private boolean released;
    private boolean interrupted;
    private boolean feeding;
    private boolean blocked, turretReady, speedReady, intakeReady;

    ShotSequence(Turret turret, Flywheel shooter, Intake intake, boolean calibratedMode) {
        this.turret = turret;
        this.shooter = shooter;
        this.intake = intake;
        this.calibratedMode = calibratedMode;
    }

    boolean update(boolean triggerPressed, boolean reverseRequested, boolean noFireZone,
                   boolean trackingTurret, double targetRpm, boolean rpmLockEnabled) {
        boolean triggerHeld = triggerPressed && !reverseRequested;
        if (shooter != null) {
            shooter.sample();
            boolean prepare = (!rpmLockEnabled || triggerHeld) && !noFireZone && targetRpm > 0;
            shooter.requestRpm(calibratedMode && !prepare ? IDLE_RPM : targetRpm);
        }
        blocked = reverseRequested || noFireZone || targetRpm <= 0 || intake == null || shooter == null
                || shooter.faulted() || !shooter.rpmCalibrated() || !shooter.hasBlocker();
        boolean checkShot = triggerHeld && !blocked;
        turretReady = checkShot && (released || interrupted || !trackingTurret
                || turret == null || !turret.calibrated() || !turret.encoderReady() || !turret.motorResponding()
                || turret.aimReady() || turret.stableForShot());
        speedReady = checkShot && (!rpmLockEnabled || shooter.readyForShot(feeding));
        intakeReady = intake != null && intake.readyToFeed();
        return updateTrigger(triggerPressed, turretReady && intakeReady);
    }

    String waitingStatus() {
        if (interrupted) return "供弹已中断，松开 RT 后再按";
        if (blocked) return "检查云台、供弹机构和飞轮";
        if (!turretReady) return "等待云台对准或停稳";
        if (!speedReady) return "等待飞轮进入 −8% 至 +12% 转速区间";
        if (!intakeReady) return "等待吸取舵机抬起";
        return null;
    }

    private boolean updateTrigger(boolean triggerPressed, boolean approvalReady) {
        if (!triggerPressed) {
            releaseTrigger();
            return false;
        }
        if (blocked) {
            interrupted |= released;
            feeding = false;
            return false;
        }
        if (interrupted) return false;
        released |= approvalReady && speedReady;
        feeding = released && speedReady;
        return feeding;
    }

    void releaseTrigger() { released = interrupted = feeding = false; }
}
