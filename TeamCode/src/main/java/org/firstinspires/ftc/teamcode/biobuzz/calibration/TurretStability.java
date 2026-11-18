package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import org.firstinspires.ftc.teamcode.biobuzz.RobotConfig;
import org.firstinspires.ftc.teamcode.biobuzz.Turret;

final class TurretStability {
    private static final double HOLD = 0.22;
    private static final double ERROR_LIMIT = Math.toRadians(1.4);
    private static final double RATE_LIMIT = Math.toRadians(10);
    private static final double OVERSHOOT_LIMIT = Math.toRadians(3.5);

    private TurretStability() {}

    static Result step(CalibrationOp op, Turret turret, double target, String label)
            throws InterruptedException {
        return step(op, turret, target, label, 2.35, false, true);
    }

    static Result pidStep(CalibrationOp op, Turret turret, double target, String label, boolean longMove,
                          boolean verification)
            throws InterruptedException {
        return step(op, turret, target, label, longMove ? 2.45 : 1.55, !verification, false);
    }

    private static Result step(CalibrationOp op, Turret turret, double target, String label, double duration,
                               boolean pidSearch, boolean baseline)
            throws InterruptedException {
        double initial = turret.angle;
        double delta = target - initial;
        double direction = Math.copySign(1, delta);
        Result result = new Result(duration);
        double scale = direction * Math.max(Math.toRadians(2), Math.abs(delta));
        try {
            op.runFor(duration, (time, dt) -> {
                turret.sample(dt);
                if (!turret.encoderReady() || !turret.motorResponding()) {
                    result.readFailed = true;
                    return false;
                }
                if (turret.angle < RobotConfig.TURRET_MARGIN
                        || turret.angle > turret.span() - RobotConfig.TURRET_MARGIN) {
                    result.unsafe = true;
                    result.limitReached = true;
                    return false;
                }
                double error = target - turret.angle;
                result.endError = error;
                result.endSpeed = turret.speed;
                result.overshoot = Math.max(result.overshoot, -direction * error);
                if (result.overshoot > Math.toRadians(8)) {
                    result.unsafe = true;
                    return false;
                }
                int sign = Math.abs(turret.speed) >= RATE_LIMIT ? (turret.speed > 0 ? 1 : -1) : 0;
                if (sign != 0) {
                    if (result.lastDirection != 0 && sign != result.lastDirection) result.reversals++;
                    result.lastDirection = sign;
                }
                if (Math.abs(error) < ERROR_LIMIT && Math.abs(turret.speed) < RATE_LIMIT) {
                    result.quiet += dt;
                } else {
                    result.quiet = 0;
                }
                turret.control(target, 0, dt);
                result.peakPower = Math.max(result.peakPower, Math.abs(turret.output()));
                result.response.sample(time, dt, 1 - error / scale, turret.output(),
                        Math.abs(error) <= RobotConfig.SHOT_AIM_TOLERANCE
                        && Math.abs(turret.speed) <= Math.toRadians(15));
                op.show(label, "误差 %+.2f°，速度 %+.0f°/s，功率 %+.2f / 峰值 %.2f",
                        Math.toDegrees(error), Math.toDegrees(turret.speed), turret.output(), result.peakPower);
                if (baseline && result.quiet >= 0.40 && result.stable()) return false;
                return !pidSearch || !result.response.finishIfSettled(0.36);
            });
        } finally { turret.stop(); }
        return result;
    }

    static final class Result {
        final StepScore response;
        double overshoot, quiet, endError, endSpeed, peakPower;
        int lastDirection, reversals;
        boolean unsafe, limitReached, readFailed;

        Result(double duration) { response = new StepScore(duration); }

        boolean stable() {
            return !unsafe && !readFailed && quiet >= HOLD && overshoot <= OVERSHOOT_LIMIT && reversals <= 1;
        }
    }
}

