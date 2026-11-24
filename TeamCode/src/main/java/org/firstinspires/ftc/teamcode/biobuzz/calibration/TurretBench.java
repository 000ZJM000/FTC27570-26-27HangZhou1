package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.biobuzz.calibration.CalibrationOp.StepScore;
import org.firstinspires.ftc.teamcode.biobuzz.config.RobotConfig;
import org.firstinspires.ftc.teamcode.biobuzz.core.Maths;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Turret;

/** Shared turret positioning and step-response test bench for calibration OpModes. */
final class TurretBench {
    private static final double HOLD = 0.22;
    private static final double ERROR_LIMIT = Math.toRadians(1.4);
    private static final double RATE_LIMIT = Math.toRadians(10);
    private static final double OVERSHOOT_LIMIT = Math.toRadians(3.5);

    private TurretBench() {}

    static boolean go(CalibrationOp op, Turret turret, double angle, double tolerance)
            throws InterruptedException {
        if (!quiet(op, turret, 1.35)) return false;
        boolean retry = false;
        for (double maxSpeed : new double[]{1.85, 0.66}) {
            if (Math.abs(turret.angle - angle) <= tolerance) return true;
            double measuredBreakaway = retry ? breakawayToward(op, turret, angle, tolerance) : 0;
            if (retry) {
                if (!Double.isFinite(measuredBreakaway)) return false;
                if (!quiet(op, turret, 1.35)) return false;
                if (Math.abs(turret.angle - angle) <= tolerance) return true;
            }
            double[] reference = {turret.angle, 0};
            double[] progress = {Math.abs(angle - turret.angle), 0};
            double duration = 1.6 + Math.abs(angle - turret.angle) / maxSpeed;
            try {
                op.runFor(duration, (time, dt) -> {
                    turret.sample(dt);
                    if (!turret.encoderReady() || !turret.motorResponding()) return false;
                    if (Math.abs(turret.angle - angle) <= tolerance && Math.abs(turret.speed) < 0.14) return false;
                    double distance = Math.abs(angle - turret.angle);
                    if (distance < progress[0] - Math.toRadians(0.40) || Math.abs(turret.speed) > 0.13) {
                        progress[0] = distance;
                        progress[1] = time;
                    } else if (time - progress[1] > 0.70) return false;
                    double remaining = angle - reference[0];
                    double velocity = Math.copySign(Math.min(maxSpeed, Math.sqrt(3.2 * Math.abs(remaining))), remaining);
                    reference[1] += Maths.clip(velocity - reference[1], -2.4 * dt, 2.4 * dt);
                    double step = reference[1] * dt;
                    if (Math.abs(step) >= Math.abs(remaining)) {
                        reference[0] = angle;
                        reference[1] = 0;
                    } else {
                        double lead = reference[0] + step - turret.angle;
                        reference[0] = turret.angle + Maths.clip(lead, -0.25, 0.25);
                    }
                    if (measuredBreakaway == 0) turret.control(reference[0], reference[1], dt);
                    else turret.controlWithMeasuredBreakaway(reference[0], reference[1], measuredBreakaway, dt);
                    return true;
                });
            } finally {
                turret.stop();
            }
            if (!quiet(op, turret, 1.35)) return false;
            if (Math.abs(turret.angle - angle) <= tolerance) return true;
            retry = true;
        }
        return false;
    }

    private static double breakawayToward(CalibrationOp op, Turret turret, double target, double tolerance)
            throws InterruptedException {
        if (!op.opModeIsActive() || op.motionAborted() || !turret.encoderReady() || !turret.motorResponding()) {
            return Double.NaN;
        }
        double origin = turret.angle;
        int direction = target > origin ? 1 : -1;
        double remaining = direction > 0 ? turret.span() - origin : origin;
        if (remaining < RobotConfig.TURRET_MARGIN + Math.toRadians(1)) return Double.NaN;
        double[] progress = {0};
        double[] peak = {0};
        double start = Maths.clip(0.75 * turret.frictionDuty(origin, direction, 0), 0.08, 0.23);
        double limit = Math.min(Math.toRadians(1.0), Math.abs(target - origin) - 0.5 * tolerance);
        try {
            op.runFor(1.45, (time, dt) -> {
                turret.sample(dt);
                if (!turret.encoderReady() || !turret.motorResponding()) return false;
                progress[0] = direction * (turret.angle - origin);
                if (progress[0] >= limit || (progress[0] >= Math.toRadians(0.23) && direction * turret.speed > 0.12)) {
                    return false;
                }
                turret.power(direction * Math.min(0.64, start + 0.40 * time));
                peak[0] = Math.max(peak[0], Math.abs(turret.output()));
                op.show("云台同向起转测试", "角差 %+.1f°，实际 %.2f V，位移 %.2f°",
                        Math.toDegrees(target - turret.angle), 12 * Math.abs(turret.output()), Math.toDegrees(progress[0]));
                return turret.motorResponding();
            });
        } finally {
            turret.stop();
        }
        boolean started = !op.motionAborted() && progress[0] >= Math.toRadians(0.23);
        if (!started && !op.motionAborted()) {
            RobotLog.ii("BioBuzzCalibration",
                    "Turret start not observed at %.2f V, angle %.2f degrees, direction %+d; sample unavailable",
                    12 * peak[0], Math.toDegrees(origin), direction);
        }
        return started ? direction * Math.min(0.66, peak[0] + 0.018) : Double.NaN;
    }

    static boolean quiet(CalibrationOp op, Turret turret, double timeout) throws InterruptedException {
        turret.stop();
        double[] rest = {0, turret.angle};
        op.runFor(timeout, (time, dt) -> {
            turret.sample(dt);
            if (!turret.encoderReady() || !turret.motorResponding() || Math.abs(turret.speed) > 0.11
                    || Math.abs(turret.angle - rest[1]) > Math.toRadians(0.22)) {
                rest[0] = 0;
                rest[1] = turret.angle;
            } else {
                rest[0] += dt;
            }
            return rest[0] < 0.17;
        });
        return op.opModeIsActive() && !op.motionAborted() && rest[0] >= 0.17;
    }

    static boolean goDirect(CalibrationOp op, Turret turret, double angle, String label)
            throws InterruptedException {
        if (!quiet(op, turret, 1.35)) return false;
        double[] settled = {0};
        try {
            op.runFor(6.5, (time, dt) -> {
                turret.sample(dt);
                if (!turret.encoderReady() || !turret.motorResponding()) return false;
                double error = angle - turret.angle;
                settled[0] = Math.abs(error) <= RobotConfig.SHOT_AIM_TOLERANCE
                        && Math.abs(turret.speed) < 0.14 ? settled[0] + dt : 0;
                turret.control(angle, 0, dt);
                op.show(label + "预备角", "误差 %+.1f°，云台 %+.0f°/s，功率 %+.2f",
                        Math.toDegrees(error), Math.toDegrees(turret.speed), turret.output());
                return turret.motorResponding() && settled[0] < 0.12;
            });
            return op.opModeIsActive() && !op.motionAborted() && turret.motorResponding() && settled[0] >= 0.12;
        } finally {
            turret.stop();
        }
    }

    static StepResult step(CalibrationOp op, Turret turret, double target, String label)
            throws InterruptedException {
        return step(op, turret, target, label, 2.35, false, true);
    }

    static StepResult pidStep(CalibrationOp op, Turret turret, double target, String label, boolean longMove,
                              boolean verification) throws InterruptedException {
        return step(op, turret, target, label, longMove ? 2.45 : 1.55, !verification, false);
    }

    private static StepResult step(CalibrationOp op, Turret turret, double target, String label, double duration,
                                   boolean pidSearch, boolean baseline) throws InterruptedException {
        double initial = turret.angle;
        double delta = target - initial;
        double direction = Math.copySign(1, delta);
        StepResult result = new StepResult(duration);
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

    static final class StepResult {
        final StepScore response;
        double overshoot, quiet, endError, endSpeed, peakPower;
        int lastDirection, reversals;
        boolean unsafe, limitReached, readFailed;

        StepResult(double duration) { response = new StepScore(duration); }

        boolean stable() {
            return !unsafe && !readFailed && quiet >= HOLD && overshoot <= OVERSHOOT_LIMIT && reversals <= 1;
        }
    }
}
