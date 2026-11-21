package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import org.firstinspires.ftc.teamcode.biobuzz.Maths;
import org.firstinspires.ftc.teamcode.biobuzz.RobotConfig;
import org.firstinspires.ftc.teamcode.biobuzz.Turret;

final class TurretMotion {
    private TurretMotion() {}

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
            com.qualcomm.robotcore.util.RobotLog.ii("BioBuzzCalibration",
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
}
