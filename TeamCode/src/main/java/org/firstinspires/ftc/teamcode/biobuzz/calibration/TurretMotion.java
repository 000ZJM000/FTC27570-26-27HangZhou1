package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import org.firstinspires.ftc.teamcode.biobuzz.Maths;
import org.firstinspires.ftc.teamcode.biobuzz.Turret;

final class TurretMotion {
    private TurretMotion() {}

    static boolean go(CalibrationOp op, Turret turret, double angle, double tolerance)
            throws InterruptedException {
        for (double maxSpeed : new double[]{1.28, 0.66}) {
            if (!quiet(op, turret, 1.35)) return false;
            if (Math.abs(turret.angle - angle) <= tolerance) return true;
            double[] reference = {turret.angle, 0};
            double duration = 1.6 + Math.abs(angle - turret.angle) / maxSpeed;
            try {
                op.runFor(duration, (time, dt) -> {
                    turret.sample(dt);
                    if (!turret.encoderReady() || !turret.motorResponding()) return false;
                    if (Math.abs(turret.angle - angle) <= tolerance && Math.abs(turret.speed) < 0.14) return false;
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
                    turret.control(reference[0], reference[1], dt);
                    return true;
                });
            } finally {
                turret.stop();
            }
            if (quiet(op, turret, 1.35) && Math.abs(turret.angle - angle) <= tolerance) return true;
        }
        return false;
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
}
