package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import org.firstinspires.ftc.teamcode.biobuzz.CalibrationStore;
import org.firstinspires.ftc.teamcode.biobuzz.Maths;
import org.firstinspires.ftc.teamcode.biobuzz.Turret;
import org.firstinspires.ftc.teamcode.biobuzz.TurretPlant;

final class TurretQuickIdentification {
    private final CalibrationOp op;
    private final Turret turret;

    TurretQuickIdentification(CalibrationOp op, Turret turret) {
        this.op = op;
        this.turret = turret;
    }

    boolean run() throws InterruptedException {
        double[] breakaway = {findBreakaway(1), findBreakaway(-1)};
        for (int side = 0; side < 2 && active(); side++) {
            if (breakaway[side] != 0) continue;
            int direction = side == 0 ? 1 : -1;
            double nearby = turret.span() / 2 - direction * Math.min(0.10 * turret.span(), Math.toRadians(12));
            op.show("起转换位重试", "移到 %.1f° 后重测%s起转",
                    Math.toDegrees(nearby), direction > 0 ? "左转" : "右转");
            if (TurretMotion.go(op, turret, nearby, Math.toRadians(1.7))) {
                breakaway[side] = findBreakaway(direction);
            }
        }
        if (!active() || breakaway[0] == 0 || breakaway[1] == 0) {
            op.telemetry.log().add("快速辨识未检测到双向起转，检查编码器、线束或静摩擦。");
            return false;
        }
        TurretPlantFit[] fit = {new TurretPlantFit(), new TurretPlantFit()};
        for (double extra : new double[]{0.055, 0.115, 0.18, 0.25}) {
            if (!active()) return false;
            for (int side = 0; side < 2; side++) {
                balance(breakaway);
                pulse(side == 0 ? 1 : -1, Math.min(0.70, breakaway[side] + extra), fit[side]);
            }
        }
        double[] model = new double[11];
        for (int side = 0; side < 2 && active(); side++) {
            double[] fitted = fit[side].fit(breakaway[side]);
            if (fitted == null) {
                for (double extra : new double[]{0.08, 0.22}) {
                    // Opposite-direction pulse recovers travel without using a feedback controller.
                    balance(breakaway);
                    pulse(side == 0 ? -1 : 1, Math.min(0.67, breakaway[1 - side] + extra), fit[1 - side]);
                    pulse(side == 0 ? 1 : -1, Math.min(0.74, breakaway[side] + extra), fit[side]);
                }
                fitted = fit[side].fit(breakaway[side]);
            }
            if (fitted == null) {
                op.telemetry.log().add("%s短脉冲辨识不一致（%d 个采样），暂停全行程测量。",
                        side == 0 ? "左转" : "右转", fit[side].count());
                return false;
            }
            System.arraycopy(fitted, 0, model, 5 * side, 5);
            op.telemetry.log().add("%s kV=%.3f，响应 %.0f ms，延迟 %.0f ms",
                    side == 0 ? "左转" : "右转", fitted[2], 1000 * fitted[3], 1000 * fitted[4]);
        }
        if (!active()) return false;
        model[10] = 0.85;
        turret.installPlant(new TurretPlant(model));
        double step = Math.min(Math.toRadians(9), 0.09 * turret.span());
        double center = Maths.clip(turret.angle, 0.34 * turret.span(), 0.66 * turret.span());
        for (double scale : new double[]{0.85, 0.69, 0.55}) {
            if (!active()) return false;
            turret.tune(scale);
            if (TurretMotion.go(op, turret, center, Math.toRadians(1.1))
                    && TurretStability.step(op, turret, center + step, "快速基线验证").stable()
                    && TurretStability.step(op, turret, center - step, "快速基线验证").stable()) {
                model[10] = scale;
                op.telemetry.log().add("开环辨识和双向稳定性验证通过，带宽比例 %.2f。", scale);
                return active() && op.save(CalibrationStore.TURRET_PLANT, model);
            }
        }
        op.telemetry.log().add("快速基线停稳验证未通过，保留原数据。");
        return false;
    }

    private double findBreakaway(int direction) throws InterruptedException {
        if (!active() || !TurretMotion.quiet(op, turret, 1.35)) return 0;
        double origin = turret.angle;
        double[] ks = {0};
        try {
            op.runFor(1.60, (t, dt) -> {
                turret.sample(dt);
                if (!valid()) return false;
                if (direction * (turret.angle - origin) >= Math.toRadians(0.55)
                        && direction * turret.speed > 0.10) {
                    ks[0] = Math.abs(turret.output());
                    return false;
                }
                double duty = Math.min(0.65, 0.027 + 0.47 * t);
                turret.power(direction * duty);
                op.show("快速辨识 开环起转", "%s 功率 %.2f", direction > 0 ? "左" : "右", duty);
                return true;
            });
        } finally { turret.stop(); }
        return ks[0];
    }

    private void pulse(int direction, double duty, TurretPlantFit fit) throws InterruptedException {
        if (!active() || !TurretMotion.quiet(op, turret, 1.35)) return;
        double origin = turret.angle;
        double[] firstPower = {Double.NaN};
        double distance = Math.min(Math.toRadians(27), turret.span() * 0.16);
        try {
            op.runFor(0.64, (t, dt) -> {
                turret.sample(dt);
                if (!valid() || Math.abs(turret.angle - origin) > distance) return false;
                if (Double.isFinite(firstPower[0]) && Math.abs(turret.output() - direction * duty) < 0.012) {
                    fit.add(t - firstPower[0], duty, direction * turret.speed);
                }
                if (t > 0.44 && Math.abs(turret.speed) > 0.85) return false;
                if (!Double.isFinite(firstPower[0])) firstPower[0] = t;
                turret.power(direction * duty);
                op.show("快速辨识 开环速度脉冲", "%+.2f 功率，%+.0f°/s",
                        direction * duty, Math.toDegrees(turret.speed));
                return true;
            });
        } finally { turret.stop(); }
    }

    private void balance(double[] breakaway) throws InterruptedException {
        double span = turret.span(), center = span * 0.5;
        if (Math.abs(turret.angle - center) < span * 0.15 || !TurretMotion.quiet(op, turret, 1.35)) return;
        int direction = turret.angle < center ? 1 : -1;
        double duty = Math.min(0.62, breakaway[direction > 0 ? 0 : 1] + 0.10);
        try {
            op.runFor(1.75, (t, dt) -> {
                turret.sample(dt);
                if (!turret.encoderReady() || !turret.motorResponding()) return false;
                if (direction * (center - turret.angle) < span * 0.06) return false;
                turret.power(direction * duty);
                op.show("快速辨识 开环回中", "当前角度 %.1f°", Math.toDegrees(turret.angle));
                return true;
            });
        } finally { turret.stop(); }
    }

    private boolean valid() {
        return turret.encoderReady() && turret.motorResponding()
                && Math.abs(turret.angle - turret.span() * 0.5) < turret.span() * 0.33;
    }

    private boolean active() { return op.opModeIsActive() && !op.motionAborted(); }
}
