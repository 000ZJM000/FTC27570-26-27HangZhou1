package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.biobuzz.CalibrationStore;
import org.firstinspires.ftc.teamcode.biobuzz.Maths;
import org.firstinspires.ftc.teamcode.biobuzz.RobotConfig;
import org.firstinspires.ftc.teamcode.biobuzz.Turret;
import org.firstinspires.ftc.teamcode.biobuzz.TurretFriction;
import org.firstinspires.ftc.teamcode.biobuzz.TurretPlant;

import java.util.Arrays;

final class TurretFrictionCalibration {
    private static final int[] ANCHORS = {4, 7, 2, 9, 0, 11};
    private static final double[] RETRY_OFFSETS = {0, -0.22, 0.22};
    private static final double EXTRA_MARGIN = Math.toRadians(0.8);
    private final CalibrationOp op;
    private boolean feedbackLost;
    private double feedbackBadSince = Double.NaN;

    TurretFrictionCalibration(CalibrationOp op) { this.op = op; }
    boolean feedbackLost() { return feedbackLost; }

    String run(Turret turret) throws InterruptedException {
        int sectors = TurretFriction.SECTORS;
        double[][] start = emptySamples(sectors), moving = emptySamples(sectors);
        double width = TurretFriction.width(turret.span());
        TurretPlant model = TurretPlant.load(op.store);
        for (double offset : RETRY_OFFSETS) {
            for (int sector : ANCHORS) {
                for (int direction : new int[]{1, -1}) {
                    if (!active()) return feedbackLost ? "反馈失效，本轮未写入" : "已中止";
                    int side = TurretFriction.side(direction);
                    double center = TurretFriction.center(sector, turret.span()) + offset * width;
                    if (!Double.isFinite(start[side][sector])) {
                        start[side][sector] = measureBreakaway(turret, center, width, direction);
                    }
                    if (active() && !Double.isFinite(moving[side][sector])) {
                        moving[side][sector] = measureMoving(turret, center, width, direction,
                                start[side][sector], model.kV(direction));
                    }
                    if (active() && (!Double.isFinite(start[side][sector]) || !Double.isFinite(moving[side][sector]))) {
                        RobotLog.ii("BioBuzzCalibration", "Friction sector %d dir %+d offset %+.2f: static=%.2f V kinetic=%.2f V",
                                sector + 1, direction, offset, 12 * start[side][sector], 12 * moving[side][sector]);
                    }
                }
            }
        }
        if (!active()) return feedbackLost ? "反馈失效，本轮未写入" : "已中止";
        return save(start, moving, model);
    }

    private double measureBreakaway(Turret turret, double center, double width, int direction)
            throws InterruptedException {
        double target = inside(turret, center);
        if (!position(turret, target, width)) return Double.NaN;
        double origin = turret.angle;
        double[] voltage = {Double.NaN, Double.NaN};
        try {
            op.runFor(1.65, (t, dt) -> {
                turret.sample(dt);
                if (!healthy(turret) || !safe(turret, direction)) return false;
                double travel = direction * (turret.angle - origin);
                double speed = direction * turret.speed;
                if (!Double.isFinite(voltage[0]) && travel >= Math.toRadians(0.14) && speed > 0.065) {
                    voltage[0] = direction * turret.output();
                }
                if (Double.isFinite(voltage[0]) && travel >= Math.toRadians(0.38) && speed > 0.075) {
                    if (voltage[0] > 0.02) voltage[1] = voltage[0];
                    return false;
                }
                double command = Math.min(0.66, 0.025 + 0.39 * t);
                turret.power(direction * command);
                op.show("静摩擦 · 静止起转", "%+.1f° / %+.0f°/s，实际 %.2f V",
                        Math.toDegrees(turret.angle), Math.toDegrees(speed), Math.abs(turret.output()) * 12);
                return healthy(turret);
            });
        } finally {
            turret.stop();
        }
        return active() ? voltage[1] : Double.NaN;
    }

    private double measureMoving(Turret turret, double center, double width, int direction, double breakaway, double kv)
            throws InterruptedException {
        double lead = Maths.clip(0.42 * width, Math.toRadians(4), Math.toRadians(9));
        double originTarget = inside(turret, center - direction * lead);
        double end = inside(turret, center + direction * lead);
        if (direction * (end - originTarget) < Math.toRadians(3.2) || !position(turret, originTarget, width)) {
            return Double.NaN;
        }
        double origin = turret.angle;
        double staticEstimate = Double.isFinite(breakaway) ? breakaway : turret.frictionDuty(origin, direction, 0);
        double reference = direction * (end - origin) < Math.toRadians(6) ? 0.24 : 0.36;
        double feedforward = turret.frictionDuty(origin, direction, 0.65) + kv * reference;
        MovingWindow stable = new MovingWindow();
        double[] trim = {0};
        boolean[] moving = {false};
        try {
            op.runFor(2.7, (t, dt) -> {
                turret.sample(dt);
                if (!healthy(turret) || !safe(turret, direction) || direction * (end - turret.angle) <= 0) return false;
                double speed = direction * turret.speed;
                double travel = direction * (turret.angle - origin);
                moving[0] |= travel >= Math.toRadians(0.30) && speed >= 0.11;
                if (moving[0] && travel >= Math.toRadians(0.70) && Math.abs(speed - reference) < 0.14) {
                    stable.sample(speed, direction * turret.output() - kv * speed, dt);
                    if (stable.ready()) return false;
                } else stable.reset();

                double command;
                if (moving[0]) {
                    double error = reference - speed;
                    trim[0] = Maths.clip(trim[0] + 0.28 * error * dt, -0.18, 0.18);
                    command = Maths.clip(feedforward + 0.32 * error + trim[0], 0, 0.66);
                } else {
                    command = Maths.clip(staticEstimate * 0.82 + 0.04 + 0.28 * t, 0.09, 0.66);
                }
                turret.power(direction * command);
                op.show("动摩擦 · 独立匀速行程", "%+.1f° / %.0f°/s，稳定采样 %.0f ms",
                        Math.toDegrees(turret.angle), Math.toDegrees(speed), stable.time * 1000);
                return healthy(turret);
            });
        } finally {
            turret.stop();
        }
        return active() && stable.ready() ? stable.frictionIntegral / stable.time : Double.NaN;
    }

    private boolean position(Turret turret, double target, double width) throws InterruptedException {
        boolean arrived = TurretMotion.go(op, turret, target, Math.min(Math.toRadians(1.5), 0.20 * width));
        if (!arrived && active()) healthy(turret);
        return arrived;
    }

    private static double inside(Turret turret, double angle) {
        double margin = RobotConfig.TURRET_MARGIN + EXTRA_MARGIN;
        return Maths.clip(angle, margin, turret.span() - margin);
    }

    private static boolean safe(Turret turret, int direction) {
        double remaining = direction > 0 ? turret.span() - turret.angle : turret.angle;
        return remaining > RobotConfig.TURRET_MARGIN + EXTRA_MARGIN;
    }

    private boolean healthy(Turret turret) {
        if (turret.encoderReady() && turret.motorResponding()) {
            feedbackBadSince = Double.NaN;
            return true;
        }
        double now = Maths.time();
        if (!Double.isFinite(feedbackBadSince)) feedbackBadSince = now;
        if (now - feedbackBadSince >= 0.14) {
            feedbackLost = true;
            RobotLog.ww("BioBuzzCalibration", "Turret friction stopped: encoderReady=%s motorResponding=%s",
                    turret.encoderReady(), turret.motorResponding());
        }
        return false;
    }

    private String save(double[][] start, double[][] moving, TurretPlant plant) {
        double[] model = new TurretFriction(op.store).values();
        int[] counts = {count(start[0]), count(start[1]), count(moving[0]), count(moving[1])};
        String coverage = String.format(java.util.Locale.ROOT, "静 L/R %d/%d，动 L/R %d/%d",
                counts[0], counts[1], counts[2], counts[3]);
        boolean complete = true;
        for (int n : counts) {
            if (n < 2) {
                RobotLog.ww("BioBuzzCalibration", "Friction coverage insufficient: %s", coverage);
                return "实测覆盖不足，沿用原表（" + coverage + "）";
            }
            complete &= n == ANCHORS.length;
        }
        for (int direction : new int[]{1, -1}) {
            int side = TurretFriction.side(direction);
            model[side] = plant.kV(direction);
            for (int sector = 0; sector < TurretFriction.SECTORS; sector++) {
                int index = TurretFriction.index(sector, direction);
                double staticDuty = interpolate(start[side], sector);
                double kineticDuty = Math.min(staticDuty, interpolate(moving[side], sector));
                model[index] = staticDuty;
                model[index + 1] = kineticDuty;
            }
        }
        RobotLog.ii("BioBuzzCalibration", "Friction coverage %s", coverage);
        if (!op.save(CalibrationStore.TURRET_FRICTION_MAP, model)) return "写入失败，沿用原表";
        return (complete ? "已保存" : "已保存，缺点独立插值") + "（" + coverage + "）";
    }

    private static double[][] emptySamples(int sectors) {
        double[][] values = new double[2][sectors];
        for (double[] row : values) Arrays.fill(row, Double.NaN);
        return values;
    }

    private static int count(double[] values) {
        int count = 0;
        for (double value : values) if (Double.isFinite(value)) count++;
        return count;
    }

    static double interpolate(double[] values, int sector) {
        if (Double.isFinite(values[sector])) return values[sector];
        int before = sector - 1, after = sector + 1;
        while (before >= 0 && !Double.isFinite(values[before])) before--;
        while (after < values.length && !Double.isFinite(values[after])) after++;
        if (before < 0) return values[after];
        if (after == values.length) return values[before];
        double t = (double) (sector - before) / (after - before);
        return values[before] + t * t * (3 - 2 * t) * (values[after] - values[before]);
    }

    private boolean active() { return op.opModeIsActive() && !op.motionAborted() && !feedbackLost; }

    private static final class MovingWindow {
        double time, frictionIntegral;
        double low = Double.POSITIVE_INFINITY, high = Double.NEGATIVE_INFINITY;

        void sample(double speed, double residual, double dt) {
            if (!Double.isFinite(residual) || residual < -0.018) {
                reset();
                return;
            }
            if (Math.max(high, speed) - Math.min(low, speed) > Math.max(0.10, 0.24 * speed)) reset();
            low = Math.min(low, speed);
            high = Math.max(high, speed);
            time += dt;
            frictionIntegral += Maths.clip(residual, 0, 0.66) * dt;
        }

        void reset() {
            time = frictionIntegral = 0;
            low = Double.POSITIVE_INFINITY;
            high = Double.NEGATIVE_INFINITY;
        }

        boolean ready() { return time >= 0.12; }
    }
}
