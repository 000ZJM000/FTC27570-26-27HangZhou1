package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.biobuzz.config.CalibrationStore;
import org.firstinspires.ftc.teamcode.biobuzz.config.RobotConfig;
import org.firstinspires.ftc.teamcode.biobuzz.core.Maths;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Turret;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.TurretModels.Friction;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.TurretModels.Plant;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

/** Open-loop 12 V turret plant identification and multi-sector friction table calibration. */
final class TurretIdentify {
    private final CalibrationOp op;
    private final Turret turret;

    private TurretIdentify(CalibrationOp op, Turret turret) {
        this.op = op;
        this.turret = turret;
    }

    static boolean runQuickIdentification(CalibrationOp op, Turret turret) throws InterruptedException {
        return new TurretIdentify(op, turret).run();
    }

    private boolean run() throws InterruptedException {
        double[] breakaway = {findBreakaway(1), findBreakaway(-1)};
        for (int side = 0; side < 2 && active(); side++) {
            if (breakaway[side] != 0) continue;
            int direction = side == 0 ? 1 : -1;
            double nearby = turret.span() / 2 - direction * Math.min(0.10 * turret.span(), Math.toRadians(12));
            op.show("起转换位重试", "移到 %.1f° 后重测%s起转",
                    Math.toDegrees(nearby), direction > 0 ? "左转" : "右转");
            if (TurretBench.go(op, turret, nearby, Math.toRadians(1.7))) {
                breakaway[side] = findBreakaway(direction);
            }
        }
        if (!active() || breakaway[0] == 0 || breakaway[1] == 0) {
            op.telemetry.log().add("快速辨识未检测到双向起转，检查编码器、线束或静摩擦。");
            return false;
        }
        PlantFit[] fit = {new PlantFit(), new PlantFit()};
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
        turret.installPlant(new Plant(model));
        double step = Math.min(Math.toRadians(9), 0.09 * turret.span());
        double center = Maths.clip(turret.angle, 0.34 * turret.span(), 0.66 * turret.span());
        for (double scale : new double[]{0.85, 0.69, 0.55}) {
            if (!active()) return false;
            turret.tune(scale);
            if (TurretBench.go(op, turret, center, Math.toRadians(1.1))
                    && TurretBench.step(op, turret, center + step, "快速基线验证").stable()
                    && TurretBench.step(op, turret, center - step, "快速基线验证").stable()) {
                model[10] = scale;
                op.telemetry.log().add("开环辨识和双向稳定性验证通过，带宽比例 %.2f。", scale);
                return active() && op.save(CalibrationStore.TURRET_PLANT, model);
            }
        }
        op.telemetry.log().add("快速基线停稳验证未通过，保留原数据。");
        return false;
    }

    private double findBreakaway(int direction) throws InterruptedException {
        if (!active() || !TurretBench.quiet(op, turret, 1.35)) return 0;
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

    private void pulse(int direction, double duty, PlantFit fit) throws InterruptedException {
        if (!active() || !TurretBench.quiet(op, turret, 1.35)) return;
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
        if (Math.abs(turret.angle - center) < span * 0.15 || !TurretBench.quiet(op, turret, 1.35)) return;
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

    /** Fits delayed first-order response jointly across several constant-duty pulses. */
    private static final class PlantFit {
        private final List<double[]> points = new ArrayList<>();

        void add(double t, double duty, double speed) {
            if (Maths.finite(t, duty, speed)) points.add(new double[]{t, duty, Math.max(0, speed)});
        }

        int count() { return points.size(); }

        double[] fit(double breakaway) {
            if (points.size() < 22) return null;
            double min = 1, max = 0, peak = 0;
            for (double[] p : points) {
                min = Math.min(min, p[1]);
                max = Math.max(max, p[1]);
                peak = Math.max(peak, p[2]);
            }
            if (max - min < 0.065 || peak < 0.30) return null;
            double best = Double.POSITIVE_INFINITY;
            double[] model = null;
            double[] shape = new double[points.size()];
            double cut = Math.max(0.10, 0.12 * peak);
            for (int i = 0; i < 34; i++) {
                double tau = 0.025 * Math.pow(0.95 / 0.025, i / 33.0);
                for (int j = 0; j <= 12; j++) {
                    double delay = j * 0.0125;
                    double gain = 1 / 0.12, drag = breakaway * 0.75;
                    for (int n = 0; n < points.size(); n++) {
                        shape[n] = -Math.expm1(-Math.max(0, points.get(n)[0] - delay) / tau);
                    }
                    for (int it = 0; it < 4; it++) {
                        double xx = 0, xz = 0, zz = 0, xy = 0, zy = 0;
                        for (int n = 0; n < points.size(); n++) {
                            double[] p = points.get(n);
                            double z = shape[n], x = z * p[1];
                            double residual = Math.abs(p[2] - gain * (x - z * drag));
                            double weight = it == 0 ? 1 : Math.min(1, cut / Math.max(1e-7, residual));
                            xx += weight * x * x;
                            xz += weight * x * z;
                            zz += weight * z * z;
                            xy += weight * x * p[2];
                            zy += weight * z * p[2];
                        }
                        double det = xx * zz - xz * xz;
                        if (det < 1e-7) break;
                        gain = (xy * zz - zy * xz) / det;
                        if (gain < 1e-5) break;
                        drag = Maths.clip((gain * xz - zy) / (gain * zz), 0.005, breakaway + 0.045);
                        gain = Maths.clip((xy - drag * zy) / (xx - 2 * drag * xz + drag * drag * zz), 1.5, 65);
                    }
                    double error = 0;
                    for (int n = 0; n < points.size(); n++) {
                        double[] p = points.get(n);
                        double r = Math.abs(p[2] - gain * Math.max(0, p[1] - drag) * shape[n]);
                        error += r < cut ? 0.5 * r * r : cut * (r - 0.5 * cut);
                    }
                    error /= points.size();
                    if (error < best) {
                        best = error;
                        model = new double[]{breakaway, drag, 1 / gain, tau, delay};
                    }
                }
            }
            return best < 0.085 * peak * peak ? model : null;
        }
    }

    /** Multi-sector static and kinetic friction sweep. */
    static final class FrictionRun {
        private static final int[] ANCHORS = {4, 7, 2, 9, 0, 11};
        private static final double[] RETRY_OFFSETS = {0, -0.22, 0.22};
        private static final double EXTRA_MARGIN = Math.toRadians(0.8);
        private final CalibrationOp op;
        private boolean feedbackLost;
        private double feedbackBadSince = Double.NaN;

        FrictionRun(CalibrationOp op) { this.op = op; }
        boolean feedbackLost() { return feedbackLost; }

        String run(Turret turret) throws InterruptedException {
            int sectors = Friction.SECTORS;
            double[][] start = emptySamples(sectors), moving = emptySamples(sectors);
            double width = Friction.width(turret.span());
            Plant model = Plant.load(op.store);
            for (double offset : RETRY_OFFSETS) {
                for (int sector : ANCHORS) {
                    for (int direction : new int[]{1, -1}) {
                        if (!active()) return feedbackLost ? "反馈失效，本轮未写入" : "已中止";
                        int side = Friction.side(direction);
                        double center = Friction.center(sector, turret.span()) + offset * width;
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
            boolean arrived = TurretBench.go(op, turret, target, Math.min(Math.toRadians(1.5), 0.20 * width));
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

        private String save(double[][] start, double[][] moving, Plant plant) {
            double[] model = new Friction(op.store).values();
            int[] counts = {count(start[0]), count(start[1]), count(moving[0]), count(moving[1])};
            String coverage = String.format(Locale.ROOT, "静 L/R %d/%d，动 L/R %d/%d",
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
                int side = Friction.side(direction);
                model[side] = plant.kV(direction);
                for (int sector = 0; sector < Friction.SECTORS; sector++) {
                    int index = Friction.index(sector, direction);
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

        private static double interpolate(double[] values, int sector) {
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
}
