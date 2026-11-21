package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.biobuzz.CalibrationStore;
import org.firstinspires.ftc.teamcode.biobuzz.Flywheel;
import org.firstinspires.ftc.teamcode.biobuzz.Maths;
import org.firstinspires.ftc.teamcode.biobuzz.RobotConfig;

final class FlywheelPidCalibration {
    private static final double HOLD = 0.55;
    private static final double PREPARE_SECONDS = 12;
    private static final double MAX_OVERSHOOT = 0.65;
    private static final double[] WARMUP_PID = {0.9, 0.65, 0.010};
    private final CalibrationOp op;
    private final Flywheel wheel;
    private BayesianPidSearch search;
    private String outcome = "未验证";
    private double low, high, step;

    FlywheelPidCalibration(CalibrationOp op, Flywheel wheel) {
        this.op = op;
        this.wheel = wheel;
    }

    void run() throws InterruptedException {
        double[] old = wheel.pidGains();
        boolean alternate = false;
        try {
            double atBattery = wheel.maxTargetRpmAtBattery();
            double start = Math.min(RobotConfig.FLYWHEEL_TARGET_RPM, 0.84 * atBattery);
            boolean ready = prepare(start, "飞轮 PID 预热");
            if (!ready && active()) {
                alternate = true;
                start = Math.min(0.85 * start, 0.78 * wheel.maxTargetRpmAtBattery());
                wheel.setPidGains(WARMUP_PID);
                ready = prepare(start, "飞轮 PID 降档重试");
            }
            if (!ready || !active()) {
                outcome = wheel.faulted() ? "飞轮反馈中断" : !active() ? "已中止"
                        : String.format(java.util.Locale.ROOT,
                        "两个起始档均未稳速，最后 %.0f / %.0f RPM，沿用原 PID", wheel.rpm(), start);
                return;
            }
            double loadedLimit = wheel.maxTargetRpmAtBattery();
            high = Math.min(Math.min(0.96 * loadedLimit, 1.16 * start), RobotConfig.FLYWHEEL_TARGET_RPM + 250);
            low = Math.min(0.54 * loadedLimit, 0.76 * start);
            if (high - low < 180) {
                outcome = "稳速后可用转速跨度不足";
                return;
            }
            step = Math.min(0.20 * (high - low), Maths.clip(0.030 * high, 55, 105));
            RobotLog.ii("BioBuzzCalibration", "Flywheel PID warmup %.0f RPM at %.2f V; search %.0f-%.0f RPM",
                    wheel.rpm(), op.battery.volts(), low, high);

            search = new BayesianPidSearch(new double[]{0.20, 0, 0}, new double[]{4.5, 4, 0.15},
                    new double[]{1.0, 0.55, 0.03}, 3);
            search.tune(alternate ? WARMUP_PID : old, this::trial, () -> !active(),
                    op.store.readHistory(CalibrationStore.FLYWHEEL_VOLTAGE_PID_HISTORY,
                            BayesianPidSearch.HISTORY_STRIDE, BayesianPidSearch.HISTORY_ROWS));
        } finally {
            wheel.stop();
            wheel.setPidGains(old);
        }
        if (!active()) {
            outcome = wheel.faulted() ? "飞轮反馈中断" : "已中止";
            return;
        }
        if (search.searchCount() > 0) op.save(CalibrationStore.FLYWHEEL_VOLTAGE_PID_HISTORY, search.history());
        BayesianPidSearch.Review best = search.bestCandidate(), current = search.incumbent();
        if (best != null && (current == null || !current.qualified() || best.score < current.score)) {
            if (op.choosePid(search, best, () -> op.telemetry.addData("留出复测", "使用比赛的 12V 前馈＋PID＋Bang-Bang"))) {
                save(best, "已保存新的 12V PID");
                return;
            }
            outcome = "保留进入自测时的 PID";
            return;
        }
        if (current != null && current.qualified()) {
            save(current, alternate ? "降档稳速基线通过复测，已保存" : "现有 PID 复测通过，已保存为 12V PID");
        }
        else outcome = "本轮未保存 PID，沿用进入自测时的参数";
    }

    private void save(BayesianPidSearch.Review result, String message) {
        if (active() && op.save(CalibrationStore.FLYWHEEL_VOLTAGE_PID, result.gains)) {
            wheel.setPidGains(result.gains);
            outcome = message;
        } else outcome = "PID 保存失败";
    }

    private BayesianPidSearch.Result trial(double[] gains, boolean verification, int pattern)
            throws InterruptedException {
        wheel.setPidGains(gains);
        double direction = (pattern % 2 == 0) ? 1 : -1;
        double[] centers = verification
                ? new double[]{center(pattern == 0 ? 0.18 : 0.84), center(pattern == 0 ? 0.84 : 0.18)}
                : new double[]{center(0.16 + 0.34 * (pattern % 3))};
        double sum = 0, margin = MAX_OVERSHOOT, endError = 0;
        for (double center : centers) {
            if (!prepare(center, "飞轮 PID 阶跃就位") || !active()) {
                if (!active()) return BayesianPidSearch.Result.unavailable("起始转速采样中断");
                String why = String.format(java.util.Locale.ROOT, "起始 %.0f RPM 等待 %.0f 秒仍未稳速", center, PREPARE_SECONDS);
                return new BayesianPidSearch.Result(8, -0.20, false, why);
            }
            for (double target : new double[]{center + direction * step, center}) {
                Response response = measure(target, verification);
                if (!active() || response.samples == 0) return BayesianPidSearch.Result.unavailable("转速采样中断");
                boolean reached = Double.isFinite(response.score.settlingTime(HOLD));
                sum += reached ? response.score.scoreWithSettlingTime(HOLD) : 3 + response.score.score();
                margin = Math.min(margin, MAX_OVERSHOOT - response.score.overshoot());
                endError = Math.max(endError, response.endError);
                if (!reached) {
                    // No return step from a still-moving initial condition.
                    String reason = String.format(java.util.Locale.ROOT,
                            "%.0f RPM 未停稳，末端误差 %.0f RPM", target, response.endError);
                    return new BayesianPidSearch.Result(Math.min(9, sum), margin, false, reason, endError);
                }
            }
            direction = -direction;
        }
        return new BayesianPidSearch.Result(sum / (2 * centers.length), margin, true, "", endError);
    }

    private double center(double fraction) {
        return low + step + fraction * (high - low - 2 * step);
    }

    private boolean prepare(double target, String phase) throws InterruptedException {
        StableSpeed steady = new StableSpeed();
        op.runFor(PREPARE_SECONDS, (time, dt) -> {
            wheel.sample();
            if (!active()) return false;
            wheel.requestRpm(target);
            steady.update(wheel.rpm(), target, dt);
            op.show(phase, "%.0f / %.0f RPM，输出 %.2f V，已等待 %.1f s，稳速 %.2f / %.2f s",
                    wheel.rpm(), target, wheel.deliveredVolts(), time, steady.time, HOLD);
            return !steady.ready();
        });
        if (!steady.ready() && active()) {
            RobotLog.ww("BioBuzzCalibration", "%s timed out: %.0f/%.0f RPM, %.2f V output, battery %.2f V, stable %.2f s",
                    phase, wheel.rpm(), target, wheel.deliveredVolts(), op.battery.volts(), steady.time);
        }
        return active() && steady.ready();
    }

    private Response measure(double target, boolean verification) throws InterruptedException {
        double initial = wheel.rpm();
        double delta = Math.copySign(Math.max(25, Math.abs(target - initial)), target - initial);
        double duration = verification ? 6.5 : 4.5;
        Response response = new Response(duration);
        StableSpeed steady = new StableSpeed();
        op.runFor(duration, (time, dt) -> {
            wheel.sample();
            if (!active()) return false;
            wheel.requestRpm(target);
            response.samples++;
            response.endError = Math.abs(wheel.rpm() - target);
            double progress = 1 - (target - wheel.rpm()) / delta;
            boolean within = steady.update(wheel.rpm(), target, dt);
            response.score.sample(time, dt, progress, wheel.deliveredVolts() / 12, within);
            op.show(verification ? "飞轮 PID 留出复测" : "飞轮 PID 拟合",
                    "%.0f / %.0f RPM，前馈 %.2f V / 输出 %.2f V，稳速 %.2f / %.2f s",
                    wheel.rpm(), target, wheel.feedforwardVolts(), wheel.deliveredVolts(), steady.time, HOLD);
            return !steady.ready() || !response.score.finishIfSettled(HOLD);
        });
        return response;
    }

    void showResults() {
        op.telemetry.addData("飞轮 PID", outcome);
        if (search == null) return;
        op.telemetry.addData("PID 搜索", "%d 轮 / 历史 %d 轮", search.searchCount(), search.historyCount());
        op.showPidReview("进入校准时", search.incumbent());
        op.showPidReview("最佳新候选", search.bestCandidate());
    }

    private boolean active() { return op.opModeIsActive() && !op.motionAborted() && !wheel.faulted(); }

    static final class StableSpeed {
        double time;
        private double minimum = Double.POSITIVE_INFINITY, maximum = Double.NEGATIVE_INFINITY;

        boolean update(double rpm, double target, double dt) {
            if (Math.abs(rpm - target) > Math.max(45, 0.018 * target)) {
                reset();
                return false;
            }
            if (Math.max(maximum, rpm) - Math.min(minimum, rpm) > Math.max(28, 0.012 * target)) {
                reset();
                minimum = maximum = rpm;
                time = dt;
                return false;
            }
            minimum = Math.min(minimum, rpm);
            maximum = Math.max(maximum, rpm);
            time += dt;
            return true;
        }

        private void reset() {
            time = 0;
            minimum = Double.POSITIVE_INFINITY;
            maximum = Double.NEGATIVE_INFINITY;
        }

        boolean ready() { return time >= HOLD; }
    }

    private static final class Response {
        final StepScore score;
        double endError;
        int samples;

        Response(double duration) { score = new StepScore(duration); }
    }
}
