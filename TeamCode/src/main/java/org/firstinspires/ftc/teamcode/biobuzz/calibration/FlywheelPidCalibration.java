package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import org.firstinspires.ftc.teamcode.biobuzz.CalibrationStore;
import org.firstinspires.ftc.teamcode.biobuzz.Flywheel;
import org.firstinspires.ftc.teamcode.biobuzz.Maths;
import org.firstinspires.ftc.teamcode.biobuzz.RobotConfig;

final class FlywheelPidCalibration {
    private static final double HOLD = 0.24;
    private static final double MAX_OVERSHOOT = 0.65;
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
        low = 0.52 * wheel.maxTargetRpmAtBattery();
        high = Math.min(0.96 * wheel.maxTargetRpmAtBattery(), RobotConfig.FLYWHEEL_TARGET_RPM + 250);
        if (high - low < 350) {
            outcome = "有效转速范围不足";
            return;
        }
        step = Maths.clip(0.034 * high, 65, 120);
        double[] old = wheel.pidGains();
        search = new BayesianPidSearch(new double[]{0.20, 0, 0}, new double[]{4.5, 4, 0.15},
                new double[]{1.0, 0.55, 0.03});
        try {
            search.tune(old, this::trial, () -> !active(),
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
        if (current != null && current.qualified()) save(current, "现有 PID 复测通过，已保存为 12V PID");
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
        boolean settled = true;
        String reason = "";
        for (double center : centers) {
            if (!prepare(center) || !active()) return BayesianPidSearch.Result.unavailable("起始转速未稳定");
            for (double target : new double[]{center + direction * step, center}) {
                Response response = measure(target, verification);
                if (!active() || response.samples == 0) return BayesianPidSearch.Result.unavailable("转速采样中断");
                boolean reached = Double.isFinite(response.score.settlingTime(HOLD));
                sum += reached ? response.score.scoreWithSettlingTime(HOLD) : 3 + response.score.score();
                margin = Math.min(margin, MAX_OVERSHOOT - response.score.overshoot());
                endError = Math.max(endError, response.endError);
                settled &= reached;
                if (reason.isEmpty() && !reached) reason = String.format(java.util.Locale.ROOT,
                        "%.0f RPM 未停稳，末端误差 %.0f RPM", target, response.endError);
            }
            direction = -direction;
        }
        return new BayesianPidSearch.Result(sum / (2 * centers.length), margin, settled, reason, endError);
    }

    private double center(double fraction) {
        return low + step + fraction * (high - low - 2 * step);
    }

    private boolean prepare(double target) throws InterruptedException {
        double[] quiet = {0};
        op.runFor(4.5, (time, dt) -> {
            wheel.sample();
            if (!active()) return false;
            wheel.requestRpm(target);
            quiet[0] = Math.abs(wheel.rpm() - target) <= Math.max(40, 0.018 * target) ? quiet[0] + dt : 0;
            op.show("飞轮 PID 阶跃起点", "%.0f / %.0f RPM，输出 %.2f V", wheel.rpm(), target, wheel.deliveredVolts());
            return quiet[0] < HOLD;
        });
        return quiet[0] >= HOLD;
    }

    private Response measure(double target, boolean verification) throws InterruptedException {
        double initial = wheel.rpm();
        double delta = Math.copySign(Math.max(25, Math.abs(target - initial)), target - initial);
        double duration = verification ? 2.1 : 1.8;
        Response response = new Response(duration);
        op.runFor(duration, (time, dt) -> {
            wheel.sample();
            if (!active()) return false;
            wheel.requestRpm(target);
            response.samples++;
            response.endError = Math.abs(wheel.rpm() - target);
            double progress = 1 - (target - wheel.rpm()) / delta;
            response.score.sample(time, dt, progress, wheel.deliveredVolts() / 12,
                    response.endError <= Math.max(25, 0.014 * target));
            op.show(verification ? "飞轮 PID 留出复测" : "飞轮 PID 拟合", "%.0f / %.0f RPM，前馈 %.2f V / 输出 %.2f V",
                    wheel.rpm(), target, wheel.feedforwardVolts(), wheel.deliveredVolts());
            return verification || !response.score.finishIfSettled(0.34);
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

    private static final class Response {
        final StepScore score;
        double endError;
        int samples;

        Response(double duration) { score = new StepScore(duration); }
    }
}
