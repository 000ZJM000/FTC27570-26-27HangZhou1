package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.biobuzz.calibration.CalibrationOp.StepScore;
import org.firstinspires.ftc.teamcode.biobuzz.config.CalibrationStore;
import org.firstinspires.ftc.teamcode.biobuzz.config.RobotConfig;
import org.firstinspires.ftc.teamcode.biobuzz.core.Maths;
import org.firstinspires.ftc.teamcode.biobuzz.localization.PinpointPose;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Drive;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Turret;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.TurretModels.Plant;

import java.util.Locale;

@TeleOp(name = "05 云台专项 12V PID·回转", group = "BIOBUZZ 自测")
public final class TurretPidCalibration extends CalibrationOp {
    private Tuning tuning;

    @Override String intro() { return "使用已有行程和摩擦。支持 30 秒 PID 快检，或完整搜索及双向回转。"; }

    @Override void calibrate() throws InterruptedException {
        Turret turret = new Turret(hardwareMap, store, battery);
        try {
            if (!turret.calibrated()) {
                telemetry.log().add("尚无云台方向和行程，请先运行 02 云台全套。");
                return;
            }
            turret.teachingMode(true);
            boolean homed = false;
            while (confirm("把云台摆到最右标记，停稳后按 A 记零位。")) {
                turret.homeAtRight();
                if (turret.encoderReady()) {
                    homed = true;
                    break;
                }
            }
            if (!homed || !opModeIsActive()) return;
            turret.teachingMode(false);
            tuning = new Tuning(this);
            if (confirm("轮子落地并清空周围。A 运行 30 秒快检（含底盘双向回转），B 进入完整搜索选项。")) {
                tuning.quickCheck(turret);
            } else if (confirm("清空云台周围。A 开始 10–16 轮搜索及留出复测，B 结束。")) {
                tuning.tune(turret, null, null);
            }
        } finally {
            turret.teachingMode(false);
        }
    }

    @Override void showResults() {
        if (tuning != null) tuning.showResults();
    }

    /** Constrained Bayesian PID tuning, quick check, and bidirectional wrap-around rewind verification. */
    static final class Tuning {
        private static final double HOLD = 0.12;
        private static final double[] VERIFICATION_CENTERS = {0.27, 0.39};
        private final CalibrationOp op;
        private int trials;
        private BayesianPidSearch completed;
        private String outcome = "未验收";
        private String rewindOutcome = "未运行";
        private double peakPower;

        Tuning(CalibrationOp op) {
            this.op = op;
        }

        boolean tune(Turret turret, Drive drive, PinpointPose pose) throws InterruptedException {
            double[] current = turret.pidGains();
            Plant plant = Plant.load(op.store);
            double[] seed = plant.pidSeed(plant.baseline());
            double[] min = {
                    Math.max(0.05, Math.min(0.40 * seed[0], 0.70 * current[0])),
                    Math.min(0.25 * seed[1], 0.60 * current[1]),
                    Math.min(0.20 * seed[2], 0.60 * current[2])
            };
            double[] max = {
                    Math.max(2.10 * seed[0], 1.35 * current[0]),
                    Math.max(2.50 * seed[1] + 0.05, 1.35 * current[1]),
                    Math.max(2.40 * seed[2] + 0.06, 1.35 * current[2] + 0.02)
            };
            double[] seedNorm = {
                    Maths.clip((seed[0] - min[0]) / Math.max(1e-9, max[0] - min[0]), 0, 1),
                    Maths.clip((seed[1] - min[1]) / Math.max(1e-9, max[1] - min[1]), 0, 1),
                    Maths.clip((seed[2] - min[2]) / Math.max(1e-9, max[2] - min[2]), 0, 1)
            };
            BayesianPidSearch search = new BayesianPidSearch(min, max,
                    new double[]{seed[0], Math.max(0.02, seed[1]), Math.max(0.03, seed[2])}, seedNorm);
            boolean pidStale = op.store.isTurretKeyStale(CalibrationStore.TURRET_PID);
            double[] positioningGains = (drive == null && turret.usesSavedPid() && !pidStale) ? current : seed;
            RobotLog.ii("BioBuzzCalibration",
                    "Turret PID search start: current=[%.3f, %.3f, %.3f] seed=[%.3f, %.3f, %.3f] "
                            + "min=[%.3f, %.3f, %.3f] max=[%.3f, %.3f, %.3f] pidStale=%b",
                    current[0], current[1], current[2], seed[0], seed[1], seed[2],
                    min[0], min[1], min[2], max[0], max[1], max[2], pidStale);
            completed = search;
            boolean saved = false;
            try {
                double[] history = pidStale ? new double[0]
                        : op.store.readHistory(CalibrationStore.TURRET_PID_HISTORY, BayesianPidSearch.HISTORY_STRIDE,
                                BayesianPidSearch.HISTORY_ROWS);
                search.tune(current,
                        (gains, verification, pattern) -> trial(turret, positioningGains, seed, gains, verification, pattern),
                        () -> !active(), history);
                if (!active()) {
                    outcome = "云台 PID 测量已停止";
                    return false;
                }
                if (search.searchCount() > 0) op.save(CalibrationStore.TURRET_PID_HISTORY, search.history());

                BayesianPidSearch.Review best = search.bestCandidate();
                BayesianPidSearch.Review original = search.incumbent();
                if (original != null && original.qualified() && (best == null || original.score <= best.score)) best = original;
                boolean full = drive != null;
                boolean fallbackCandidate = false;
                if (best == null) {
                    BayesianPidSearch.Review fallback = !full ? search.bestFallbackCandidate() : null;
                    if (fallback == null) {
                        outcome = "本轮缺少通过留出复测的 PID";
                        RobotLog.ww("BioBuzzCalibration", "Turret PID search failed: no candidate passed verification");
                        return false;
                    }
                    best = fallback;
                    fallbackCandidate = true;
                }
                turret.setPidGains(best.gains);
                if (RewindCheck.supported(turret)) {
                    RewindCheck check = new RewindCheck(op, turret);
                    rewindOutcome = full ? check.run(drive, pose) : check.run(true);
                    RobotLog.ii("BioBuzzCalibration",
                            "Turret rewind check (full=%b skipped=%b passed=%b): %s",
                            full, check.skipped(), check.passed(), rewindOutcome);
                    if ((!check.passed() && !(!full && check.skipped())) || !active()) {
                        outcome = "双向回转未通过，保留原 PID";
                        return false;
                    }
                } else rewindOutcome = "行程不足 270°，按双组阶跃验收";
                turret.stop();
                String choosePrompt = fallbackCandidate
                        ? "留出复测未满 2/2。A 仍保存此最佳新 PID，B 保留旧 PID。"
                        : "请选择 PID。A 保存最佳新值，B 保留旧值。代价越低越好。";
                if (!full && best != original && !op.choosePid(choosePrompt, search, best,
                        () -> op.telemetry.addData("回转", rewindOutcome))) {
                    outcome = "保留旧 PID";
                    return false;
                }
                double steadyToSave = Double.isFinite(best.steadyError) && best.steadyError > 0
                        ? best.steadyError : Math.toRadians(2.5);
                saved = active() && op.store.saveTurretPid(best.gains, steadyToSave);
                double displayScore = best.qualified() ? best.score : best.measuredScore;
                String outcomePrefix = best == original ? "原 PID 通过复测"
                        : best.qualified() ? "已保存新 12V PID"
                        : String.format(Locale.ROOT, "已确认保存新 12V PID（复测 %d/%d）",
                                best.passed, BayesianPidSearch.VERIFICATION_ROUNDS);
                outcome = saved ? String.format(Locale.ROOT, "%s，验收代价 %.3f，最大末端误差 %.2f°",
                        outcomePrefix, displayScore, Math.toDegrees(steadyToSave)) : "PID 未写入";
                op.telemetry.log().add("%s，P/I/D %.3f / %.3f / %.3f", outcome, best.gains[0], best.gains[1], best.gains[2]);
                RobotLog.ii("BioBuzzCalibration", "Turret PID result (saved=%b): %s | P=%.3f I=%.3f D=%.3f",
                        saved, outcome, best.gains[0], best.gains[1], best.gains[2]);
                return saved;
            } finally {
                turret.stop();
                if (!saved) turret.setPidGains(current);
            }
        }

        void showResults() {
            op.telemetry.addData("云台 PID", outcome);
            op.telemetry.addData("回转验收", rewindOutcome);
            if (completed == null) return;
            op.telemetry.addData("PID 搜索", "历史 %d / 本轮 %d，最低 %s",
                    completed.historyCount(), completed.searchCount(), costText(completed.bestSearchCost()));
            op.telemetry.addData("PID 动作峰值", "%.2f V", 12 * peakPower);
            op.showPidReview("进入校准时", completed.incumbent());
            BayesianPidSearch.Review bestNew = completed.bestCandidate();
            op.showPidReview("最佳新候选", bestNew != null ? bestNew : completed.bestFallbackCandidate());
        }

        void quickCheck(Turret turret) throws InterruptedException {
            double[] gains = turret.pidGains();
            op.motionDeadline = Maths.time() + 30;
            try {
                double residual = 0;
                for (int group = 0; group < VERIFICATION_CENTERS.length; group++) {
                    double center = turret.span() * (group == 0 ? 0.30 : 0.70);
                    if (!TurretBench.go(op, turret, center, Math.toRadians(2.2))) {
                        outcome = "快检起始角就位失败";
                        return;
                    }
                    double step = (group == 0 ? 1 : -1) * Math.min(Math.toRadians(35), turret.span() * 0.19);
                    for (double target : new double[]{center + step, center}) {
                        TurretBench.StepResult check = TurretBench.pidStep(op, turret, target, "PID 快检", false, false);
                        if (!active() || check.unsafe || check.readFailed || !Double.isFinite(check.response.settlingTime(HOLD))) {
                            outcome = "快检阶跃未通过";
                            return;
                        }
                        residual = Math.max(residual, Math.abs(check.endError));
                    }
                }
                if (RewindCheck.supported(turret)) {
                    RewindCheck check = new RewindCheck(op, turret);
                    rewindOutcome = check.run(false);
                    if (!check.passed()) {
                        outcome = "快检回转未通过，" + rewindOutcome;
                        return;
                    }
                } else rewindOutcome = "行程不足 270°，仅验收阶跃";
                outcome = active() && op.store.saveTurretPid(gains, residual)
                        ? String.format(Locale.ROOT, "快检通过，末端误差 %.2f°，%s", Math.toDegrees(residual), rewindOutcome)
                        : "快检结束，参数未写入";
            } finally {
                op.motionDeadline = Double.POSITIVE_INFINITY;
            }
        }

        private static String costText(double cost) {
            return Double.isFinite(cost) ? String.format(Locale.US, "%.3f", cost) : "无";
        }

        private BayesianPidSearch.Result trial(Turret turret, double[] positioningGains, double[] seed,
                                               double[] gains, boolean verification, int pattern)
                throws InterruptedException {
            double span = turret.span();
            double sign = verification ? (pattern % 2 == 0 ? 1 : -1) : 1.0;
            double band = verification ? VERIFICATION_CENTERS[pattern % VERIFICATION_CENTERS.length] : 0.36;
            double center = span * (sign > 0 ? band : 1 - band);
            turret.setPidGains(positioningGains);
            if (!TurretBench.go(op, turret, center, Math.toRadians(2.2))) {
                if (positioningGains != seed) {
                    RobotLog.ww("BioBuzzCalibration",
                            "Positioning to %.1f deg failed with current PID; retrying with seed",
                            Math.toDegrees(center));
                    turret.setPidGains(seed);
                }
                if (positioningGains == seed || !TurretBench.go(op, turret, center, Math.toRadians(2.2))) {
                    return BayesianPidSearch.Result.unavailable(String.format(Locale.US, "起始角 %.1f° 就位失败",
                            Math.toDegrees(center)));
                }
            }
            turret.setPidGains(gains);
            trials++;
            double small = Math.min(Math.toRadians(18), span * 0.13);
            double large = Math.min(Math.toRadians(105), span * 0.29);
            double[] targets = {center - sign * small, center, center + sign * large, center};
            double total = 0, worst = 0, margin = 0.34, steadyError = 0, totalSettle = 0;
            int maxReversals = 0;
            boolean settled = true;
            String firstFailure = "";
            for (int i = 0; i < targets.length; i++) {
                double target = targets[i];
                if (!active()) return null;
                double change = target - turret.angle;
                String step = String.format(Locale.US, "动作 %d（%+.1f°）", i + 1, Math.toDegrees(change));
                if (Math.abs(change) < Math.toRadians(3)) {
                    total += 4;
                    worst = Math.max(worst, 4);
                    settled = false;
                    if (firstFailure.isEmpty()) firstFailure = step + " 未形成有效阶跃";
                    continue;
                }
                TurretBench.StepResult run = TurretBench.pidStep(op, turret, target,
                        (verification ? "云台 PID 复测 " : "云台 PID 搜索 ") + trials, i >= 2, verification);
                peakPower = Math.max(peakPower, run.peakPower);
                steadyError = Math.max(steadyError, Math.abs(run.endError));
                if (!active()) return null;
                if (run.readFailed) return BayesianPidSearch.Result.unavailable(step + " 编码器或电机读数中断");
                if (run.unsafe) {
                    String reason = step + (run.limitReached ? " 触发安全行程"
                            : String.format(Locale.US, " 超调 %.1f°（安全上限 %.0f°）",
                                    Math.toDegrees(run.overshoot), Math.toDegrees(run.hardOvershootLimit)));
                    RobotLog.ii("BioBuzzCalibration", "Turret trial %d unsafe (%s): P=%.3f I=%.3f D=%.3f",
                            trials, reason, gains[0], gains[1], gains[2]);
                    return new BayesianPidSearch.Result(9.0, -0.22, false, reason);
                }
                StepScore response = run.response;
                double stepDuration = i >= 2 ? 2.65 : 1.85;
                double settleTime = response.settlingTime(HOLD);
                boolean arrived = Double.isFinite(settleTime);
                double effectiveSettle = arrived ? settleTime : stepDuration;
                totalSettle += effectiveSettle;
                maxReversals = Math.max(maxReversals, run.reversals);
                double stepAbs = Math.abs(change);
                double overDeg = Math.toDegrees(run.overshoot);
                double oscDeg = Math.toDegrees(run.postArrivalTravel);
                double endErrDeg = Math.toDegrees(Math.abs(run.endError));
                double softLimit = stepAbs > Math.toRadians(50) ? Math.min(0.28, Math.toRadians(16.0) / stepAbs) : 0.34;
                double overshootMargin = (softLimit - response.overshoot()) / softLimit * 0.34;
                double maxOscDeg = Math.max(22.0, 0.60 * Math.toDegrees(stepAbs));
                double oscMargin = 0.34;
                if (run.reversals >= 3) {
                    oscMargin = Math.min(oscMargin, -0.08 * (run.reversals - 2));
                }
                if (oscDeg > maxOscDeg) {
                    oscMargin = Math.min(oscMargin, -0.02 * (oscDeg - maxOscDeg));
                }
                double stepMargin = Math.min(overshootMargin, oscMargin);
                margin = Math.min(margin, stepMargin);
                if (firstFailure.isEmpty()) {
                    if (response.overshoot() > softLimit) {
                        firstFailure = String.format(Locale.US, "%s 超调 %.0f%%（%.1f°，上限 %.0f%%）",
                                step, response.overshoot() * 100, overDeg, softLimit * 100);
                    } else if (run.reversals >= 3 || oscDeg > maxOscDeg) {
                        firstFailure = String.format(Locale.US, "%s 震荡 %d 次（多余行程 %.1f°）",
                                step, run.reversals, oscDeg);
                    } else if (!arrived) {
                        firstFailure = String.format(Locale.US, "%s 未停稳，末端误差 %+.1f°，速度 %+.0f°/s", step,
                                Math.toDegrees(run.endError), Math.toDegrees(run.endSpeed));
                    }
                }
                double timeCost = 3.0 * (effectiveSettle / stepDuration) + (arrived ? 0.0 : 2.2);
                double reversalPenalty = run.reversals == 0 ? 0.0
                        : (run.reversals == 1 ? 0.25 : 0.25 + 0.90 * (run.reversals - 1));
                double overshootPenalty = 1.6 * response.overshoot() + 0.06 * Math.max(0.0, overDeg - 1.5);
                double travelPenalty = 0.055 * Math.max(0.0, oscDeg - 3.0);
                double endErrCost = 0.12 * Math.max(0.0, endErrDeg - 0.8);
                double cost = Math.min(9.0, timeCost + reversalPenalty + overshootPenalty + travelPenalty + endErrCost);
                total += cost;
                worst = Math.max(worst, cost);
                settled &= arrived;
            }
            double score = 0.70 * total / targets.length + 0.30 * worst;
            op.telemetry.log().add("云台第 %d 轮 %.3f，用时 %.2fs，反转 %d 次，裕量 %+.2f%s",
                    trials, score, totalSettle, maxReversals, margin, settled && margin >= 0 ? "，停稳" : "");
            RobotLog.ii("BioBuzzCalibration",
                    "Turret trial %d (%s): P=%.3f I=%.3f D=%.3f score=%.3f time=%.2fs maxRev=%d margin=%+.3f settled=%b steady=%.2f deg fail=%s",
                    trials, verification ? "verify" : "search", gains[0], gains[1], gains[2],
                    score, totalSettle, maxReversals, margin, settled, Math.toDegrees(steadyError), firstFailure);
            return new BayesianPidSearch.Result(score, margin, settled, firstFailure, steadyError);
        }

        private boolean active() { return op.opModeIsActive() && !op.motionAborted(); }
    }

    private static final class RewindCheck {
        private final CalibrationOp op;
        private final Turret turret;
        private int passedTurns;
        private boolean skipped;

        RewindCheck(CalibrationOp op, Turret turret) {
            this.op = op;
            this.turret = turret;
        }

        static boolean supported(Turret turret) { return turret.span() >= Math.toRadians(270); }
        boolean passed() { return passedTurns == 2; }
        boolean skipped() { return skipped; }

        String run(boolean ask) throws InterruptedException {
            if (ask && !op.confirm("回转验证（可选）：轮子落地并清空周围。A 运行双向回转验收；B 跳过回转直接进入阶跃复测保存。")) {
                skipped = true;
                return "已跳过（按留出阶跃验收）";
            }
            Drive drive;
            try {
                drive = new Drive(op.hardwareMap);
            } catch (RuntimeException unavailable) {
                skipped = true;
                return "找不到底盘，已跳过";
            }
            PinpointPose pose = new PinpointPose(op.hardwareMap, op.battery);
            try {
                if (!op.initializePinpoint(pose)) return "Pinpoint 未就绪";
                return run(drive, pose);
            } finally {
                drive.stop();
                turret.stop();
            }
        }

        String run(Drive drive, PinpointPose pose) throws InterruptedException {
            try {
                pose.update();
                if (!pose.anchor(0, 0, 0)) return "Pinpoint 航向未就绪";
                String left = turn(drive, pose, 1);
                if (!active()) return left;
                return left + "；" + turn(drive, pose, -1);
            } finally {
                drive.stop();
                turret.stop();
            }
        }

        private String turn(Drive drive, PinpointPose pose, int yawSign) throws InterruptedException {
            double edgeDistance = Math.toRadians(17);
            double travel = turret.span() - 2 * RobotConfig.TURRET_MARGIN;
            double start = RobotConfig.TURRET_MARGIN + edgeDistance;
            if (yawSign < 0) start = turret.span() - start;
            String label = yawSign > 0 ? "底盘左转" : "底盘右转";
            if (!TurretBench.goDirect(op, turret, start, label)) return label + "未到达预备角";
            pose.update();
            if (!pose.headingValid || !pose.headingUsable()) return label + "没有航向，" + pose.status;
            double startHeading = pose.heading;
            double turnLimit = Maths.TURN - travel + edgeDistance + Math.toRadians(13);
            boolean[] flags = new boolean[4];
            double[] peak = {0};
            double[] timing = {Double.NaN, Double.NaN};
            double[] settled = {0};
            String[] failure = {"未到位"};
            try {
                op.runFor(6.5, (t, dt) -> {
                    pose.update();
                    turret.sample(dt);
                    if (!pose.headingUsable() || !turret.encoderReady() || !turret.motorResponding()) {
                        failure[0] = !pose.headingUsable() ? pose.status : "云台反馈中断";
                        return false;
                    }
                    double turned = yawSign * Maths.wrap(pose.heading - startHeading);
                    double stick = pose.headingValid && t >= 0.14 && turned < turnLimit ? yawSign * 0.32 : 0;
                    double effective = drive.robot(0, 0, stick);
                    turret.aim(pose, 0, 0, 0, 0, effective, dt);
                    if (turret.rewinding()) {
                        if (!flags[0]) timing[0] = t;
                        flags[0] = true;
                        flags[1] |= turret.rewindPredicted();
                        flags[2] |= Math.abs(turret.brakeDuty()) > 0.024;
                        peak[0] = Math.max(peak[0], Math.abs(turret.output()));
                    }
                    if (flags[0] && !turret.rewinding() && !Double.isFinite(timing[1])) timing[1] = t - timing[0];
                    op.show("PID 回转 " + label, "底盘 %+.0f°/s / 云台 %+.0f°/s / 功率 %.2f / 峰值 %.2f",
                            Math.toDegrees(pose.yawRate), Math.toDegrees(turret.speed), turret.output(), peak[0]);
                    boolean aligned = pose.headingValid && flags[0] && turned >= turnLimit && turret.aimReady()
                            && Math.abs(turret.target - turret.angle) <= RobotConfig.SHOT_AIM_TOLERANCE
                            && Math.abs(pose.yawRate) < 0.25 && Math.abs(turret.speed) < 0.32;
                    settled[0] = aligned ? settled[0] + dt : 0;
                    if (settled[0] >= 0.12) {
                        flags[3] = true;
                        return false;
                    }
                    return true;
                });
            } finally {
                drive.stop();
                turret.stop();
            }
            if (flags[3]) passedTurns++;
            String duration = Double.isFinite(timing[1]) ? String.format(Locale.US, "%.2f 秒", timing[1]) : failure[0];
            if (!flags[3] && Double.isFinite(timing[1])) duration += "，" + failure[0];
            return String.format(Locale.US, "%s%s，%s，峰值 %.2f，%s，%s", label, flags[3] ? "回转完成" : "未完成",
                    flags[1] ? "提前触发" : "未提前触发", peak[0], flags[2] ? "有刹车" : "未进入刹车", duration);
        }

        private boolean active() { return op.opModeIsActive() && !op.motionAborted(); }
    }
}
