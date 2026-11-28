package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.biobuzz.calibration.CalibrationOp.Measurements;
import org.firstinspires.ftc.teamcode.biobuzz.calibration.CalibrationOp.StepScore;
import org.firstinspires.ftc.teamcode.biobuzz.config.CalibrationStore;
import org.firstinspires.ftc.teamcode.biobuzz.config.RobotConfig;
import org.firstinspires.ftc.teamcode.biobuzz.core.BatteryVoltage;
import org.firstinspires.ftc.teamcode.biobuzz.core.Maths;
import org.firstinspires.ftc.teamcode.biobuzz.core.Maths.LeastSquares;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Intake;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.function.IntSupplier;

@TeleOp(name = "04 飞轮校准 12V前馈·PID·回速", group = "BIOBUZZ 自测")
public final class FlywheelCalibration extends CalibrationOp {
    private static final double[] VOLTAGE_LEVELS = {3.0, 4.5, 6.0, 7.2, 8.4, 9.6, 10.8, 12.0};
    private static final double[] VERIFY_RATIOS = {0.35, 0.48, 0.60, 0.72, 0.83, 0.92};
    private static final double[] SEARCH_BASE_RATIOS = {0.58, 0.64, 0.60};
    private static final double[] VERIFY_BASE_RATIOS = {0.56, 0.70};
    private static final double[] PID_MIN = {0.6, 0.2, 0.0};
    private static final double[] PID_MAX = {5.5, 5.0, 0.12};
    private static final double[] PID_SCALE = {1.5, 1.0, 0.025};
    private static final double[] PID_SAFE_CENTER = {0.33, 0.25, 0.08};
    private static final double[] POSITIONING_PID = {2.2, 1.6, 0.005};

    private final List<String> recoveries = new ArrayList<>();
    private double curveLimit;
    private String modelSummary = "沿用已有曲线";
    private String pidSummary = "未运行";
    private BayesianPidSearch completedPidSearch;
    private int pidTrials;

    @Override String intro() {
        return "完成方向、每转计数、12V 前馈曲线+闭环校验、PID 贝叶斯拟合与三球回速。";
    }

    @Override void calibrate() throws InterruptedException {
        Flywheel wheel = new Flywheel(hardwareMap, store, battery);
        boolean refitFeedforward = false;
        try {
            if (confirm("A 从手转识别开始全套校准；B 沿用已有方向和每转计数。")) {
                if (!calibrateDirection(wheel)) return;
                wheel = new Flywheel(hardwareMap, store, battery);
                if (wheel.ticksPerRev <= 0 || confirm("A 重测每转计数，B 使用当前 RPM 比例。")) {
                    if (!calibrateScale(wheel)) return;
                }
                wheel = new Flywheel(hardwareMap, store, battery);
            }
            if (confirm("确认供弹通道为空、手已移开。A 开始 8 档电压扫描（3V–12V）+ 闭环前馈校验；B 跳过直接进 PID / 回速。")) {
                if (!fitModel(wheel, battery)) return;
                refitFeedforward = true;
                wheel = new Flywheel(hardwareMap, store, battery);
            }
            if (!wheel.rpmCalibrated()) {
                telemetry.log().add("缺少 RPM 比例或电压曲线。");
                return;
            }
            curveLimit = wheel.maxTargetRpm();
            if (confirm("确认供弹通道为空。A 开始飞轮 PID 贝叶斯拟合与留出复测（约1分钟）；B 跳过保持当前 PID。")) {
                if (tunePid(wheel, refitFeedforward)) {
                    wheel = new Flywheel(hardwareMap, store, battery);
                }
            }
            if (!active() || wheel.faulted()) return;
            if (wheel.hasBlocker() && confirm("可装入最多三颗球，移开手并朝向得分区。A 开始三球回速实测，B 结束。")) {
                testRecovery(wheel);
            }
        } finally {
            wheel.stop();
        }
    }

    @Override void showResults() {
        telemetry.addData("飞轮前馈", modelSummary);
        telemetry.addData("飞轮 PID", pidSummary);
        if (curveLimit > 0) telemetry.addData("目标上限", "%.0f RPM", curveLimit);
        if (completedPidSearch != null) {
            telemetry.addData("飞轮 PID 搜索", "历史 %d / 本轮 %d，最低 %s",
                    completedPidSearch.historyCount(), completedPidSearch.searchCount(),
                    Double.isFinite(completedPidSearch.bestSearchCost())
                            ? String.format(Locale.US, "%.3f", completedPidSearch.bestSearchCost()) : "无");
            showPidReview("进入时 PID", completedPidSearch.incumbent());
            BayesianPidSearch.Review bestNew = completedPidSearch.bestCandidate();
            showPidReview("最佳新 PID", bestNew != null ? bestNew : completedPidSearch.bestFallbackCandidate());
        }
        for (String recovery : recoveries) telemetry.addLine(recovery);
    }

    private boolean active() { return opModeIsActive() && !motionAborted(); }

    private boolean calibrateDirection(Flywheel wheel) throws InterruptedException {
        wheel.stop();
        int encoderSign = readDirection(RobotConfig.FLYWHEEL_ENCODER_MOTOR, wheel::ticks, wheel.encoderDirection());
        if (encoderSign == 0) return false;

        int leftSign;
        if (confirm("m-7 若有编码器，A 手转独立识别。只有 m-8 有编码器时，B 按现有两电机的相对方向推算。")) {
            leftSign = readDirection(RobotConfig.FLYWHEEL_LEFT, wheel::leftTicks, wheel.leftDirection());
            if (leftSign == 0) return false;
        } else {
            leftSign = encoderSign * directionSign(wheel.leftDirection()) * directionSign(wheel.encoderDirection());
        }
        if (!opModeIsActive() || !save(CalibrationStore.FLYWHEEL_DIRECTIONS, leftSign, encoderSign)) return false;
        telemetry.log().add("飞轮出球转向 m-7 %s / m-8 %s",
                leftSign > 0 ? Direction.FORWARD : Direction.REVERSE,
                encoderSign > 0 ? Direction.FORWARD : Direction.REVERSE);
        return true;
    }

    private int readDirection(String motor, IntSupplier ticks, Direction direction) throws InterruptedException {
        if (!confirm(motor + " 停稳后按 A 记录起点。手转阶段电机保持零功率。")) return 0;
        int previous = ticks.getAsInt();
        int firstSign = 0;
        while (confirm(motor + " 手转 2–3 圈。让贴着弹道的轮面朝出球口移动，停稳后按 A。")) {
            int current = ticks.getAsInt();
            int delta = current - previous;
            previous = current;
            if (Math.abs((long) delta) < 2) {
                telemetry.log().add("%s 编码器计数不足，检查接线后再试。", motor);
                continue;
            }
            int sign = Integer.signum(delta);
            if (firstSign == sign) return sign * directionSign(direction);
            if (firstSign == 0) telemetry.log().add("已记录 %s，请沿同一方向再转一次。", motor);
            else telemetry.log().add("%s 两次方向不同，请沿出球方向重复。", motor);
            firstSign = sign;
        }
        return 0;
    }

    private static int directionSign(Direction direction) {
        return direction == Direction.FORWARD ? 1 : -1;
    }

    private boolean calibrateScale(Flywheel wheel) throws InterruptedException {
        Measurements scales = new Measurements();
        for (int pass = 0; pass < 4 && opModeIsActive(); pass++) {
            int turns = pass % 2 == 0 ? 5 : 3;
            if (!confirm("标记已对齐？A 记下起点。")) break;
            int from = wheel.ticks();
            if (!confirm("沿同一方向用手转飞轮整 " + turns + " 圈，停在标记位置后按 A。")) break;
            double observed = Math.abs((double) (wheel.ticks() - from)) / turns;
            if (observed >= 1 && observed <= 500000) scales.add(observed);
            else telemetry.log().add("编码器变化无效，检查接线后继续。");
            if (scales.size() >= 2 && scales.relativeSpread() <= 0.04) {
                if (!save(CalibrationStore.FLYWHEEL_SCALE, scales.median())) return false;
                telemetry.log().add("每飞轮转 %.1f ticks", scales.median());
                return true;
            }
        }
        telemetry.log().add("RPM 比例缺少一致样本，本轮结束。");
        return false;
    }

    private boolean fitModel(Flywheel wheel, BatteryVoltage battery) throws InterruptedException {
        List<double[]> points = new ArrayList<>();
        try {
            for (double volts : VOLTAGE_LEVELS) {
                if (!active() || wheel.faulted()) return false;
                double[] row = measurePlateau(wheel, battery, volts);
                if (row != null && (points.isEmpty() || row[1] - points.get(points.size() - 1)[1] > 0.10)) {
                    points.add(row);
                    telemetry.log().add("%.2f V → %.0f ticks/s", row[1], row[0]);
                    RobotLog.ii("BioBuzzCalibration", "Flywheel FF open-loop %.2f V -> %.0f ticks/s", row[1], row[0]);
                }
            }
        } finally {
            wheel.stop();
        }
        if (!active() || wheel.faulted()) return false;
        double highestVolts = points.isEmpty() ? 0 : points.get(points.size() - 1)[1];
        double highestSpeed = points.isEmpty() ? 0 : points.get(points.size() - 1)[0];
        if (points.size() < 4 || highestVolts < 7.5 || highestSpeed <= 20) {
            telemetry.log().add("有效稳态点不足（%d 组，最高 %.2f V / %.0f ticks/s）。检查编码器及电压后重测，保留旧曲线。",
                    points.size(), highestVolts, highestSpeed);
            RobotLog.ww("BioBuzzCalibration", "Flywheel FF insufficient points: count=%d maxV=%.2f maxSpd=%.0f",
                    points.size(), highestVolts, highestSpeed);
            return false;
        }
        double ref = highestSpeed;
        if (points.get(0)[0] > 0.92 * ref) {
            telemetry.log().add("最低与最高档的稳态转速过于接近（%.0f vs %.0f ticks/s），保留旧前馈曲线。",
                    points.get(0)[0], ref);
            RobotLog.ww("BioBuzzCalibration", "Flywheel FF span too narrow: minSpd=%.0f maxSpd=%.0f",
                    points.get(0)[0], ref);
            return false;
        }
        double[] initialCurve = fitCurve(points, ref, true);
        if (!Flywheel.validModel(initialCurve)) initialCurve = fitCurve(points, ref, false);
        if (!Flywheel.validModel(initialCurve)) {
            telemetry.log().add("曲线拟合无效，保留旧曲线。");
            return false;
        }
        double[] refined = wheel.ticksPerRev > 0
                ? refineFeedforward(wheel, points, initialCurve)
                : initialCurve;
        if (refined == null || !active() || wheel.faulted()) return false;
        if (!save(CalibrationStore.FLYWHEEL_VOLTAGE_MODEL, refined)) return false;
        RobotLog.ii("BioBuzzCalibration",
                "Flywheel FF saved: ref=%.1f kS=%.3f V kV=%.3f V kQ=%.3f V (%s)",
                refined[0], refined[1], refined[2], refined[3], modelSummary);
        return true;
    }

    private double[] refineFeedforward(Flywheel wheel, List<double[]> openLoopPoints, double[] initialCurve)
            throws InterruptedException {
        double[] savedModel = wheel.voltageModel();
        double[] savedPid = wheel.pidGains();
        List<double[]> combined = new ArrayList<>(openLoopPoints);
        List<double[]> verifySamples = new ArrayList<>();
        try {
            wheel.stop();
            wheel.setVoltageModel(initialCurve);
            wheel.setPidGains(new double[]{2.4, 2.2, 0.0});
            double maxRpm = wheel.maxTargetRpmAtBattery();
            double previousTarget = 0;
            for (double ratio : VERIFY_RATIOS) {
                if (!active() || wheel.faulted()) return null;
                double targetRpm = Math.min(0.92 * maxRpm, Math.max(1200.0, ratio * maxRpm));
                if (targetRpm - previousTarget < 120.0) continue;
                previousTarget = targetRpm;
                double[] accum = new double[3]; // [holdTime, speedSum, voltsSum]
                Measurements tailSpeeds = new Measurements();
                Measurements tailVolts = new Measurements();
                runFor(3.2, (t, dt) -> {
                    wheel.sample();
                    if (wheel.faulted()) return false;
                    wheel.requestRpm(targetRpm);
                    double rpm = wheel.rpm();
                    double errRpm = Math.abs(rpm - targetRpm);
                    if (t >= 0.85) {
                        if (errRpm <= Math.max(55.0, 0.025 * targetRpm)) {
                            accum[0] += dt;
                            accum[1] += wheel.speed * dt;
                            accum[2] += wheel.deliveredVolts() * dt;
                        } else if (errRpm > Math.max(110.0, 0.055 * targetRpm)) {
                            accum[0] = accum[1] = accum[2] = 0;
                        }
                    }
                    if (t >= 2.00 && wheel.speed > 10 && wheel.deliveredVolts() >= 1.0) {
                        tailSpeeds.add(wheel.speed);
                        tailVolts.add(wheel.deliveredVolts());
                    }
                    show("飞轮闭环前馈校验", "目标 %.0f / 实际 %.0f RPM / %.2f V",
                            targetRpm, rpm, wheel.deliveredVolts());
                    return accum[0] < 0.55;
                });
                if (!active() || wheel.faulted()) return null;
                double avgSpeed = Double.NaN, avgVolts = Double.NaN;
                if (accum[0] >= 0.55) {
                    avgSpeed = accum[1] / accum[0];
                    avgVolts = accum[2] / accum[0];
                } else if (tailSpeeds.size() >= 5 && tailVolts.size() >= 5) {
                    double medSpeed = tailSpeeds.median();
                    double medVolts = tailVolts.median();
                    double medRpm = medSpeed * 60.0 / wheel.ticksPerRev;
                    if (Math.abs(medRpm - targetRpm) <= Math.max(150.0, 0.08 * targetRpm) && medVolts >= 1.0) {
                        avgSpeed = medSpeed;
                        avgVolts = medVolts;
                        RobotLog.ii("BioBuzzCalibration",
                                "Flywheel FF verify %.0f RPM tail-median fallback -> %.0f ticks/s at %.2f V",
                                targetRpm, avgSpeed, avgVolts);
                    }
                }
                if (Double.isFinite(avgSpeed) && Double.isFinite(avgVolts)) {
                    combined.add(new double[]{avgSpeed, avgVolts, 4.0});
                    verifySamples.add(new double[]{avgSpeed, avgVolts});
                    RobotLog.ii("BioBuzzCalibration",
                            "Flywheel FF verify %.0f RPM -> steady %.0f ticks/s at %.2f V",
                            targetRpm, avgSpeed, avgVolts);
                }
            }
        } finally {
            wheel.stop();
            if (savedModel != null) wheel.setVoltageModel(savedModel);
            wheel.setPidGains(savedPid);
        }

        double[] finalCurve = initialCurve;
        String fitSource = "仅开环";
        double ref = initialCurve[0];
        for (double[] row : combined) ref = Math.max(ref, row[0]);
        if (verifySamples.size() >= 4) {
            double[] direct = fitCurve(verifySamples, ref, true);
            if (!Flywheel.validModel(direct)) direct = fitCurve(verifySamples, ref, false);
            if (Flywheel.validModel(direct)) {
                finalCurve = direct;
                fitSource = "闭环直拟";
            }
        }
        if ("仅开环".equals(fitSource) && verifySamples.size() >= 2) {
            double[] refitted = fitCurve(combined, ref, true);
            if (!Flywheel.validModel(refitted)) refitted = fitCurve(combined, ref, false);
            if (Flywheel.validModel(refitted)) {
                finalCurve = refitted;
                fitSource = "联合拟合";
            }
        }

        if (verifySamples.isEmpty()) {
            modelSummary = String.format(Locale.ROOT,
                    "已保存 kS=%.2f V, kV=%.2f V, kQ=%.2f V（仅开环）",
                    finalCurve[1], finalCurve[2], finalCurve[3]);
            return finalCurve;
        }

        double maxVoltsErr = 0, sqVoltsErr = 0, maxRpmErr = 0;
        for (double[] sample : verifySamples) {
            double spd = sample[0], actualV = sample[1];
            double rate = spd / finalCurve[0];
            double predV = finalCurve[1] + rate * (finalCurve[2] + rate * finalCurve[3]);
            double errV = Math.abs(actualV - predV);
            maxVoltsErr = Math.max(maxVoltsErr, errV);
            sqVoltsErr += errV * errV;
            double dVdRate = Math.max(0.5, finalCurve[2] + 2.0 * finalCurve[3] * rate);
            double rpmPerRate = finalCurve[0] * 60.0 / wheel.ticksPerRev;
            maxRpmErr = Math.max(maxRpmErr, errV * rpmPerRate / dVdRate);
        }
        double rmsVoltsErr = Math.sqrt(sqVoltsErr / verifySamples.size());
        modelSummary = String.format(Locale.ROOT,
                "已保存（%s）kS=%.2f V, kV=%.2f V, kQ=%.2f V | 闭环残差 RMS %.2f V / max %.2f V (约 ±%.0f RPM)",
                fitSource, finalCurve[1], finalCurve[2], finalCurve[3], rmsVoltsErr, maxVoltsErr, maxRpmErr);
        telemetry.log().add("%s", modelSummary);
        return finalCurve;
    }

    private static double[] fitCurve(List<double[]> points, double ref, boolean quadratic) {
        LeastSquares fit = new LeastSquares(quadratic ? new double[]{1e-6, 1e-6, 0.001}
                : new double[]{1e-6, 1e-6}, null);
        for (double[] point : points) {
            double rate = point[0] / ref;
            double weight = point.length >= 3 ? point[2] : 1.0;
            fit.add(quadratic ? new double[]{1, rate, rate * rate} : new double[]{1, rate}, point[1], weight);
        }
        double[] values = fit.solve();
        return values == null ? null : new double[]{ref, Math.max(0, values[0]), values[1], quadratic ? values[2] : 0};
    }

    private double[] measurePlateau(Flywheel wheel, BatteryVoltage battery, double volts) throws InterruptedException {
        final double minSettle = (volts == VOLTAGE_LEVELS[0]) ? 1.50 : 1.00;
        final double windowDuration = 0.45;
        final double maxDuration = 4.6;
        final double tailStart = 2.8;
        final double[] bin = {0, 0, 0};
        final double[] prevMean = {Double.NaN, Double.NaN};
        final double[] steady = {Double.NaN, Double.NaN};
        final boolean[] stalledLow = {false};
        final Measurements tailSpeeds = new Measurements();
        final Measurements tailVolts = new Measurements();

        runFor(maxDuration, (t, dt) -> {
            wheel.sample();
            if (wheel.faulted()) return false;
            wheel.powerVolts(volts, dt);
            double delivered = wheel.deliveredVolts();
            if (t >= 1.5 && volts == VOLTAGE_LEVELS[0] && wheel.speed <= 10) {
                stalledLow[0] = true;
                return false;
            }
            if (t >= minSettle && wheel.speed > 10 && delivered >= 1.0) {
                bin[0] += dt;
                bin[1] += wheel.speed * dt;
                bin[2] += delivered * dt;
                if (bin[0] >= windowDuration) {
                    double meanSpeed = bin[1] / bin[0];
                    double meanVolts = bin[2] / bin[0];
                    if (Double.isFinite(prevMean[0])
                            && Math.abs(meanSpeed - prevMean[0]) <= Math.max(10.0, 0.012 * Math.max(meanSpeed, prevMean[0]))
                            && Math.abs(meanVolts - prevMean[1]) <= 0.30) {
                        steady[0] = meanSpeed;
                        steady[1] = meanVolts;
                        return false;
                    }
                    prevMean[0] = meanSpeed;
                    prevMean[1] = meanVolts;
                    bin[0] = bin[1] = bin[2] = 0;
                }
            }
            if (t >= tailStart && wheel.speed > 10 && delivered >= 1.0) {
                tailSpeeds.add(wheel.speed);
                tailVolts.add(delivered);
            }
            show("飞轮空载曲线", "%.1f V / 实际 %.2f V / %.0f ticks/s", volts, delivered, wheel.speed);
            return true;
        });

        if (!active() || wheel.faulted() || stalledLow[0]) return null;
        if (Double.isFinite(steady[0]) && Double.isFinite(steady[1])) {
            return new double[]{steady[0], steady[1], 1.0};
        }
        if (tailSpeeds.size() >= 5 && tailVolts.size() >= 5) {
            double medSpeed = tailSpeeds.median();
            double medVolts = tailVolts.median();
            if (medSpeed > 10 && medVolts >= 1.0) {
                RobotLog.ii("BioBuzzCalibration",
                        "Flywheel FF open-loop %.1f V tail-median fallback -> %.2f V / %.0f ticks/s (%d samples)",
                        volts, medVolts, medSpeed, tailSpeeds.size());
                return new double[]{medSpeed, medVolts, 1.0};
            }
        }
        return null;
    }

    private boolean tunePid(Flywheel wheel, boolean freshCurve) throws InterruptedException {
        double[] original = wheel.pidGains();
        double[] incumbent = {
                Maths.clip(original[0], PID_MIN[0], PID_MAX[0]),
                Maths.clip(original[1], PID_MIN[1], PID_MAX[1]),
                Maths.clip(original[2], PID_MIN[2], PID_MAX[2])
        };
        BayesianPidSearch search = new BayesianPidSearch(PID_MIN, PID_MAX, PID_SCALE, PID_SAFE_CENTER);
        completedPidSearch = search;
        pidTrials = 0;
        boolean saved = false;
        double[] history = freshCurve ? new double[0]
                : store.readHistory(CalibrationStore.FLYWHEEL_PID_HISTORY, BayesianPidSearch.HISTORY_STRIDE,
                        BayesianPidSearch.HISTORY_ROWS);
        RobotLog.ii("BioBuzzCalibration",
                "Flywheel PID search start: incumbent=[%.3f, %.3f, %.3f] historyRows=%d freshCurve=%b",
                incumbent[0], incumbent[1], incumbent[2], history.length / BayesianPidSearch.HISTORY_STRIDE, freshCurve);
        try {
            search.tune(incumbent,
                    (gains, verification, pattern) -> flywheelTrial(wheel, gains, verification, pattern),
                    () -> !active() || wheel.faulted(), history);
            if (!active() || wheel.faulted()) {
                pidSummary = wheel.faulted() ? "飞轮传感器/电机故障" : "飞轮 PID 测量已停止";
                return false;
            }
            if (search.searchCount() > 0) save(CalibrationStore.FLYWHEEL_PID_HISTORY, search.history());

            BayesianPidSearch.Review best = search.bestCandidate();
            BayesianPidSearch.Review baseReview = search.incumbent();
            if (baseReview != null && baseReview.qualified() && (best == null || baseReview.score <= best.score)) {
                best = baseReview;
            }
            boolean fallbackCandidate = false;
            if (best == null) {
                BayesianPidSearch.Review fallback = search.bestFallbackCandidate();
                if (fallback == null) {
                    pidSummary = "本轮缺少通过留出复测的 PID，保留原参数";
                    RobotLog.ww("BioBuzzCalibration", "Flywheel PID search failed: no candidate qualified");
                    return false;
                }
                best = fallback;
                fallbackCandidate = true;
            }
            final BayesianPidSearch.Review selected = best;
            String oldSteadyText = baseReview != null && Double.isFinite(baseReview.steadyError)
                    ? String.format(Locale.ROOT, "%.0f RPM (%s)", baseReview.steadyError,
                            baseReview.qualified() ? "通过" : baseReview.passed + "/" + BayesianPidSearch.VERIFICATION_ROUNDS)
                    : "未通过";
            String newSteadyText = Double.isFinite(selected.steadyError)
                    ? String.format(Locale.ROOT, "%.0f RPM", selected.steadyError) : "未知";
            String prompt = fallbackCandidate
                    ? String.format(Locale.ROOT, "留出复测未满 %d/%d（可对比旧/新 PID）。A 仍保存最佳新 PID，B 保留原 PID。",
                            BayesianPidSearch.VERIFICATION_ROUNDS, BayesianPidSearch.VERIFICATION_ROUNDS)
                    : "请选择 PID。A 保存最佳新值，B 保留旧值。代价越低越好。";
            if (selected != baseReview && !choosePid(prompt, search, selected,
                    () -> telemetry.addData("稳态误差对比", "旧 %s / 新 %s", oldSteadyText, newSteadyText))) {
                pidSummary = "用户选择保留原 PID";
                return false;
            }
            saved = active() && save(CalibrationStore.FLYWHEEL_VOLTAGE_PID, selected.gains);
            double displayCost = selected.qualified() ? selected.score : selected.measuredScore;
            String label = selected == baseReview ? "原 PID 通过复测"
                    : selected.qualified() ? "已保存新 12V PID"
                    : String.format(Locale.ROOT, "已确认保存新 12V PID（复测 %d/%d）",
                            selected.passed, BayesianPidSearch.VERIFICATION_ROUNDS);
            pidSummary = saved
                    ? String.format(Locale.ROOT, "%s：P/I/D %.3f / %.3f / %.3f（代价 %.3f，稳态误差 %s）",
                            label, selected.gains[0], selected.gains[1], selected.gains[2], displayCost, newSteadyText)
                    : "PID 写入失败";
            telemetry.log().add("%s", pidSummary);
            RobotLog.ii("BioBuzzCalibration", "Flywheel PID result (saved=%b): %s", saved, pidSummary);
            return saved;
        } finally {
            wheel.stop();
            if (!saved) wheel.setPidGains(original);
        }
    }

    private BayesianPidSearch.Result flywheelTrial(Flywheel wheel, double[] gains, boolean verification, int pattern)
            throws InterruptedException {
        double maxRpm = wheel.maxTargetRpmAtBattery();
        if (maxRpm < 1600) {
            return BayesianPidSearch.Result.unavailable("电池电压下飞轮上限过低");
        }
        double baseRatio = verification
                ? VERIFY_BASE_RATIOS[pattern % VERIFY_BASE_RATIOS.length]
                : SEARCH_BASE_RATIOS[pattern % SEARCH_BASE_RATIOS.length];
        double deltaRpm = verification
                ? Maths.clip(0.11 * maxRpm, 240.0, 400.0)
                : Maths.clip(0.10 * maxRpm, 220.0, 380.0);
        double baseRpm = Math.min(maxRpm - deltaRpm - 80.0, baseRatio * maxRpm);
        if (baseRpm < 1000.0) {
            baseRpm = Math.max(900.0, 0.50 * maxRpm);
            deltaRpm = Math.min(deltaRpm, 0.20 * maxRpm);
        }
        double highRpm = Math.min(0.95 * maxRpm, baseRpm + deltaRpm);
        deltaRpm = highRpm - baseRpm;
        if (deltaRpm < 150.0) {
            return BayesianPidSearch.Result.unavailable("阶跃幅值不足");
        }

        pidTrials++;
        String mode = (verification ? "飞轮 PID 复测 " : "飞轮 PID 搜索 ") + pidTrials;
        final double settleHold = 0.38;
        final double evalHold = 0.15;

        try {
            wheel.setRecoveryBoostEnabled(false);
            // Pre-spin smoothly to baseRpm using known-good positioning gains
            wheel.setPidGains(POSITIONING_PID);
            double[] preSpinHold = {0};
            final double finalBaseRpm = baseRpm;
            runFor(4.0, (t, dt) -> {
                wheel.sample();
                if (wheel.faulted()) return false;
                wheel.requestRpm(finalBaseRpm);
                double rpm = wheel.rpm();
                double err = Math.abs(rpm - finalBaseRpm);
                if (err <= Math.max(70.0, 0.035 * finalBaseRpm)) preSpinHold[0] += dt;
                else if (err > Math.max(110.0, 0.06 * finalBaseRpm)) preSpinHold[0] = 0;
                show(mode + " · 预热就位", "目标 %.0f / 当前 %.0f RPM", finalBaseRpm, rpm);
                return preSpinHold[0] < 0.20;
            });
            if (!active() || wheel.faulted()) return null;
            double currentRpm = wheel.rpm();
            if (preSpinHold[0] < 0.20
                    && (Math.abs(currentRpm - finalBaseRpm) > Math.max(150.0, 0.08 * finalBaseRpm)
                            || highRpm - currentRpm < 120.0)) {
                return BayesianPidSearch.Result.unavailable("基线转速未就位");
            }
            final double startBaseRpm = preSpinHold[0] >= 0.20 ? finalBaseRpm : currentRpm;
            final double finalDeltaRpm = highRpm - startBaseRpm;

            // Subtest A: Closed-loop upward step response
            wheel.setPidGains(gains);
            double durationA = 2.4;
            StepScore scoreA = new StepScore(durationA);
            double[] stateA = {wheel.rpm(), wheel.rpm(), 0}; // [peakRpm, endRpm, maxOvershoot]
            double[] quietA = {0, 0}; // [time, errSum]
            double[] tailA = {0, 0};  // [time, errSum]
            boolean[] flagsA = new boolean[2]; // [readFailed, unsafe]
            runFor(durationA, (t, dt) -> {
                wheel.sample();
                if (wheel.faulted()) {
                    flagsA[0] = true;
                    return false;
                }
                wheel.requestRpm(highRpm);
                double rpm = wheel.rpm();
                stateA[0] = Math.max(stateA[0], rpm);
                stateA[1] = rpm;
                stateA[2] = Math.max(0.0, stateA[0] - highRpm);
                if (stateA[2] > Math.max(380.0, 0.90 * finalDeltaRpm)) {
                    flagsA[1] = true;
                    return false;
                }
                double progress = (rpm - startBaseRpm) / finalDeltaRpm;
                boolean within = Math.abs(rpm - highRpm) <= Math.max(65.0, 0.025 * highRpm);
                if (within) {
                    quietA[0] += dt;
                    quietA[1] += (rpm - highRpm) * dt;
                } else {
                    quietA[0] = quietA[1] = 0;
                }
                if (t >= durationA * 0.70) {
                    tailA[0] += dt;
                    tailA[1] += (rpm - highRpm) * dt;
                }
                scoreA.sample(t, dt, progress, wheel.deliveredVolts() / BatteryVoltage.NOMINAL_VOLTS, within);
                show(mode + " · 阶跃", "目标 %.0f / 当前 %.0f RPM / %.2f V", highRpm, rpm, wheel.deliveredVolts());
                return !scoreA.finishIfSettled(settleHold);
            });
            if (flagsA[0] || wheel.faulted()) return BayesianPidSearch.Result.unavailable("飞轮读数中断");
            if (!active()) return null;
            if (flagsA[1]) {
                return new BayesianPidSearch.Result(9.0, -0.25, false,
                        String.format(Locale.US, "阶跃超调 +%.0f RPM", stateA[2]));
            }

            // Subtest B: Simulated ball-drop recovery (pure closed-loop PID)
            double targetDrop = Maths.clip(0.08 * highRpm, 180.0, 280.0);
            double[] lowestRpm = {wheel.rpm()};
            runFor(0.65, (t, dt) -> {
                wheel.sample();
                if (wheel.faulted()) {
                    flagsA[0] = true;
                    return false;
                }
                wheel.pulseCoast();
                lowestRpm[0] = Math.min(lowestRpm[0], wheel.rpm());
                show(mode + " · 模拟掉速", "降至 %.0f / 目标 %.0f RPM", wheel.rpm(), highRpm);
                return wheel.rpm() > highRpm - targetDrop;
            });
            if (flagsA[0] || wheel.faulted()) return BayesianPidSearch.Result.unavailable("飞轮读数中断");
            if (!active()) return null;

            double dropDepth = Math.max(60.0, highRpm - lowestRpm[0]);
            final double effectiveDrop = Math.max(140.0, dropDepth);
            double durationB = 2.4;
            StepScore scoreB = new StepScore(durationB);
            double[] stateB = {wheel.rpm(), 0}; // [endRpm, maxOvershoot]
            double[] quietB = {0, 0}; // [time, errSum]
            double[] tailB = {0, 0};  // [time, errSum]
            boolean[] unsafeB = {false};
            runFor(durationB, (t, dt) -> {
                wheel.sample();
                if (wheel.faulted()) {
                    flagsA[0] = true;
                    return false;
                }
                wheel.requestRpm(highRpm);
                double rpm = wheel.rpm();
                stateB[0] = rpm;
                stateB[1] = Math.max(stateB[1], rpm - highRpm);
                if (stateB[1] > Math.max(380.0, 0.90 * effectiveDrop)) {
                    unsafeB[0] = true;
                    return false;
                }
                double progress = (rpm - lowestRpm[0]) / dropDepth;
                boolean within = Math.abs(rpm - highRpm) <= Math.max(65.0, 0.025 * highRpm);
                if (within) {
                    quietB[0] += dt;
                    quietB[1] += (rpm - highRpm) * dt;
                } else {
                    quietB[0] = quietB[1] = 0;
                }
                if (t >= durationB * 0.70) {
                    tailB[0] += dt;
                    tailB[1] += (rpm - highRpm) * dt;
                }
                scoreB.sample(t, dt, progress, wheel.deliveredVolts() / BatteryVoltage.NOMINAL_VOLTS, within);
                show(mode + " · 回速恢复", "目标 %.0f / 当前 %.0f RPM / %.2f V", highRpm, rpm, wheel.deliveredVolts());
                return !scoreB.finishIfSettled(settleHold);
            });
            if (flagsA[0] || wheel.faulted()) return BayesianPidSearch.Result.unavailable("飞轮读数中断");
            if (!active()) return null;
            if (unsafeB[0]) {
                return new BayesianPidSearch.Result(9.0, -0.25, false,
                        String.format(Locale.US, "回速超调 +%.0f RPM", stateB[1]));
            }

            boolean arrivedA = Double.isFinite(scoreA.settlingTime(evalHold));
            boolean arrivedB = Double.isFinite(scoreB.settlingTime(evalHold));
            double allowedOverA = Math.max(130.0, 0.45 * finalDeltaRpm);
            double allowedOverB = Math.max(160.0, 0.55 * effectiveDrop);
            double overshootRatioA = stateA[2] / finalDeltaRpm;
            double overshootRatioB = stateB[1] / effectiveDrop;
            double marginA = 0.45 * (allowedOverA - stateA[2]) / allowedOverA;
            double marginB = 0.45 * (allowedOverB - stateB[1]) / allowedOverB;
            double margin = Math.min(marginA, marginB);
            double steadyErrA = quietA[0] >= 0.10 ? Math.abs(quietA[1] / quietA[0])
                    : tailA[0] >= 0.10 ? Math.abs(tailA[1] / tailA[0]) : Math.abs(stateA[1] - highRpm);
            double steadyErrB = quietB[0] >= 0.10 ? Math.abs(quietB[1] / quietB[0])
                    : tailB[0] >= 0.10 ? Math.abs(tailB[1] / tailB[0]) : Math.abs(stateB[0] - highRpm);
            double steadyRpmErr = Math.max(steadyErrA, steadyErrB);
            double steadyPenalty = 2.0 * (steadyRpmErr / Math.max(100.0, 0.03 * highRpm));
            double rawCostA = arrivedA ? scoreA.scoreWithSettlingTime(evalHold) : 2.6 + scoreA.score();
            double rawCostB = arrivedB ? scoreB.scoreWithSettlingTime(evalHold) : 2.6 + scoreB.score();
            double costA = Double.isFinite(rawCostA) ? rawCostA : 9.0;
            double costB = Double.isFinite(rawCostB) ? rawCostB : 9.0;
            double cost = Math.min(9.0, 0.50 * (costA + costB) + steadyPenalty);
            boolean settled = arrivedA && arrivedB;
            String reason = "";
            if (stateA[2] > allowedOverA) {
                reason = String.format(Locale.US, "阶跃超调 +%.0f RPM (%.0f%%)", stateA[2], overshootRatioA * 100);
            } else if (!arrivedA) {
                reason = String.format(Locale.US, "阶跃未停稳 (差 %+.0f RPM)", stateA[1] - highRpm);
            } else if (stateB[1] > allowedOverB) {
                reason = String.format(Locale.US, "回速超调 +%.0f RPM (%.0f%%)", stateB[1], overshootRatioB * 100);
            } else if (!arrivedB) {
                reason = String.format(Locale.US, "回速未停稳 (差 %+.0f RPM)", stateB[0] - highRpm);
            }
            telemetry.log().add("飞轮第 %d 轮 %.3f，稳态误差 %.0f RPM，裕量 %+.2f%s",
                    pidTrials, cost, steadyRpmErr, margin, settled && margin >= 0 ? "，停稳" : "");
            RobotLog.ii("BioBuzzCalibration",
                    "Flywheel trial %d (%s): P=%.3f I=%.3f D=%.3f base=%.0f high=%.0f cost=%.3f margin=%+.3f steady=%.0f RPM settled=%b fail=%s",
                    pidTrials, verification ? "verify" : "search", gains[0], gains[1], gains[2],
                    startBaseRpm, highRpm, cost, margin, steadyRpmErr, settled, reason);
            return new BayesianPidSearch.Result(cost, margin, settled, reason, steadyRpmErr);
        } finally {
            wheel.setRecoveryBoostEnabled(true);
            wheel.stop();
        }
    }

    private void testRecovery(Flywheel wheel) throws InterruptedException {
        Intake intake = new Intake(hardwareMap);
        double target = Math.min(RobotConfig.FLYWHEEL_TARGET_RPM, 0.96 * wheel.maxTargetRpmAtBattery());
        double[] drop = {Double.NaN, 0};
        boolean[] feeding = {false};
        try {
            runFor(15, (time, dt) -> {
                wheel.sample();
                if (wheel.faulted()) return false;
                wheel.requestRpm(target);
                double rpm = wheel.rpm();
                boolean held = gamepad1.right_trigger > 0.10;
                boolean ready = Math.abs(rpm - target) <= 0.05 * target;
                show("三球回速测试，停稳后按住 RT，B 停止", "%.0f / %.0f RPM，已恢复 %d 球", rpm, target, recoveries.size());
                if (feeding[0] && !held) return false;
                if (!feeding[0] && (!held || !ready || !intake.readyToFeed())) return true;
                if (!wheel.feed(true)) return false;
                intake.set(false, true, false);
                feeding[0] = true;
                if (!Double.isFinite(drop[0]) && rpm < 0.95 * target) {
                    drop[0] = time;
                    drop[1] = rpm;
                } else if (Double.isFinite(drop[0])) {
                    drop[1] = Math.min(drop[1], rpm);
                    if (ready || rpm > target) {
                        recordRecovery(drop[1], 1000 * (time - drop[0]));
                        drop[0] = Double.NaN;
                    }
                }
                return recoveries.size() < 3;
            });
            if (Double.isFinite(drop[0])) recordRecovery(drop[1], -1);
            if (recoveries.isEmpty()) telemetry.log().add("未检测到超过 5% 的掉速事件。");
        } finally {
            wheel.stop();
            intake.stop();
        }
    }

    private void recordRecovery(double minimum, double millis) {
        String value = String.format(Locale.ROOT, "第 %d 次掉速：最低 %.0f RPM，%s",
                recoveries.size() + 1, minimum, millis < 0 ? "结束时未恢复" : String.format(Locale.ROOT, "%.0f ms 恢复", millis));
        recoveries.add(value);
        telemetry.log().add("%s", value);
    }
}
