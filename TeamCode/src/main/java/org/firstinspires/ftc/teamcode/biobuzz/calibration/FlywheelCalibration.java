package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import org.firstinspires.ftc.teamcode.biobuzz.CalibrationStore;
import org.firstinspires.ftc.teamcode.biobuzz.Flywheel;
import org.firstinspires.ftc.teamcode.biobuzz.Maths;
import org.firstinspires.ftc.teamcode.biobuzz.RobotConfig;

import java.util.Arrays;
import java.util.Locale;
import java.util.function.IntSupplier;

@TeleOp(name = "04 自测飞轮 转速和 PID", group = "BIOBUZZ 自测")
public final class FlywheelCalibration extends CalibrationOp {
    private static final double PID_HOLD = 0.25;
    private static final double MIN_STEP_DURATION = 4.5;
    private static final int RANGE_POINTS = 7;
    private static final double REFERENCE_POWER = 0.70;
    private static final double FULL_POWER = RobotConfig.FLYWHEEL_MAX_POWER;
    private static final double[] POWER_LEVELS = {
            0.10, 0.14, 0.18, 0.22, 0.26, 0.32, 0.38, 0.44, 0.50, 0.56, 0.63,
            REFERENCE_POWER, 0.77, 0.84, 0.90, RobotConfig.FLYWHEEL_TARGET_MAX_DUTY, FULL_POWER
    };
    private int trials;
    private BayesianPidSearch completed;
    private double fullPowerSpeed, fullPowerRpm, peakPidPower;
    private String outcome;

    @Override String intro() {
        return "清空仓内和进弹通道，移开飞轮周围物品。自测中 SB 保持关闭。";
    }

    @Override void calibrate() throws InterruptedException {
        Flywheel wheel = new Flywheel(hardwareMap, store);
        try {
            Flywheel currentWheel = wheel;
            if (confirm("先识别出球方向。A 手转识别，B 沿用当前转向。", () ->
                    telemetry.addData("m-7 / m-8 当前转向", "%s / %s",
                            currentWheel.leftDirection(), currentWheel.encoderDirection()))) {
                if (!calibrateDirection(wheel)) return;
                wheel = new Flywheel(hardwareMap, store);
            }
            if (!opModeIsActive()) return;
            if (confirm(wheel.ticksPerRev > 0
                    ? "A 重测每转计数，B 沿用已有 RPM 比例。"
                    : "标定实际 RPM？在飞轮上标记起点。A 进入手转测量，B 用 ticks/s。")) {
                if (!calibrateScale(wheel)) return;
                wheel = new Flywheel(hardwareMap, store);
            }
            String prompt = wheel.fullPowerSpeed() > 0
                    ? "A 重测 0.10 到 1.00 的 17 点功率曲线，B 沿用已保存曲线。更换结构后选 A。"
                    : "确认无球后按 A 测 0.10 到 1.00 的 17 点功率曲线。";
            if (confirm(prompt)) {
                while (!fitModel(wheel)) {
                    if (!opModeIsActive() || motionAborted() || !confirm("曲线未测成。A 重测，B 结束。")) return;
                    wheel = new Flywheel(hardwareMap, store);
                }
                wheel = new Flywheel(hardwareMap, store);
            }
            if (wheel.fullPowerSpeed() <= 0) {
                telemetry.log().add("缺少 1.00 稳态转速。检查 m-8 编码器和电池后重跑 04。");
                return;
            }
            fullPowerSpeed = wheel.fullPowerSpeed();
            fullPowerRpm = wheel.ticksPerRev > 0 ? 60 * fullPowerSpeed / wheel.ticksPerRev : 0;
            double[] speeds = tuningSpeeds(wheel);
            if (speeds.length == 0) {
                telemetry.log().add("可稳速范围过窄，无法形成有效 PID 阶跃。");
                return;
            }
            double duration = Maths.clip(1.65 * wheel.longestMeasuredTransition() + 1.40, MIN_STEP_DURATION, 10.0);
            if (!confirm("确认无球。A 开始 20–32 轮 PID 搜索和留出复测，B 跳过。",
                    () -> showCoverage(speeds, duration))) return;
            Flywheel tested = wheel;
            double[] current = wheel.pidGains();
            BayesianPidSearch search = new BayesianPidSearch(
                    new double[]{Math.max(1.1, current[0] * 1.5), Math.max(1.5, current[1] * 1.5),
                            Math.max(0.080, current[2] * 1.5)},
                    new double[]{0.40, 0.26, 0.012});
            search.tune(current,
                    (gains, verification, pattern) -> pidTrial(tested, gains, speeds, duration, verification, pattern),
                    () -> !opModeIsActive() || motionAborted() || tested.faulted(),
                    store.readHistory(CalibrationStore.FLYWHEEL_PID_HISTORY, BayesianPidSearch.HISTORY_STRIDE,
                            BayesianPidSearch.HISTORY_ROWS));
            completed = search;
            if (active(tested) && search.searchCount() > 0) {
                save(CalibrationStore.FLYWHEEL_PID_HISTORY, search.history());
            }
            BayesianPidSearch.Review best = search.bestCandidate();
            if (best != null && active(tested)) {
                tested.stop();
                if (!choosePid(search, best, () -> showCoverage(speeds, duration))) {
                    outcome = "保留旧 PID";
                    return;
                }
                outcome = active(tested) && save(CalibrationStore.FLYWHEEL_PID, best.gains)
                        ? "已保存新 PID" : "新 PID 未写入";
                telemetry.log().add("%s，代价 %.3f，P/I/D %.3f / %.3f / %.3f",
                        outcome, best.score, best.gains[0], best.gains[1], best.gains[2]);
            } else if (opModeIsActive() && !motionAborted()) {
                outcome = tested.faulted() ? "飞轮编码器异常，保留原 PID"
                        : search.searchCount() < BayesianPidSearch.MIN_ROUNDS ? "有效搜索轮数不足，保留原 PID"
                        : "没有新参数全部通过复测，保留原 PID";
                telemetry.log().add("%s", outcome);
            }
        } finally {
            wheel.stop();
        }
    }

    @Override void showResults() {
        if (fullPowerSpeed > 0) telemetry.addData("1.00 稳态", speedText(fullPowerSpeed, fullPowerRpm));
        if (completed == null) return;
        telemetry.addData("结果", outcome == null ? "中止" : outcome);
        telemetry.addData("搜索", "历史 %d / 本轮 %d / 最低代价 %.3f",
                completed.historyCount(), completed.searchCount(), completed.bestSearchCost());
        telemetry.addData("PID 实测峰值功率", "%.2f", peakPidPower);
        showPidReview("旧 PID", completed.incumbent());
        showPidReview("最佳新 PID", completed.bestCandidate());
    }

    private static String speedText(double ticks, double rpm) {
        return rpm > 0 ? String.format(Locale.US, "%.0f ticks/s (%.0f RPM)", ticks, rpm)
                : String.format(Locale.US, "%.0f ticks/s", ticks);
    }

    private double testedRpm(double speed) { return speed * fullPowerRpm / fullPowerSpeed; }

    private void showCoverage(double[] speeds, double duration) {
        double low = speeds[0], high = speeds[speeds.length - 1];
        telemetry.addData("PID 复测范围", "%s 到 %s，%d 点", speedText(low, testedRpm(low)),
                speedText(high, testedRpm(high)), speeds.length);
        telemetry.addData("每个阶跃最长", "%.1f 秒", duration);
    }

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

    private boolean fitModel(Flywheel wheel) throws InterruptedException {
        Measurements.PowerFit fit = new Measurements.PowerFit();
        Measurements referenceSpeeds = new Measurements(), fullSpeeds = new Measurements();
        double minimumSpeed = Double.POSITIVE_INFINITY, longestTransition = 0;
        int measuredPoints = 0;
        try {
            for (double power : POWER_LEVELS) {
                if (!active(wheel)) break;
                Plateau plateau = measurePlateau(wheel, power, power <= 0.22 ? 3.5 : 7.0);
                if (!Double.isFinite(plateau.speed)) continue;
                measuredPoints++;
                minimumSpeed = Math.min(minimumSpeed, plateau.speed);
                fit.add(plateau.speed, power);
                if (power == REFERENCE_POWER) referenceSpeeds.add(plateau.speed);
                if (power == FULL_POWER) fullSpeeds.add(plateau.speed);
            }
            for (int repeat = 0; repeat < 2 && active(wheel); repeat++) {
                if (fullSpeeds.size() >= 2 && fullSpeeds.relativeSpread() <= 0.04) break;
                // Recheck the high end after a separate lower-duty plateau.
                Plateau reference = measurePlateau(wheel, REFERENCE_POWER, 8.0);
                referenceSpeeds.add(reference.speed);
                fit.add(reference.speed, REFERENCE_POWER);
                longestTransition = Math.max(longestTransition, reference.seconds);
                if (!active(wheel)) break;
                Plateau full = measurePlateau(wheel, FULL_POWER, 8.0);
                fit.add(full.speed, FULL_POWER);
                fullSpeeds.add(full.speed);
                longestTransition = Math.max(longestTransition, full.seconds);
            }
        } finally {
            wheel.stop();
        }
        if (!active(wheel)) return false;
        double full = fullSpeeds.median();
        double[] coefficients = fit.fit(Math.max(8, full * 0.07));
        if (measuredPoints < 6 || fullSpeeds.size() < 2 || fullSpeeds.relativeSpread() > 0.05 || !(full >= 40)
                || coefficients == null || coefficients[0] > 0.50) {
            telemetry.log().add("曲线未通过，稳态功率点 %d/%d，满功率样本 %d 组。",
                    measuredPoints, POWER_LEVELS.length, fullSpeeds.size());
            return false;
        }
        double reference = referenceSpeeds.size() > 0 ? referenceSpeeds.median()
                : (REFERENCE_POWER - coefficients[0]) / coefficients[1];
        if (!(reference >= 20 && reference < full)) return false;
        double slope = (RobotConfig.FLYWHEEL_MAX_POWER - coefficients[0]) * reference / full;
        telemetry.log().add("功率曲线有效 %d/%d 点，1.00 稳态 %.0f ticks/s，共 %d 组。",
                measuredPoints, POWER_LEVELS.length, full, fullSpeeds.size());
        return save(CalibrationStore.FLYWHEEL_FULL_POWER_MODEL, reference, coefficients[0], slope, full,
                minimumSpeed, longestTransition);
    }

    private Plateau measurePlateau(Flywheel wheel, double power, double timeout) throws InterruptedException {
        Plateau plateau = new Plateau();
        runFor(timeout, (t, dt) -> {
            wheel.sample();
            if (wheel.faulted()) return false;
            // Use samples only after both motors have held the requested output.
            if (Math.abs(wheel.output - power) < 1e-6) plateau.sample(wheel.speed, dt);
            wheel.power(power, dt);
            show("飞轮功率曲线", "设定 %.2f / 实际 %.2f / %s",
                    power, wheel.output, speedText(wheel.speed,
                            wheel.ticksPerRev > 0 ? 60 * wheel.speed / wheel.ticksPerRev : 0));
            if (Double.isFinite(plateau.speed)) plateau.seconds = t;
            return !Double.isFinite(plateau.speed);
        });
        if (Double.isFinite(plateau.speed)) telemetry.log().add("功率 %.2f，稳态 %.0f ticks/s", power, plateau.speed);
        return plateau;
    }

    private static final class Plateau {
        private double warmup, window, total, previous = Double.NaN;
        double speed = Double.NaN, seconds;

        void sample(double velocity, double dt) {
            warmup += dt;
            if (warmup < 0.65) return;
            window += dt;
            total += velocity * dt;
            if (window < 0.32) return;
            double mean = total / window;
            if (mean >= 20 && Math.abs(mean - previous) <= 0.022 * mean) speed = (mean + previous) / 2;
            previous = mean;
            window = total = 0;
        }
    }

    private double[] tuningSpeeds(Flywheel wheel) {
        double maximum = wheel.estimatedSpeedAtPower(RobotConfig.FLYWHEEL_TARGET_MAX_DUTY);
        double measuredMinimum = wheel.lowestMeasuredSpeed();
        double minimum = Math.max(0.18 * maximum, 1.05 * measuredMinimum);
        double operating = wheel.rpmCalibrated()
                ? Math.min(RobotConfig.FLYWHEEL_TARGET_RPM * wheel.ticksPerRev / 60, maximum) : Double.NaN;
        if (Double.isFinite(operating) && operating < minimum) {
            if (operating >= measuredMinimum && operating > 0) minimum = operating;
            else telemetry.log().add("%.0f RPM 低于已测稳定转速，本轮从已测范围开始。",
                    RobotConfig.FLYWHEEL_TARGET_RPM);
        }
        return rangeSpeeds(minimum, maximum, operating);
    }

    static double[] rangeSpeeds(double minimum, double maximum, double operating) {
        if (!Maths.finite(minimum, maximum) || minimum <= 0 || maximum - minimum < 0.18 * maximum) {
            return new double[0];
        }
        boolean includeOperating = operating > minimum && operating < maximum;
        double[] speeds = new double[RANGE_POINTS + (includeOperating ? 1 : 0)];
        for (int i = 0; i < RANGE_POINTS; i++) {
            speeds[i] = minimum + (maximum - minimum) * i / (RANGE_POINTS - 1);
            if (includeOperating && Math.abs(speeds[i] - operating) <= 1e-6 * maximum) {
                speeds[i] = operating;
                includeOperating = false;
            }
        }
        if (!includeOperating) return Arrays.copyOf(speeds, RANGE_POINTS);
        speeds[RANGE_POINTS] = operating;
        Arrays.sort(speeds);
        return speeds;
    }

    static double[] verificationTargets(double[] speeds, int pattern) {
        double[] targets = new double[speeds.length];
        for (int i = 0; i < targets.length; i++) {
            // A stride of three visits every point in either a seven- or eight-point grid.
            int index = (3 + 3 * i) % speeds.length;
            targets[i] = speeds[pattern == 0 ? index : speeds.length - 1 - index];
        }
        return targets;
    }

    private BayesianPidSearch.Result pidTrial(Flywheel wheel, double[] gains, double[] speeds, double duration,
                                               boolean verification, int pattern) throws InterruptedException {
        int last = speeds.length - 1;
        double start = speeds[verification ? (pattern == 0 ? 0 : last) : last / 4];
        double[] targets = verification ? verificationTargets(speeds, pattern)
                : new double[]{speeds[3 * last / 4], speeds[0], speeds[last], speeds[last / 2]};
        if (!active(wheel)) return null;
        wheel.setGains(Flywheel.DEFAULT_GAINS);
        trials++;
        try {
            prepareSpeed(wheel, start, duration, false);
            if (active(wheel) && Math.abs(wheel.speed - start) > 0.09 * start) {
                prepareSpeed(wheel, start, duration, true);
            }
            if (!active(wheel)) return null;
            if (Math.abs(wheel.speed - start) > 0.12 * start) {
                return BayesianPidSearch.Result.unavailable("起始转速未到位 " + speedText(start, testedRpm(start)));
            }
            return measureSteps(wheel, gains, targets, duration, verification);
        } finally {
            wheel.stop();
        }
    }

    private void prepareSpeed(Flywheel wheel, double target, double duration, boolean feedforward)
            throws InterruptedException {
        runFor(feedforward ? 0.65 * duration : duration, (t, dt) -> {
            wheel.sample();
            if (wheel.faulted()) return false;
            if (feedforward) wheel.power(wheel.estimatedPowerForSpeed(target), dt);
            else wheel.regulate(target);
            show("飞轮准备起始转速", "%s / %s", speedText(wheel.speed, testedRpm(wheel.speed)),
                    speedText(target, testedRpm(target)));
            return t < 0.22 || Math.abs(wheel.speed - target) > target * 0.045;
        });
    }

    private BayesianPidSearch.Result measureSteps(Flywheel wheel, double[] gains, double[] targets, double duration,
                                                   boolean verification) throws InterruptedException {
        wheel.setGains(gains);
        double total = 0, worst = 0, margin = 0.11, overspeed = fullPowerSpeed * 1.08;
        boolean settled = true;
        String firstFailure = "";
        for (int index = 0; index < targets.length; index++) {
            double target = targets[index];
            int stepNumber = index + 1;
            double initial = wheel.speed;
            double step = target - initial;
            if (Math.abs(step) < target * 0.04) {
                total += 4;
                worst = Math.max(worst, 4);
                settled = false;
                if (firstFailure.isEmpty()) firstFailure = "未形成有效转速阶跃";
                continue;
            }
            StepScore response = new StepScore(duration);
            boolean[] runaway = {false};
            runFor(duration, (t, dt) -> {
                wheel.sample();
                if (wheel.faulted()) return false;
                if (wheel.speed > overspeed) {
                    runaway[0] = true;
                    return false;
                }
                wheel.regulate(target);
                peakPidPower = Math.max(peakPidPower, wheel.output);
                response.sample(t, dt, (wheel.speed - initial) / step, wheel.output, Math.abs(wheel.speed - target)
                        <= target * RobotConfig.FLYWHEEL_SPEED_TOLERANCE);
                show((verification ? "飞轮 PID 复测 " : "飞轮 PID 搜索 ") + trials,
                        "测点 %d/%d，%s / %s，功率 %.2f，峰值 %.2f", stepNumber, targets.length,
                        speedText(wheel.speed, testedRpm(wheel.speed)), speedText(target, testedRpm(target)),
                        wheel.output, peakPidPower);
                return verification || !response.finishIfSettled(0.55);
            });
            if (!active(wheel)) return null;
            if (runaway[0]) return new BayesianPidSearch.Result(10, -0.6, false, "转速保护");
            boolean arrived = Double.isFinite(response.settlingTime(PID_HOLD));
            margin = Math.min(margin, 0.11 - response.overshoot() * Math.abs(step) / target);
            if (firstFailure.isEmpty() && (!arrived || margin < 0)) {
                firstFailure = speedText(target, testedRpm(target)) + (margin < 0 ? " 超调超限"
                        : String.format(Locale.US, " 未稳速，末端 %+.1f%%", 100 * (wheel.speed / target - 1)));
            }
            double cost = Math.min(9, arrived ? response.scoreWithSettlingTime(PID_HOLD) : 2.6 + response.score());
            total += cost;
            worst = Math.max(worst, cost);
            settled &= arrived;
        }
        double score = 0.70 * total / targets.length + 0.30 * worst;
        telemetry.log().add("飞轮第 %d 轮 %.3f，超调裕量 %+.2f%s", trials, score, margin,
                settled && margin >= 0 ? "，稳速" : "");
        return new BayesianPidSearch.Result(score, margin, settled, firstFailure);
    }

    private boolean active(Flywheel wheel) { return opModeIsActive() && !motionAborted() && !wheel.faulted(); }
}
