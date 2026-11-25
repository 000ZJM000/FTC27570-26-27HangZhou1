package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.biobuzz.config.CalibrationStore;
import org.firstinspires.ftc.teamcode.biobuzz.core.BatteryVoltage;
import org.firstinspires.ftc.teamcode.biobuzz.core.ControlLoop;
import org.firstinspires.ftc.teamcode.biobuzz.core.Maths;
import org.firstinspires.ftc.teamcode.biobuzz.localization.PinpointPose;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.RearStatusLed;

import java.util.ArrayList;
import java.util.List;

/** Base LinearOpMode for self-tests, bundling loop control, measurements, and step scoring. */
abstract class CalibrationOp extends LinearOpMode {
    CalibrationStore store;
    BatteryVoltage battery;
    private boolean motionAborted;
    double motionDeadline = Double.POSITIVE_INFINITY;
    private double nextDisplay;
    private RearStatusLed lights;

    @Override public final void runOpMode() throws InterruptedException {
        store = new CalibrationStore(hardwareMap.appContext);
        battery = new BatteryVoltage(hardwareMap);
        lights = RearStatusLed.open(hardwareMap);
        try {
            lights.show(RearStatusLed.State.INIT);
            telemetry.addLine(intro());
            showVoltage();
            telemetry.addLine("START 进入自测。A 确认，B 跳过/停止当前动作。");
            if (!lights.configured()) telemetry.addData("尾灯", "未配置 LED");
            telemetry.update();
            waitForStart();
            if (!opModeIsActive()) return;
            boolean error = false;
            try {
                calibrate();
            } catch (RuntimeException failure) {
                error = true;
                telemetry.log().add("自测中断：%s", failure.getMessage());
            }
            if (opModeIsActive()) {
                lights.show(error || motionAborted ? RearStatusLed.State.FAULT : RearStatusLed.State.CAL_DONE);
            }
            while (opModeIsActive()) {
                telemetry.addLine("本轮结束。请查看日志，再按 STOP。");
                showVoltage();
                showResults();
                telemetry.update();
                sleep(100);
            }
        } finally {
            ControlLoop.safeStop("尾灯", lights::stop);
        }
    }

    abstract String intro();
    abstract void calibrate() throws InterruptedException;
    void showResults() {}

    boolean confirm(String instructions) throws InterruptedException {
        return confirm(instructions, () -> {});
    }

    boolean confirm(String instructions, Runnable display) throws InterruptedException {
        if (opModeIsActive()) lights.show(RearStatusLed.State.CAL_WAIT);
        while (opModeIsActive() && (gamepad1.a || gamepad1.b)) sleep(20);
        while (opModeIsActive()) {
            telemetry.addLine(instructions);
            telemetry.addLine("A 确认 / B 跳过");
            showVoltage();
            display.run();
            telemetry.update();
            if (gamepad1.b) {
                while (opModeIsActive() && gamepad1.b) sleep(20);
                return false;
            }
            if (gamepad1.a) {
                while (opModeIsActive() && gamepad1.a) sleep(20);
                motionAborted = false;
                lights.show(RearStatusLed.State.CAL_RUNNING);
                return true;
            }
            sleep(25);
        }
        return false;
    }

    boolean choosePid(BayesianPidSearch search, BayesianPidSearch.Review candidate, Runnable display)
            throws InterruptedException {
        return choosePid("请选择 PID。A 保存最佳新值，B 保留旧值。代价越低越好。", search, candidate, display);
    }

    boolean choosePid(String prompt, BayesianPidSearch search, BayesianPidSearch.Review candidate, Runnable display)
            throws InterruptedException {
        return confirm(prompt, () -> {
            showPidReview("旧 PID", search.incumbent());
            showPidReview("最佳新 PID", candidate);
            display.run();
        });
    }

    void showPidReview(String title, BayesianPidSearch.Review row) {
        if (row == null) return;
        telemetry.addData(title + " P / I / D", "%.3f / %.3f / %.3f", row.gains[0], row.gains[1], row.gains[2]);
        if (row.qualified()) telemetry.addData(title + " 复测", "%d/%d 组通过，代价 %.3f",
                row.passed, BayesianPidSearch.VERIFICATION_ROUNDS, row.score);
        else if (Double.isFinite(row.measuredScore)) telemetry.addData(title + " 复测", "%d/%d 组通过（均价 %.3f），%s",
                row.passed, BayesianPidSearch.VERIFICATION_ROUNDS, row.measuredScore, row.firstFailure);
        else telemetry.addData(title + " 复测", "%d/%d 组通过，%s",
                row.passed, BayesianPidSearch.VERIFICATION_ROUNDS, row.firstFailure);
    }

    interface Tick {
        boolean step(double seconds, double dt);
    }

    boolean runFor(double duration, Tick task) throws InterruptedException {
        if (opModeIsActive() && !motionAborted()) lights.show(RearStatusLed.State.CAL_RUNNING);
        try (ControlLoop loop = new ControlLoop(hardwareMap)) {
            double start = Maths.time(), previous = start - ControlLoop.CALIBRATION_PERIOD;
            while (opModeIsActive() && !motionAborted()) {
                double now = loop.beginFrame(), elapsed = now - start;
                if (elapsed >= duration) return true;
                battery.update(now);
                if (!battery.measured()) {
                    telemetry.log().add("电池电压读数缺失，停止本轮 12V 测量。");
                    motionAborted = true;
                    return false;
                }
                if (!task.step(elapsed, Maths.clip(now - previous, 0.005, 0.25))) return false;
                previous = now;
                long remaining = loop.remainingMillis(now);
                if (remaining > 0) sleep(remaining);
            }
        }
        return false;
    }

    boolean initializePinpoint(PinpointPose pose) throws InterruptedException {
        if (!pose.startImu()) return false;
        runFor(2.0, (time, dt) -> {
            pose.update();
            return !pose.headingValid;
        });
        return opModeIsActive() && !motionAborted() && pose.headingValid;
    }

    boolean motionAborted() {
        motionAborted |= gamepad1.b || Maths.time() >= motionDeadline;
        return motionAborted;
    }

    boolean motionWasAborted() { return motionAborted; }

    boolean save(String key, double... values) {
        boolean saved = store.save(key, values);
        telemetry.log().add("%s %s", key, saved ? "已保存" : "保存失败");
        return saved;
    }

    void show(String task, String format, Object... values) {
        double now = Maths.time();
        if (now < nextDisplay) return;
        nextDisplay = now + 0.10;
        telemetry.addLine(task);
        telemetry.addData("测量", format, values);
        showVoltage();
        telemetry.addLine("B 停止当前动作");
        telemetry.update();
    }

    private void showVoltage() {
        battery.update(Maths.time());
        if (battery.measured()) telemetry.addData("12V 电压补偿", "实测 %.2f V / 标称 12.00 V", battery.volts());
        else telemetry.addData("12V 电压补偿", "等待电池读数，标称 12.00 V");
    }

    /** Robust median / Theil-Sen helper for noisy calibration samples. */
    static final class Measurements {
        private final double[] values = new double[256];
        private int count;

        void add(double value) {
            if (Double.isFinite(value) && count < values.length) values[count++] = value;
        }

        double median() { return Maths.median(values, count); }
        int size() { return count; }

        double relativeSpread() {
            double middle = median();
            double[] differences = new double[count];
            for (int i = 0; i < count; i++) differences[i] = Math.abs(values[i] - middle);
            return Maths.median(differences, count) / Math.max(1e-9, Math.abs(middle));
        }

        static final class PowerFit {
            private final List<double[]> points = new ArrayList<>();

            void add(double speed, double power) {
                if (speed > 0 && Maths.finite(speed, power)) points.add(new double[]{speed, power});
            }

            double[] fit(double minimumSpeedDifference) {
                Measurements slopes = new Measurements();
                for (int i = 0; i < points.size(); i++) {
                    for (int j = i + 1; j < points.size(); j++) {
                        double[] a = points.get(i), b = points.get(j);
                        double dx = b[0] - a[0];
                        if (Math.abs(dx) > minimumSpeedDifference) slopes.add((b[1] - a[1]) / dx);
                    }
                }
                double slope = slopes.median();
                if (slopes.size() < 2 || !(slope > 0)) return null;
                Measurements intercepts = new Measurements();
                for (double[] point : points) intercepts.add(point[1] - slope * point[0]);
                return new double[]{Math.max(0, intercepts.median()), slope};
            }
        }
    }

    /** Normalized step response cost evaluator. */
    static final class StepScore {
        private final double duration;
        private double errorArea, overshoot, tailTime, tailError, tailSquare, effort, lastPower;
        private double lastSample, withinToleranceSince = Double.NaN;
        private double quietTime, quietError, quietSquare, quietAbsolute;
        private boolean endedEarly;

        StepScore(double duration) { this.duration = duration; }

        void sample(double time, double dt, double progress, double power, boolean withinTolerance) {
            double error = 1 - progress;
            errorArea += Math.abs(error) * dt;
            overshoot = Math.max(overshoot, -error);
            effort += Math.abs(power - lastPower);
            lastPower = power;
            if (time > duration * 0.65) {
                tailTime += dt;
                tailError += error * dt;
                tailSquare += error * error * dt;
            }
            lastSample = time;
            if (withinTolerance) {
                if (!Double.isFinite(withinToleranceSince)) withinToleranceSince = time;
                quietTime += dt;
                quietError += error * dt;
                quietSquare += error * error * dt;
                quietAbsolute += Math.abs(error) * dt;
            } else {
                withinToleranceSince = Double.NaN;
                quietTime = quietError = quietSquare = quietAbsolute = 0;
            }
        }

        boolean finishIfSettled(double hold) {
            endedEarly = Double.isFinite(settlingTime(hold));
            return endedEarly;
        }

        double settlingTime(double hold) {
            return lastSample - withinToleranceSince >= hold
                    ? withinToleranceSince + hold : Double.POSITIVE_INFINITY;
        }

        double scoreWithSettlingTime(double hold) {
            return 1.65 * settlingTime(hold) / duration + 0.50 * score();
        }

        double overshoot() { return overshoot; }

        double score() {
            if (endedEarly) {
                double area = errorArea + Math.max(0, duration - lastSample) * quietAbsolute / quietTime;
                return score(area, quietError / quietTime, quietSquare / quietTime);
            }
            if (tailTime < duration * 0.12) return Double.POSITIVE_INFINITY;
            return score(errorArea, tailError / tailTime, tailSquare / tailTime);
        }

        private double score(double area, double mean, double meanSquare) {
            double deviation = Math.sqrt(Math.max(0, meanSquare - mean * mean));
            return area / duration + 1.8 * Math.abs(mean) + 2 * deviation + 1.5 * overshoot + 0.07 * effort;
        }
    }
}
