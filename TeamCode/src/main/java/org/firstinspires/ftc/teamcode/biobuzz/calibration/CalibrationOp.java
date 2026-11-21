package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.biobuzz.BatteryVoltage;
import org.firstinspires.ftc.teamcode.biobuzz.CalibrationStore;
import org.firstinspires.ftc.teamcode.biobuzz.ControlLoop;
import org.firstinspires.ftc.teamcode.biobuzz.Maths;
import org.firstinspires.ftc.teamcode.biobuzz.PinpointPose;
import org.firstinspires.ftc.teamcode.biobuzz.RearStatusLed;

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
            lights.stop();
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
            if (gamepad1.b) return false;
            if (gamepad1.a) {
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
        return confirm("请选择 PID。A 保存最佳新值，B 保留旧值。代价越低越好。", () -> {
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
        else telemetry.addData(title + " 复测", "%d/%d 组通过，%s",
                row.passed, BayesianPidSearch.VERIFICATION_ROUNDS, row.firstFailure);
    }

    interface Tick {
        boolean step(double seconds, double dt);
    }

    boolean runFor(double duration, Tick task) throws InterruptedException {
        if (opModeIsActive() && !motionAborted()) lights.show(RearStatusLed.State.CAL_RUNNING);
        try (ControlLoop loop = new ControlLoop(hardwareMap)) {
            double start = Maths.time(), previous = start - ControlLoop.PERIOD;
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
}
