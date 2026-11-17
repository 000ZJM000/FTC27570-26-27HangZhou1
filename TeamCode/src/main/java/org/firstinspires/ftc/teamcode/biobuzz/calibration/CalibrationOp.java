package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.biobuzz.CalibrationStore;
import org.firstinspires.ftc.teamcode.biobuzz.Maths;
import org.firstinspires.ftc.teamcode.biobuzz.RearStatusLed;

abstract class CalibrationOp extends LinearOpMode {
    CalibrationStore store;
    private boolean motionAborted;
    private RearStatusLed lights;

    @Override public final void runOpMode() throws InterruptedException {
        store = new CalibrationStore(hardwareMap.appContext);
        lights = RearStatusLed.open(hardwareMap);
        try {
            lights.show(RearStatusLed.State.INIT);
            telemetry.addLine(intro());
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

    interface Tick {
        boolean step(double seconds, double dt);
    }

    boolean runFor(double duration, Tick task) throws InterruptedException {
        if (opModeIsActive() && !motionAborted()) lights.show(RearStatusLed.State.CAL_RUNNING);
        double start = Maths.time(), previous = start - 0.02;
        while (opModeIsActive() && !motionAborted()) {
            double now = Maths.time(), elapsed = now - start;
            if (elapsed >= duration) return true;
            if (!task.step(elapsed, Maths.clip(now - previous, 0.005, 0.25))) return false;
            previous = now;
            sleep(20);
        }
        return false;
    }

    boolean motionAborted() {
        motionAborted |= gamepad1.b;
        return motionAborted;
    }

    boolean save(String key, double... values) {
        boolean saved = store.save(key, values);
        telemetry.log().add("%s %s", key, saved ? "已保存" : "保存失败");
        return saved;
    }

    void show(String task, String format, Object... values) {
        telemetry.addLine(task);
        telemetry.addData("测量", format, values);
        telemetry.addLine("B 停止当前动作");
        telemetry.update();
    }
}
