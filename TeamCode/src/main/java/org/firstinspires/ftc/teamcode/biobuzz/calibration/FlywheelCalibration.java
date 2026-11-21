package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import org.firstinspires.ftc.teamcode.biobuzz.BatteryVoltage;
import org.firstinspires.ftc.teamcode.biobuzz.CalibrationStore;
import org.firstinspires.ftc.teamcode.biobuzz.Flywheel;
import org.firstinspires.ftc.teamcode.biobuzz.Intake;
import org.firstinspires.ftc.teamcode.biobuzz.LeastSquares;
import org.firstinspires.ftc.teamcode.biobuzz.RobotConfig;

import java.util.ArrayList;
import java.util.List;
import java.util.function.IntSupplier;

@TeleOp(name = "04 飞轮校准 12V前馈·三球回速", group = "BIOBUZZ 自测")
public final class FlywheelCalibration extends CalibrationOp {
    private static final double[] VOLTAGE_LEVELS = {3.0, 4.5, 6.0, 7.5, 9.0, 10.5, 12.0};
    private final List<String> recoveries = new ArrayList<>();
    private double curveLimit;
    private String modelSummary = "沿用已有曲线";

    @Override String intro() {
        return "完成方向、每转计数、7档12V前馈曲线拟合，以及可选三球实弹回速测试。";
    }

    @Override void calibrate() throws InterruptedException {
        Flywheel wheel = new Flywheel(hardwareMap, store, battery);
        try {
            if (confirm("A 从手转识别开始全套校准；B 沿用已有方向和每转计数。")) {
                if (!calibrateDirection(wheel)) return;
                wheel = new Flywheel(hardwareMap, store, battery);
                if (wheel.ticksPerRev <= 0 || confirm("A 重测每转计数，B 使用当前 RPM 比例。")) {
                    if (!calibrateScale(wheel)) return;
                }
                wheel = new Flywheel(hardwareMap, store, battery);
            }
            if (confirm("确认供弹通道为空、手已移开。A 开始 7 档电压扫描（3V–12V）拟合前馈曲线；B 跳过直接进三球回速测试。")) {
                if (!fitModel(wheel, battery)) return;
                wheel = new Flywheel(hardwareMap, store, battery);
            }
            if (!wheel.rpmCalibrated()) {
                telemetry.log().add("缺少 RPM 比例或电压曲线。");
                return;
            }
            curveLimit = wheel.maxTargetRpm();
            if (wheel.hasBlocker() && confirm("可装入最多三颗球，移开手并朝向得分区。A 开始三球回速实测，B 结束。")) {
                testRecovery(wheel);
            }
        } finally {
            wheel.stop();
        }
    }

    @Override void showResults() {
        telemetry.addData("飞轮前馈", modelSummary);
        if (curveLimit > 0) telemetry.addData("目标上限", "%.0f RPM", curveLimit);
        for (String recovery : recoveries) telemetry.addLine(recovery);
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

    private boolean fitModel(Flywheel wheel, BatteryVoltage battery) throws InterruptedException {
        List<double[]> points = new ArrayList<>();
        try {
            for (double volts : VOLTAGE_LEVELS) {
                if (!opModeIsActive() || motionAborted() || wheel.faulted()) return false;
                double[] row = measurePlateau(wheel, battery, volts);
                if (row != null && (points.isEmpty() || row[1] - points.get(points.size() - 1)[1] > 0.1)) {
                    points.add(row);
                    telemetry.log().add("%.2f V → %.0f ticks/s", row[1], row[0]);
                }
            }
        } finally {
            wheel.stop();
        }
        if (!opModeIsActive() || motionAborted() || wheel.faulted()) return false;
        if (points.size() < 6 || points.get(points.size() - 1)[1] < 9) {
            telemetry.log().add("有效稳态点不足。检查编码器及电压后重测，保留旧曲线。");
            return false;
        }
        double ref = points.get(points.size() - 1)[0];
        if (points.get(0)[0] > 0.88 * ref) {
            telemetry.log().add("最低与最高档的稳态转速过于接近，保留旧前馈曲线。");
            return false;
        }
        double[] curve = fitCurve(points, ref, true);
        if (!Flywheel.validModel(curve)) curve = fitCurve(points, ref, false);
        if (!Flywheel.validModel(curve)) {
            telemetry.log().add("曲线拟合无效，保留旧曲线。");
            return false;
        }
        if (!save(CalibrationStore.FLYWHEEL_VOLTAGE_MODEL, curve)) return false;
        modelSummary = String.format(java.util.Locale.ROOT,
                "已保存 kS=%.2f V, kV=%.2f V, kQ=%.2f V", curve[1], curve[2], curve[3]);
        return true;
    }

    private static double[] fitCurve(List<double[]> points, double ref, boolean quadratic) {
        LeastSquares fit = new LeastSquares(quadratic ? new double[]{1e-6, 1e-6, 0.001}
                : new double[]{1e-6, 1e-6}, null);
        for (double[] point : points) {
            double rate = point[0] / ref;
            fit.add(quadratic ? new double[]{1, rate, rate * rate} : new double[]{1, rate}, point[1], 1);
        }
        double[] values = fit.solve();
        return values == null ? null : new double[]{ref, Math.max(0, values[0]), values[1], quadratic ? values[2] : 0};
    }

    private double[] measurePlateau(Flywheel wheel, BatteryVoltage battery, double volts) throws InterruptedException {
        Plateau plateau = new Plateau();
        runFor(5.5, (t, dt) -> {
            wheel.sample();
            if (wheel.faulted()) return false;
            double delivered = wheel.deliveredVolts();
            double target = Math.min(volts, battery.volts());
            if (t >= 1.8 && volts == VOLTAGE_LEVELS[0] && wheel.speed <= 10) return false;
            if (t > 0.7 && wheel.speed > 10 && Math.abs(delivered - target) < 0.12) {
                plateau.sample(wheel.speed, delivered, dt);
            } else plateau.reset();
            wheel.powerVolts(volts, dt);
            show("飞轮空载曲线", "%.1f V / 实际 %.2f V / %.0f ticks/s", volts, delivered, wheel.speed);
            return !plateau.ready();
        });
        return plateau.ready() ? new double[]{plateau.speedIntegral / plateau.time, plateau.voltsIntegral / plateau.time}
                : null;
    }

    private static final class Plateau {
        double time, speedIntegral, voltsIntegral;
        double minimum = Double.POSITIVE_INFINITY, maximum = Double.NEGATIVE_INFINITY;

        void sample(double speed, double volts, double dt) {
            double low = Math.min(minimum, speed), high = Math.max(maximum, speed);
            if (high - low > Math.max(3, 0.03 * high)) reset();
            minimum = Math.min(minimum, speed);
            maximum = Math.max(maximum, speed);
            time += dt;
            speedIntegral += speed * dt;
            voltsIntegral += volts * dt;
        }

        void reset() {
            time = speedIntegral = voltsIntegral = 0;
            minimum = Double.POSITIVE_INFINITY;
            maximum = Double.NEGATIVE_INFINITY;
        }

        boolean ready() { return time >= 0.55; }
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
        String value = String.format(java.util.Locale.ROOT, "第 %d 次掉速：最低 %.0f RPM，%s",
                recoveries.size() + 1, minimum, millis < 0 ? "结束时未恢复" : String.format(java.util.Locale.ROOT, "%.0f ms 恢复", millis));
        recoveries.add(value);
        telemetry.log().add("%s", value);
    }
}
