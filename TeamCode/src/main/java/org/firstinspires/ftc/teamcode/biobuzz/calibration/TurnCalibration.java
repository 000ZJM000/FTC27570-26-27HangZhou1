package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.biobuzz.calibration.CalibrationOp.Measurements;
import org.firstinspires.ftc.teamcode.biobuzz.config.CalibrationStore;
import org.firstinspires.ftc.teamcode.biobuzz.core.Maths;
import org.firstinspires.ftc.teamcode.biobuzz.localization.PinpointPose;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Drive;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Drive.TurnLinearity;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Turret;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@TeleOp(name = "07 转向摇杆线性校准", group = "BIOBUZZ 自测")
public final class TurnCalibration extends CalibrationOp {
    private static final int SWEEP_POINTS = 7;
    private static final double[] VERIFY_STICKS = {0.20, 0.40, 0.60, 0.80, 1.00};
    private TurnLinearity model;
    private String outcome = "未运行", leftEquation = "—", rightEquation = "—", verifySummary = "—";
    private final List<String> verifyRows = new ArrayList<>();

    @Override String intro() {
        return "自动输出旋转虚拟杆量，辨识死区与非线性并生成校准函数，使右摇杆杆量与旋转速度线性对应。";
    }

    @Override void calibrate() throws InterruptedException {
        Turret turret = optionalTurret();
        if (turret != null) turret.teachingMode(false);
        Drive drive = new Drive(hardwareMap);
        PinpointPose pose = new PinpointPose(hardwareMap, battery);
        try {
            model = TurnLinearity.load(store);
            boolean hasExisting = model.calibrated();
            boolean remeasure = !hasExisting
                    ? confirm("轮子落地并留出旋转空间。A 初始化 Pinpoint 后自动输出虚拟杆量校准。")
                    : confirm("轮子落地并留出旋转空间。A 自动输出虚拟杆量重测并生成校准函数；B 直接复测现有函数。");
            if (!remeasure && !hasExisting) {
                outcome = "已取消";
                return;
            }
            if (!initializePinpoint(pose) || !pose.anchor(0, 0, 0)) {
                outcome = "Pinpoint 航向未就绪：" + pose.status;
                return;
            }
            if (!remeasure) {
                leftEquation = model.equation(1);
                rightEquation = model.equation(-1);
                verifyLinearity(drive, pose, model, Double.NaN);
                outcome = "沿用已保存函数并完成线性复测";
                offerLiveTest(drive, pose, model);
                return;
            }

            double ksLeft = measureBreakaway(drive, pose, 1);
            if (!active()) {
                outcome = "已中止";
                return;
            }
            double ksRight = measureBreakaway(drive, pose, -1);
            if (!active()) {
                outcome = "已中止";
                return;
            }
            if (!(ksLeft > 0.02)) ksLeft = 0.14;
            if (!(ksRight > 0.02)) ksRight = 0.14;
            telemetry.log().add("起转死区 12V 基准：左 %.2f / 右 %.2f", ksLeft, ksRight);

            double[][] duties = {buildSweep(ksLeft), buildSweep(ksRight)};
            double[][] rates = {new double[SWEEP_POINTS], new double[SWEEP_POINTS]};
            for (int i = 0; i < SWEEP_POINTS && active(); i++) {
                int first = i % 2 == 0 ? 0 : 1;
                for (int k = 0; k < 2 && active(); k++) {
                    int side = (first + k) % 2;
                    int dir = side == 0 ? 1 : -1;
                    double r = measureSteadyRate(drive, pose, dir, duties[side][i],
                            "自动虚拟杆量扫描", i + 1, SWEEP_POINTS);
                    if (!Double.isFinite(r) && active()) {
                        r = measureSteadyRate(drive, pose, dir, duties[side][i],
                                "自动虚拟杆量扫描", i + 1, SWEEP_POINTS);
                    }
                    rates[side][i] = r;
                }
            }
            if (!active()) {
                outcome = "扫描已中止，未覆盖原参数";
                return;
            }
            double[][] cleanLeft = compactValid(duties[0], rates[0]);
            double[][] cleanRight = compactValid(duties[1], rates[1]);
            TurnLinearity fitted = TurnLinearity.fit(
                    ksLeft, cleanLeft[0], cleanLeft[1],
                    ksRight, cleanRight[0], cleanRight[1]);
            if (fitted == null) {
                outcome = "有效角速度样本不足或畸变过大，保留原参数";
                return;
            }
            double rawRmse = uncalibratedRmse(duties, rates, fitted.omegaMax());
            if (!save(CalibrationStore.TURN_LINEARITY, fitted.values())) {
                outcome = "校准函数写入失败";
                return;
            }
            model = fitted;
            leftEquation = fitted.equation(1);
            rightEquation = fitted.equation(-1);
            verifyLinearity(drive, pose, fitted, rawRmse);
            outcome = "已生成并保存转向线性校准函数";
            offerLiveTest(drive, pose, fitted);
        } finally {
            drive.stop();
            if (turret != null) turret.teachingMode(false);
        }
    }

    @Override void showResults() {
        telemetry.addData("校准状态", outcome);
        telemetry.addData("左转函数", leftEquation);
        telemetry.addData("右转函数", rightEquation);
        telemetry.addData("线性度复测", verifySummary);
        for (String row : verifyRows) telemetry.addLine(row);
    }

    private Turret optionalTurret() {
        try {
            return new Turret(hardwareMap, store, battery);
        } catch (RuntimeException unavailable) {
            return null;
        }
    }

    private static double[] buildSweep(double ks) {
        double start = Maths.clip(ks + 0.04, 0.14, 0.42);
        double[] levels = new double[SWEEP_POINTS];
        for (int i = 0; i < SWEEP_POINTS; i++) {
            double frac = i / (double) (SWEEP_POINTS - 1);
            levels[i] = start + (1.0 - start) * frac;
        }
        return levels;
    }

    private double measureBreakaway(Drive drive, PinpointPose pose, int direction) throws InterruptedException {
        if (!waitQuiet(drive, pose, 0.90)) return Double.NaN;
        double startYaw = pose.sensorYaw();
        double[] detected = {Double.NaN};
        try {
            runFor(1.60, (t, dt) -> {
                pose.update();
                if (!pose.headingValid || !pose.headingUsable()) return false;
                double nominal = Math.min(0.45, 0.05 + 0.34 * t);
                double duty = battery.duty(nominal, 1.0);
                drive.robot(0, 0, direction * duty);
                double moved = direction * Maths.wrap(pose.sensorYaw() - startYaw);
                double rate = direction * pose.yawRate;
                if (!Double.isFinite(detected[0]) && moved >= Math.toRadians(2.2) && rate >= 0.14) {
                    detected[0] = nominal;
                    return false;
                }
                show("测起转死区 · " + (direction > 0 ? "左转" : "右转"),
                        "虚拟杆量 %.2f，角速度 %.0f°/s", nominal, Math.toDegrees(rate));
                return true;
            });
        } finally {
            drive.stop();
        }
        return detected[0];
    }

    private double measureSteadyRate(Drive drive, PinpointPose pose, int direction, double command,
                                     String title, int stepIndex, int stepTotal) throws InterruptedException {
        if (!waitQuiet(drive, pose, 0.90)) return Double.NaN;
        double startYaw = pose.sensorYaw();
        Measurements steady = new Measurements();
        boolean mapped = title.contains("复测");
        try {
            runFor(0.60, (t, dt) -> {
                pose.update();
                if (!pose.headingValid || !pose.headingUsable()) return false;
                double duty = mapped
                        ? model.mapDuty(direction * command, 0, 0, battery, 1.0)
                        : direction * battery.duty(command, 1.0);
                drive.robot(0, 0, duty);
                double rate = direction * pose.yawRate;
                if (t >= 0.18 && Double.isFinite(rate)) steady.add(Math.max(0, rate));
                double turned = Math.abs(Maths.wrap(pose.sensorYaw() - startYaw));
                show(title + " · " + (direction > 0 ? "左" : "右") + " " + stepIndex + "/" + stepTotal,
                        "虚拟杆量 %.2f → 占空比 %+.2f | 角速度 %.0f°/s",
                        command, duty, Math.toDegrees(rate));
                return t < 0.34 || (turned < Math.toRadians(110) && t < 0.58);
            });
        } finally {
            drive.stop();
        }
        return steady.size() >= 3 ? steady.median() : Double.NaN;
    }

    private boolean waitQuiet(Drive drive, PinpointPose pose, double timeout) throws InterruptedException {
        drive.stop();
        double[] rest = {0};
        runFor(timeout, (t, dt) -> {
            pose.update();
            double rawRate = pose.observedYawRate();
            if (pose.headingValid && Double.isFinite(rawRate) && Math.abs(rawRate) < 0.08) rest[0] += dt;
            else rest[0] = 0;
            return rest[0] < 0.12;
        });
        if (active() && pose.headingValid && !pose.headingUsable()) pose.anchor(0, 0, 0);
        return active() && pose.headingValid && pose.headingUsable();
    }

    private static double[][] compactValid(double[] duties, double[] rates) {
        int count = 0;
        for (int i = 0; i < duties.length; i++) {
            if (Maths.finite(duties[i], rates[i])) count++;
        }
        double[] validDuties = new double[count];
        double[] validRates = new double[count];
        int j = 0;
        for (int i = 0; i < duties.length; i++) {
            if (!Maths.finite(duties[i], rates[i])) continue;
            validDuties[j] = duties[i];
            validRates[j] = rates[i];
            j++;
        }
        return new double[][]{validDuties, validRates};
    }

    private void verifyLinearity(Drive drive, PinpointPose pose, TurnLinearity activeModel, double rawRmse)
            throws InterruptedException {
        verifyRows.clear();
        this.model = activeModel;
        double sqErr = 0, sqTot = 0, sumY = 0;
        int n = 0;
        double[] measuredAll = new double[VERIFY_STICKS.length * 2];
        double[] idealAll = new double[VERIFY_STICKS.length * 2];
        for (int i = 0; i < VERIFY_STICKS.length && active(); i++) {
            double u = VERIFY_STICKS[i];
            double ideal = u * activeModel.omegaMax();
            double leftRate = measureSteadyRate(drive, pose, 1, u, "线性度自动复测", i + 1, VERIFY_STICKS.length);
            double rightRate = measureSteadyRate(drive, pose, -1, u, "线性度自动复测", i + 1, VERIFY_STICKS.length);
            if (!active()) break;
            if (Double.isFinite(leftRate)) {
                measuredAll[n] = leftRate;
                idealAll[n] = ideal;
                sumY += leftRate;
                n++;
            }
            if (Double.isFinite(rightRate)) {
                measuredAll[n] = rightRate;
                idealAll[n] = ideal;
                sumY += rightRate;
                n++;
            }
            verifyRows.add(String.format(Locale.ROOT, "u=%.2f 理想%.0f°/s → 左%.0f | 右%.0f",
                    u, Math.toDegrees(ideal), Math.toDegrees(leftRate), Math.toDegrees(rightRate)));
        }
        if (n < 4) {
            verifySummary = "复测采样不足";
            return;
        }
        double meanY = sumY / n;
        for (int i = 0; i < n; i++) {
            double e = measuredAll[i] - idealAll[i];
            sqErr += e * e;
            double d = measuredAll[i] - meanY;
            sqTot += d * d;
        }
        double rmse = Math.sqrt(sqErr / n);
        double r2 = sqTot > 1e-6 ? 1 - sqErr / sqTot : 1;
        verifySummary = Double.isFinite(rawRmse)
                ? String.format(Locale.ROOT, "校正前 %.1f°/s → 校正后 %.1f°/s (%.1f%%), R²=%.3f, 峰值 %.0f°/s",
                Math.toDegrees(rawRmse), Math.toDegrees(rmse), 100 * rmse / activeModel.omegaMax(), r2,
                Math.toDegrees(activeModel.omegaMax()))
                : String.format(Locale.ROOT, "线性误差 %.1f°/s (%.1f%%), R²=%.3f, 峰值 %.0f°/s",
                Math.toDegrees(rmse), 100 * rmse / activeModel.omegaMax(), r2,
                Math.toDegrees(activeModel.omegaMax()));
    }

    private static double uncalibratedRmse(double[][] duties, double[][] rates, double omegaMax) {
        double sq = 0;
        int n = 0;
        for (int side = 0; side < 2; side++) {
            for (int i = 0; i < duties[side].length; i++) {
                if (!Maths.finite(duties[side][i], rates[side][i])) continue;
                double e = rates[side][i] - duties[side][i] * omegaMax;
                sq += e * e;
                n++;
            }
        }
        return n > 0 ? Math.sqrt(sq / n) : Double.NaN;
    }

    private void offerLiveTest(Drive drive, PinpointPose pose, TurnLinearity activeModel) throws InterruptedException {
        if (!confirm("A 进入右摇杆实时试驾验证，B 直接结束并查看校准函数。")) return;
        try {
            runFor(45, (t, dt) -> {
                if (gamepad1.x) return false;
                pose.update();
                if (pose.headingValid && !pose.headingUsable()) pose.anchor(0, 0, 0);
                double f = Maths.deadband(-gamepad1.left_stick_y);
                double l = Maths.deadband(-gamepad1.left_stick_x);
                double rawTurn = Maths.deadband(-gamepad1.right_stick_x);
                double turn = activeModel.mapDuty(rawTurn, f, l, battery, 1.0);
                drive.robot(f, l, turn);
                show("右摇杆实时线性试驾（按 X 结束，B 急停）",
                        "杆量 %+.2f → 输出 %+.2f | 期望 %+.0f°/s | 实测 %+.0f°/s",
                        rawTurn, turn, Math.toDegrees(activeModel.expectedRate(rawTurn)),
                        Math.toDegrees(pose.yawRate));
                return true;
            });
        } finally {
            drive.stop();
        }
    }

    private boolean active() { return opModeIsActive() && !motionAborted(); }
}
