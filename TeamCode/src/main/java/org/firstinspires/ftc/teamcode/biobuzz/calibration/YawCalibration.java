package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.biobuzz.calibration.CalibrationOp.Measurements;
import org.firstinspires.ftc.teamcode.biobuzz.config.CalibrationStore;
import org.firstinspires.ftc.teamcode.biobuzz.core.Maths;
import org.firstinspires.ftc.teamcode.biobuzz.localization.PinpointPose;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Drive;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Turret;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@TeleOp(name = "03 云台专项 12V联动补偿", group = "BIOBUZZ 自测")
public final class YawCalibration extends CalibrationOp {
    @Override String intro() { return "轮子落地并腾出空地。用底盘转动测试云台反向补偿。"; }

    @Override void calibrate() throws InterruptedException {
        Turret turret = new Turret(hardwareMap, store, battery);
        try {
            if (!turret.calibrated() || !turret.plantIdentified()) {
                telemetry.log().add("先运行 02，保存云台行程和动力学。");
                return;
            }
            turret.teachingMode(true);
            if (!confirm("把云台摆到最右端，停稳后 A 记零位。")) return;
            turret.homeAtRight();
            turret.teachingMode(false);
            if (!turret.encoderReady()
                    || !confirm("轮子落地、清空周围并保持底盘静止。A 初始化 Pinpoint 后开始双向联动。")) return;
            PinpointPose pose = new PinpointPose(hardwareMap, battery);
            if (!initializePinpoint(pose)) {
                telemetry.log().add("Pinpoint 航向未就绪：%s。", pose.status);
                return;
            }
            Drive drive = new Drive(hardwareMap);
            try {
                boolean saved = runCoupling(this, drive, pose, turret);
                telemetry.log().add(saved ? "双向联动结果已验收保存" : "联动未保存，沿用之前的参数");
            } finally {
                drive.stop();
            }
        } finally {
            turret.teachingMode(false);
        }
    }

    static boolean runCoupling(CalibrationOp op, Drive drive, PinpointPose pose, Turret turret)
            throws InterruptedException {
        return new Runner(op).run(drive, pose, turret);
    }

    private static final class Runner {
        private static final int PASSES = 2;
        private static final double[] STICKS = {0.29, 0.54, 0.78};
        private static final double[] HOLDOUT = {0.42, 0.68};
        private final CalibrationOp op;
        private int trials;

        Runner(CalibrationOp op) { this.op = op; }

        boolean run(Drive drive, PinpointPose pose, Turret turret) throws InterruptedException {
            double[] original = turret.coupling();
            boolean saved = false;
            try {
                pose.update();
                if (!pose.anchor(0, 0, 0)) {
                    op.telemetry.log().add("Pinpoint 航向暂不可用。");
                    return false;
                }
                CouplingFit[] data = {new CouplingFit(), new CouplingFit()};
                for (int pass = 0; pass < PASSES && active(); pass++) {
                    for (int i = 0; i < STICKS.length && active(); i++) {
                        int level = (i + pass) % STICKS.length;
                        for (int j = 0; j < 2 && active(); j++) {
                            int side = (j + pass + i) % 2;
                            turn(drive, pose, turret, signedStick(STICKS[level], side), 0, (pass + i) % 4, data[side]);
                        }
                    }
                }
                if (!active()) return false;
                double[] verified = original.clone();
                for (int side = 0; side < 2 && active(); side++) {
                    double[] fit = data[side].fit();
                    String direction = side == 0 ? "云台左转" : "云台右转";
                    if (fit == null) {
                        op.telemetry.log().add("%s有效数据不足（%d 段），保留原参数。", direction, data[side].size());
                        return false;
                    }
                    double[] proposed = original.clone();
                    System.arraycopy(fit, 0, proposed, side * 2, 2);
                    op.telemetry.log().add("%s %d 段，kV %.3f / kA %.3f", direction,
                            data[side].size(), fit[0], fit[1]);
                    Validation result = validate(drive, pose, turret, side, original, proposed, direction);
                    if (result == Validation.INCOMPLETE) return false;
                    if (result == Validation.ADOPT) System.arraycopy(fit, 0, verified, side * 2, 2);
                    turret.setCoupling(original);
                }
                saved = active() && op.save(CalibrationStore.TURRET_COUPLED_DYNAMICS, verified);
                if (saved) turret.setCoupling(verified);
                return saved;
            } finally {
                drive.stop();
                turret.stop();
                if (!saved) turret.setCoupling(original);
            }
        }

        private enum Validation { ADOPT, KEEP, INCOMPLETE }

        private Validation validate(Drive drive, PinpointPose pose, Turret turret, int side, double[] original,
                                    double[] proposed, String direction) throws InterruptedException {
            Measurements baseline = new Measurements(), candidate = new Measurements();
            double worstChange = Double.NEGATIVE_INFINITY;
            for (int i = 0; i < HOLDOUT.length && active(); i++) {
                double stick = signedStick(HOLDOUT[i], side);
                double[] scores = {Double.NaN, Double.NaN};
                for (int j = 0; j < 2 && active(); j++) {
                    int model = (i + j) % 2;
                    turret.setCoupling(model == 0 ? original : proposed);
                    scores[model] = turn(drive, pose, turret, stick, i % 2 == 0 ? 1 : 2, (i + 2) % 4, null);
                }
                if (!Maths.finite(scores)) continue;
                baseline.add(scores[0]);
                candidate.add(scores[1]);
                worstChange = Math.max(worstChange, scores[1] - scores[0]);
            }
            double oldError = baseline.median(), newError = candidate.median();
            boolean accepted = active() && candidate.size() == HOLDOUT.length
                    && newError < oldError - Math.max(Math.toRadians(0.1), 0.03 * oldError)
                    && worstChange <= Math.max(Math.toRadians(1.5), 0.30 * oldError);
            if (candidate.size() == HOLDOUT.length) {
                op.telemetry.log().add("%s留出实测 %.2f° → %.2f°，%s", direction,
                        Math.toDegrees(oldError), Math.toDegrees(newError), accepted ? "采纳" : "保留原参数");
            } else op.telemetry.log().add("%s留出复测不足，保留原参数。", direction);
            if (!active() || candidate.size() < HOLDOUT.length) return Validation.INCOMPLETE;
            return accepted ? Validation.ADOPT : Validation.KEEP;
        }

        private double turn(Drive drive, PinpointPose pose, Turret turret, double stick, int pattern, int lane,
                            CouplingFit data) throws InterruptedException {
            int direction = stick > 0 ? -1 : 1;
            double from = 0.18 + 0.11 * lane;
            double start = turret.span() * (direction > 0 ? from : 1 - from);
            if (!active() || !TurretBench.go(op, turret, start, Math.toRadians(3.0))) return Double.NaN;
            double expectedRate = Math.max(0.75, Math.abs(stick) * 5.0);
            double duration = Maths.clip(turret.span() * (0.75 - from) / expectedRate, 0.23, 0.76);
            if (pattern == 1) duration *= 0.64;
            final double pulse = duration;
            TurnSample sample = new TurnSample();
            trials++;
            boolean completed;
            try {
                completed = op.runFor(0.12 + pulse + 0.48, (time, dt) -> {
                    pose.update();
                    turret.sample(dt);
                    if (!pose.headingValid || !pose.isAnchored() || !turret.encoderReady()
                            || Math.abs(turret.angle - turret.span() / 2) > turret.span() * 0.41) return false;
                    double elapsed = time - 0.12;
                    double command = elapsed >= 0 && elapsed < pulse ? stick : 0;
                    if (pattern == 2 && command != 0 && elapsed < pulse * 0.38) command *= 0.45;
                    if (data != null) sample.addWindow(data, turret, direction, dt);
                    double effective = drive.robot(0, 0, command);
                    turret.aim(pose, 0, 0, 0, 0, effective, dt);
                    if (!turret.motorResponding() || turret.rewinding()) return false;
                    if (elapsed >= 0) {
                        double error = Math.abs(turret.worldError);
                        sample.square += dt * error * error;
                        sample.time += dt;
                        sample.peak = Math.max(sample.peak, error);
                        sample.yaw = Math.max(sample.yaw, Math.abs(pose.yawRate));
                    }
                    op.show(data == null ? "底盘云台留出复测" : "底盘云台动态拟合",
                            "第 %d 次，杆量 %+.2f，地面误差 %+.1f°",
                            trials, effective, Math.toDegrees(turret.worldError));
                    return true;
                });
            } finally {
                drive.stop();
                turret.stop();
            }
            return completed && sample.time > 0.35 && sample.yaw > 0.12
                    ? 0.75 * Math.sqrt(sample.square / sample.time) + 0.25 * sample.peak : Double.NaN;
        }

        private boolean active() { return op.opModeIsActive() && !op.motionAborted(); }
        private static double signedStick(double stick, int side) { return side == 0 ? -stick : stick; }

        private static final class TurnSample {
            double square, time, peak, yaw;
            private double elapsed, powerArea, startAngle, startVelocity, previousAngle, previousVelocity;
            private boolean initialized;

            void addWindow(CouplingFit fit, Turret turret, int direction, double dt) {
                double angle = direction * turret.angle;
                double velocity = direction * turret.speed;
                double duty = direction * turret.output();
                if (!initialized || velocity < 0.15 || previousVelocity < 0.15 || Math.abs(duty) >= 0.98
                        || dt > 0.08) {
                    elapsed = powerArea = 0;
                    startAngle = angle;
                    startVelocity = velocity;
                } else {
                    if (elapsed == 0) {
                        startAngle = previousAngle;
                        startVelocity = previousVelocity;
                    }
                    elapsed += dt;
                    double midAngle = direction * (angle + previousAngle) * 0.5;
                    double rollingFriction = turret.frictionDuty(midAngle, direction, (velocity + previousVelocity) * 0.5);
                    powerArea += dt * (duty - rollingFriction);
                    if (elapsed >= 0.10) {
                        fit.add((angle - startAngle) / elapsed, (velocity - startVelocity) / elapsed,
                                powerArea / elapsed);
                        elapsed = powerArea = 0;
                    }
                }
                initialized = true;
                previousAngle = angle;
                previousVelocity = velocity;
            }
        }
    }

    private static final class CouplingFit {
        private static final double MAX_KV = 1.2, MAX_KA = 0.20;
        private final List<double[]> windows = new ArrayList<>();

        void add(double velocity, double acceleration, double power) {
            if (velocity > 0.12 && Maths.finite(velocity, acceleration, power)) {
                windows.add(new double[]{velocity, acceleration, power});
            }
        }

        int size() { return windows.size(); }

        double[] fit() {
            if (windows.size() < 12) return null;
            double[] speeds = windows.stream().mapToDouble(row -> row[0]).sorted().toArray();
            if (speeds[speeds.length * 9 / 10] - speeds[speeds.length / 10] < 0.40) return null;
            double[] weights = new double[windows.size()];
            double[] residuals = new double[weights.length];
            Arrays.fill(weights, 1);
            double kv = 0, ka = 0;
            for (int iteration = 0; iteration < 16; iteration++) {
                double vv = 0, aa = 0, va = 0, vp = 0, ap = 0;
                for (int i = 0; i < windows.size(); i++) {
                    double[] row = windows.get(i);
                    double v = row[0], a = row[1], p = row[2], weight = weights[i];
                    vv += weight * v * v;
                    aa += weight * a * a;
                    va += weight * v * a;
                    vp += weight * v * p;
                    ap += weight * a * p;
                }
                if (vv < 1e-7) return null;
                if (aa < 1e-7 || vv * aa - va * va < 0.04 * vv * aa) {
                    kv = Maths.clip(vp / vv, 0, MAX_KV);
                    ka = 0;
                } else {
                    for (int sweep = 0; sweep < 40; sweep++) {
                        kv = Maths.clip((vp - va * ka) / vv, 0, MAX_KV);
                        ka = Maths.clip((ap - va * kv) / aa, 0, MAX_KA);
                    }
                }
                for (int i = 0; i < residuals.length; i++) {
                    double[] row = windows.get(i);
                    residuals[i] = Math.abs(row[2] - row[0] * kv - row[1] * ka);
                }
                double cutoff = Math.max(0.022, 1.7 * Maths.median(residuals, residuals.length));
                for (int i = 0; i < weights.length; i++) {
                    weights[i] = Math.min(1, cutoff / Math.max(1e-9, residuals[i]));
                }
            }
            return kv > 0.002 ? new double[]{kv, ka} : null;
        }
    }
}
