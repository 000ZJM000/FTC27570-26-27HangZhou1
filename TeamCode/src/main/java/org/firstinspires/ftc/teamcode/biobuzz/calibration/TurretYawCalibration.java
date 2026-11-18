package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import org.firstinspires.ftc.teamcode.biobuzz.CalibrationStore;
import org.firstinspires.ftc.teamcode.biobuzz.Drive;
import org.firstinspires.ftc.teamcode.biobuzz.Maths;
import org.firstinspires.ftc.teamcode.biobuzz.PinpointPose;
import org.firstinspires.ftc.teamcode.biobuzz.Turret;

final class TurretYawCalibration {
    private static final int PASSES = 4;
    private static final double[] STICKS = {0.18, 0.29, 0.42, 0.58, 0.77, 1.0};
    private static final double[] HOLDOUT = {0.21, 0.51, 1.0};
    private final CalibrationOp op;
    private int trials;

    TurretYawCalibration(CalibrationOp op) {
        this.op = op;
    }

    void run(Drive drive, PinpointPose pose) throws InterruptedException {
        Turret turret = new Turret(op.hardwareMap, op.store);
        if (!turret.calibrated()) {
            op.telemetry.log().add("云台行程待标定。运行 02 后再测耦合前馈。");
            return;
        }
        if (!turret.plantIdentified()) {
            op.telemetry.log().add("先运行 02 并通过快速稳定性验证。");
            return;
        }
        try {
            turret.teachingMode(true);
            while (op.confirm("把云台摆到最右标记，按 A 记零位。")) {
                turret.homeAtRight();
                if (turret.encoderReady()) break;
                op.telemetry.log().add("等待云台编码器，保持在右端后重试。");
            }
            if (!op.opModeIsActive() || !turret.encoderReady()) return;
            turret.teachingMode(false);
            if (!op.confirm("底盘与云台将同时左右转。先测 24 次，缺数据的方向才补测。A 开始，B 停止。")) return;
            pose.update();
            if (!pose.anchor(0, 0, 0)) {
                op.telemetry.log().add("Pinpoint 航向暂不可用。恢复后重跑 03。");
                return;
            }

            double[] original = turret.coupling();
            CouplingFit[] data = {new CouplingFit(), new CouplingFit()};
            double[][] fits = new double[2][];
            for (int pass = 0; pass < PASSES && active(); pass++) {
                for (int i = 0; i < STICKS.length && active(); i++) {
                    int level = (i * 5 + pass) % STICKS.length;
                    for (int j = 0; j < 2 && active(); j++) {
                        int side = (j + pass + i) % 2;
                        if (fits[side] != null) continue;
                        turn(drive, pose, turret, signedStick(STICKS[level], side), 0, (pass + i) % 4, data[side]);
                    }
                }
                if (pass >= 1) {
                    for (int side = 0; side < 2 && active(); side++) {
                        if (fits[side] == null) fits[side] = data[side].fit();
                    }
                    if (fits[0] != null && fits[1] != null) break;
                }
            }
            if (!active()) return;
            double[] verified = original.clone();
            boolean improved = false;
            for (int side = 0; side < 2 && active(); side++) {
                double[] fit = fits[side];
                String direction = side == 0 ? "云台左转" : "云台右转";
                if (fit == null) {
                    op.telemetry.log().add("%s：有效数据不足（%d 段），保留原参数。", direction, data[side].size());
                    continue;
                }
                double[] proposed = original.clone();
                System.arraycopy(fit, 0, proposed, side * 2, 2);
                op.telemetry.log().add("%s %d 段，kV %.3f / kA %.3f", direction,
                        data[side].size(), fit[0], fit[1]);
                if (validate(drive, pose, turret, side, original, proposed, direction)) {
                    System.arraycopy(fit, 0, verified, side * 2, 2);
                    improved = true;
                }
                turret.setCoupling(original);
            }
            if (improved && active()) op.save(CalibrationStore.TURRET_COUPLED_DYNAMICS, verified);
        } finally {
            drive.stop();
            turret.teachingMode(false);
        }
    }

    private boolean validate(Drive drive, PinpointPose pose, Turret turret, int side, double[] original,
                             double[] proposed, String direction) throws InterruptedException {
        Measurements baseline = new Measurements(), candidate = new Measurements();
        double worstChange = Double.NEGATIVE_INFINITY;
        int retry = -1;
        for (int run = 0; run <= HOLDOUT.length && candidate.size() < HOLDOUT.length && active(); run++) {
            int i = run < HOLDOUT.length ? run : retry;
            if (i < 0) break;
            double stick = signedStick(HOLDOUT[i], side);
            double[] scores = new double[2];
            for (int j = 0; j < 2 && active(); j++) {
                int model = (i + j) % 2;
                turret.setCoupling(model == 0 ? original : proposed);
                scores[model] = turn(drive, pose, turret, stick, i % 2 == 0 ? 1 : 2, (i + 2) % 4, null);
            }
            if (!Maths.finite(scores)) {
                if (retry < 0) retry = i;
                continue;
            }
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
        return accepted;
    }

    private double turn(Drive drive, PinpointPose pose, Turret turret, double stick, int pattern, int lane,
                        CouplingFit data) throws InterruptedException {
        int direction = stick > 0 ? -1 : 1;
        double from = 0.18 + 0.11 * lane;
        double start = turret.span() * (direction > 0 ? from : 1 - from);
        if (!active() || !TurretMotion.go(op, turret, start, Math.toRadians(3.0))) return Double.NaN;
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
                turret.aim(pose, null, effective, dt);
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
            double duty = direction * turret.output(); // power applied during the preceding interval
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
