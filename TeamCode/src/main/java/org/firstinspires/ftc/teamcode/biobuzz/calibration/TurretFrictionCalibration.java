package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import org.firstinspires.ftc.teamcode.biobuzz.CalibrationStore;
import org.firstinspires.ftc.teamcode.biobuzz.Maths;
import org.firstinspires.ftc.teamcode.biobuzz.RobotConfig;
import org.firstinspires.ftc.teamcode.biobuzz.Turret;
import org.firstinspires.ftc.teamcode.biobuzz.TurretFriction;
import org.firstinspires.ftc.teamcode.biobuzz.TurretPlant;

final class TurretFrictionCalibration {
    private static final int SECTORS = TurretFriction.SECTORS;
    private static final int[] PASSES = {0, 3, 1};
    private static final int[] ANCHORS = {0, 2, 4, 7, 9, 11};
    private final CalibrationOp op;

    TurretFrictionCalibration(CalibrationOp op) { this.op = op; }

    boolean run(Turret turret) throws InterruptedException {
        Bin[][] data = new Bin[SECTORS][2];
        for (Bin[] row : data) for (int side = 0; side < 2; side++) row[side] = new Bin();
        for (int round = 0; round < PASSES.length && active(); round++) {
            for (int i = 0; i < ANCHORS.length && active(); i++) {
                int sector = ANCHORS[round % 2 == 0 ? i : ANCHORS.length - i - 1];
                for (int j = 0; j < 2 && active(); j++) {
                    int direction = (j + round + sector) % 2 == 0 ? 1 : -1;
                    Bin bin = data[sector][TurretFriction.side(direction)];
                    if (!bin.consistent() || bin.movingCount < 2) {
                        measure(turret, bin, sector, direction, PASSES[round], round + 1);
                    }
                }
            }
        }
        if (!active()) return false;
        return save(data);
    }

    private void measure(Turret turret, Bin bin, int sector, int direction, int pulse, int round)
            throws InterruptedException {
        double width = TurretFriction.width(turret.span());
        double target = TurretFriction.center(sector, turret.span()) + (pulse - 1.5) * 0.12 * width;
        double tolerance = Math.min(Math.toRadians(1.8), width * 0.18);
        if (!TurretMotion.go(op, turret, target, tolerance)) return;
        double origin = turret.angle;
        double[] onset = {-1, 0, 0};
        Measurements speeds = new Measurements(), duty = new Measurements();
        try {
            op.runFor(1.8, (time, dt) -> {
                turret.sample(dt);
                if (!turret.encoderReady() || !turret.motorResponding()) return false;
                double movement = direction * (turret.angle - origin);
                double speed = direction * turret.speed;
                if (onset[0] < 0 && movement > Math.toRadians(0.38) && speed > 0.09) {
                    onset[0] = time;
                    onset[1] = Math.abs(turret.output());
                    onset[2] = turret.angle;
                }
                double maxMotion = Math.min(width * 0.75, Math.toRadians(14));
                double available = direction > 0 ? turret.span() - turret.angle : turret.angle;
                if (Math.abs(movement) > maxMotion || available < RobotConfig.TURRET_MARGIN + Math.toRadians(1)) {
                    return false;
                }
                double power = Math.min(0.66, 0.025 + time * (0.39 + pulse * 0.025));
                if (onset[0] >= 0) {
                    if (time - onset[0] > 0.19) return false;
                    power = Math.min(0.72, onset[1] + 0.04 + pulse * 0.044);
                    if (time - onset[0] > 0.075 && speed > 0.15
                            && Math.abs(turret.angle - onset[2]) < width * 0.52) {
                        speeds.add(speed);
                        duty.add(Math.abs(turret.output()));
                    }
                }
                turret.power(direction * power);
                op.show("代表角摩擦测量", "%d/%d 位置，%s，第 %d 次",
                        sector + 1, SECTORS, direction > 0 ? "左转" : "右转", round);
                return true;
            });
        } finally {
            turret.stop();
        }
        if (active() && onset[1] > 0.015) bin.start.add(onset[1]);
        if (speeds.size() >= 4 && speeds.relativeSpread() <= 0.22) bin.addMoving(speeds.median(), duty.median());
    }

    private boolean save(Bin[][] data) {
        double[] model = new TurretFriction(op.store).values();
        TurretPlant baseline = TurretPlant.load(op.store);
        for (int direction : new int[]{1, -1}) {
            int side = TurretFriction.side(direction);
            model[side] = baseline.kV(direction);
            int count = 0;
            for (int sector : ANCHORS) {
                Bin bin = data[sector][side];
                if (bin.consistent()) count++;
            }
            if (count < 4) {
                op.telemetry.log().add("%s有效段仅 %d/%d，保留原摩擦表。", direction > 0 ? "左转" : "右转",
                        count, ANCHORS.length);
                return false;
            }
            op.telemetry.log().add("%s有效摩擦测量 %d/%d 段", direction > 0 ? "左转" : "右转", count, SECTORS);
            for (int sector : ANCHORS) {
                Bin bin = data[sector][side];
                if (!bin.consistent()) continue;
                int i = TurretFriction.index(sector, direction);
                model[i] = Maths.clip(0.96 * bin.start.median(), 0.02, 0.62);
                Measurements rolling = new Measurements();
                for (int j = 0; j < bin.movingCount; j++) {
                    rolling.add(bin.powers[j] - model[side] * bin.speeds[j]);
                }
                model[i + 1] = rolling.size() >= 2
                        ? Maths.clip(rolling.median(), 0.015, 1.1 * model[i]) : model[i] * 0.80;
            }
            for (int sector = 0; sector < SECTORS; sector++) {
                if (data[sector][side].consistent()) continue;
                int before = -1, after = -1;
                for (int anchor : ANCHORS) {
                    if (!data[anchor][side].consistent()) continue;
                    if (anchor < sector) before = anchor;
                    if (anchor > sector && after < 0) after = anchor;
                }
                if (before < 0) before = after;
                if (after < 0) after = before;
                double t = before == after ? 0 : (double) (sector - before) / (after - before);
                double blend = t * t * (3 - 2 * t);
                int i = TurretFriction.index(sector, direction);
                int a = TurretFriction.index(before, direction), b = TurretFriction.index(after, direction);
                for (int j = 0; j < 2; j++) model[i + j] = model[a + j] + blend * (model[b + j] - model[a + j]);
            }
        }
        return op.save(CalibrationStore.TURRET_FRICTION_MAP, model);
    }

    private boolean active() { return op.opModeIsActive() && !op.motionAborted(); }

    private static final class Bin {
        final Measurements start = new Measurements();
        final double[] speeds = new double[PASSES.length], powers = new double[speeds.length];
        int movingCount;

        boolean consistent() { return start.size() >= 2 && start.relativeSpread() <= 0.17; }

        void addMoving(double speed, double power) {
            speeds[movingCount] = speed;
            powers[movingCount++] = power;
        }
    }
}
