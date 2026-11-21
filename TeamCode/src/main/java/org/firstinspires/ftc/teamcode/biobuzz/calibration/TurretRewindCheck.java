package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import org.firstinspires.ftc.teamcode.biobuzz.Drive;
import org.firstinspires.ftc.teamcode.biobuzz.Maths;
import org.firstinspires.ftc.teamcode.biobuzz.PinpointPose;
import org.firstinspires.ftc.teamcode.biobuzz.RobotConfig;
import org.firstinspires.ftc.teamcode.biobuzz.Turret;

import java.util.Locale;

final class TurretRewindCheck {
    private final CalibrationOp op;
    private final Turret turret;
    private int passedTurns;

    TurretRewindCheck(CalibrationOp op, Turret turret) {
        this.op = op;
        this.turret = turret;
    }

    static boolean supported(Turret turret) { return turret.span() >= Math.toRadians(270); }
    boolean passed() { return passedTurns == 2; }

    String run(boolean ask) throws InterruptedException {
        if (ask && !op.confirm("回转验证：清空周围，底盘会依次向左右转。A 开始，双向通过才保存；B 保留原 PID。")) {
            return "已跳过";
        }
        Drive drive;
        try {
            drive = new Drive(op.hardwareMap);
        } catch (RuntimeException unavailable) {
            return "找不到底盘，已跳过";
        }
        PinpointPose pose = new PinpointPose(op.hardwareMap, op.battery);
        try {
            if (!op.initializePinpoint(pose)) return "Pinpoint 未就绪";
            return run(drive, pose);
        } finally {
            drive.stop();
            turret.stop();
        }
    }

    String run(Drive drive, PinpointPose pose) throws InterruptedException {
        try {
            pose.update();
            if (!pose.anchor(0, 0, 0)) return "Pinpoint 航向未就绪";
            String left = turn(drive, pose, 1);
            if (!active()) return left;
            return left + "；" + turn(drive, pose, -1);
        } finally {
            drive.stop();
            turret.stop();
        }
    }

    private String turn(Drive drive, PinpointPose pose, int yawSign) throws InterruptedException {
        double edgeDistance = Math.toRadians(17);
        double travel = turret.span() - 2 * RobotConfig.TURRET_MARGIN;
        double start = RobotConfig.TURRET_MARGIN + edgeDistance;
        if (yawSign < 0) start = turret.span() - start;
        String label = yawSign > 0 ? "底盘左转" : "底盘右转";
        if (!TurretMotion.goDirect(op, turret, start, label)) return label + "未到达预备角";
        pose.update();
        if (!pose.headingValid || !pose.headingUsable()) return label + "没有航向，" + pose.status;
        double startHeading = pose.heading;
        double turnLimit = Maths.TURN - travel + edgeDistance + Math.toRadians(13);
        boolean[] flags = new boolean[4]; // began, predicted, braking, finished
        double[] peak = {0};
        double[] timing = {Double.NaN, Double.NaN};
        double[] settled = {0};
        String[] failure = {"未到位"};
        try {
            op.runFor(6.5, (t, dt) -> {
                pose.update();
                turret.sample(dt);
                if (!pose.headingUsable() || !turret.encoderReady() || !turret.motorResponding()) {
                    failure[0] = !pose.headingUsable() ? pose.status : "云台反馈中断";
                    return false;
                }
                double turned = yawSign * Maths.wrap(pose.heading - startHeading);
                double stick = pose.headingValid && t >= 0.14 && turned < turnLimit ? yawSign * 0.32 : 0;
                double effective = drive.robot(0, 0, stick);
                turret.aim(pose, null, effective, dt);
                if (turret.rewinding()) {
                    if (!flags[0]) timing[0] = t;
                    flags[0] = true;
                    flags[1] |= turret.rewindPredicted();
                    flags[2] |= Math.abs(turret.brakeDuty()) > 0.024;
                    peak[0] = Math.max(peak[0], Math.abs(turret.output()));
                }
                if (flags[0] && !turret.rewinding() && !Double.isFinite(timing[1])) timing[1] = t - timing[0];
                op.show("PID 回转 " + label, "底盘 %+.0f°/s / 云台 %+.0f°/s / 功率 %.2f / 峰值 %.2f",
                        Math.toDegrees(pose.yawRate), Math.toDegrees(turret.speed), turret.output(), peak[0]);
                boolean aligned = pose.headingValid && flags[0] && turned >= turnLimit && turret.aimReady()
                        && Math.abs(turret.target - turret.angle) <= RobotConfig.SHOT_AIM_TOLERANCE
                        && Math.abs(pose.yawRate) < 0.25 && Math.abs(turret.speed) < 0.32;
                settled[0] = aligned ? settled[0] + dt : 0;
                if (settled[0] >= 0.12) {
                    flags[3] = true;
                    return false;
                }
                return true;
            });
        } finally {
            drive.stop();
            turret.stop();
        }
        if (flags[3]) passedTurns++;
        String duration = Double.isFinite(timing[1]) ? String.format(Locale.US, "%.2f 秒", timing[1]) : failure[0];
        if (!flags[3] && Double.isFinite(timing[1])) duration += "，" + failure[0];
        return String.format(Locale.US, "%s%s，%s，峰值 %.2f，%s，%s", label, flags[3] ? "回转完成" : "未完成",
                flags[1] ? "提前触发" : "未提前触发", peak[0], flags[2] ? "有刹车" : "未进入刹车", duration);
    }

    private boolean active() { return op.opModeIsActive() && !op.motionAborted(); }
}
