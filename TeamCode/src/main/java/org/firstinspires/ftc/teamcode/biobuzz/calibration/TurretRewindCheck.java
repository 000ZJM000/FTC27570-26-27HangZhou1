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

    TurretRewindCheck(CalibrationOp op, Turret turret) {
        this.op = op;
        this.turret = turret;
    }

    String run() throws InterruptedException {
        if (turret.span() < Math.toRadians(270)) return "已跳过（行程不足 270°）";
        if (!op.confirm("回转测试：清空周围，底盘会依次向左右转。按 A 开始，B 跳过。")) return "已跳过";
        Drive drive;
        try {
            drive = new Drive(op.hardwareMap);
        } catch (RuntimeException unavailable) {
            return "找不到底盘，已跳过";
        }
        PinpointPose pose = new PinpointPose(op.hardwareMap);
        try {
            if (!pose.startImu()) return "Pinpoint 未就绪";
            op.runFor(1.15, (t, dt) -> { pose.update(); return true; });
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
        if (!TurretMotion.go(op, turret, start, Math.toRadians(2.5))) return label + "未到达预备角";
        pose.update();
        if (!pose.headingUsable()) return label + "没有航向";
        double startHeading = pose.heading;
        double turnLimit = Maths.TURN - travel + edgeDistance + Math.toRadians(13);
        boolean[] flags = new boolean[4]; // began, predicted, braking, finished
        double[] peak = {0};
        try {
            op.runFor(6.5, (t, dt) -> {
                pose.update();
                turret.sample(dt);
                if (!pose.headingUsable() || !turret.encoderReady() || !turret.motorResponding()) return false;
                double turned = yawSign * Maths.wrap(pose.heading - startHeading);
                double stick = t >= 0.14 && turned < turnLimit ? yawSign * 0.32 : 0;
                double effective = drive.robot(0, 0, stick);
                turret.aim(pose, null, effective, dt);
                if (turret.rewinding()) {
                    flags[0] = true;
                    flags[1] |= turret.rewindPredicted();
                    flags[2] |= Math.abs(turret.brakeDuty()) > 0.024;
                    peak[0] = Math.max(peak[0], Math.abs(turret.output()));
                }
                op.show("PID 回转 " + label, "底盘 %+.0f°/s / 云台 %+.0f°/s / 功率 %.2f / 峰值 %.2f",
                        Math.toDegrees(pose.yawRate), Math.toDegrees(turret.speed), turret.output(), peak[0]);
                if (flags[0] && !turret.rewinding() && stick == 0
                        && Math.abs(turret.worldError) < Math.toRadians(6)) {
                    flags[3] = true;
                    return false;
                }
                return true;
            });
        } finally {
            drive.stop();
            turret.stop();
        }
        return String.format(Locale.US, "%s%s，%s，峰值 %.2f，%s", label, flags[3] ? "回转完成" : "未完成",
                flags[1] ? "提前触发" : "未提前触发", peak[0], flags[2] ? "有刹车" : "未进入刹车");
    }

    private boolean active() { return op.opModeIsActive() && !op.motionAborted(); }
}
