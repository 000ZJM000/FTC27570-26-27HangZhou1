package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.biobuzz.CalibrationStore;
import org.firstinspires.ftc.teamcode.biobuzz.Maths;
import org.firstinspires.ftc.teamcode.biobuzz.RobotConfig;
import org.firstinspires.ftc.teamcode.biobuzz.Turret;

@TeleOp(name = "02 自测云台 行程 摩擦 刹车模型", group = "BIOBUZZ 自测")
public final class TurretCalibration extends CalibrationOp {
    private int homeTick;

    @Override String intro() { return "先记行程，再做短脉冲快速辨识和停稳验证，通过后才精测全圈。"; }

    @Override void calibrate() throws InterruptedException {
        Turret turret = new Turret(hardwareMap, store);
        try {
            turret.teachingMode(true);
            if (!confirm("把云台轻轻摆到最右端，停稳后 A。")) return;
            homeTick = turret.rawTicks();
            if (confirm("重测零位方向、正前和行程？已有标定可按 B 跳过。")) {
                if (teach(turret)) turret = new Turret(hardwareMap, store);
                else telemetry.log().add("本次几何数据无效，尝试沿用之前的标定。");
            }
            if (!turret.calibrated() || !opModeIsActive()) {
                telemetry.log().add("需要完成方向、正前方与限位标定才能自动上电。");
                return;
            }
            turret.homeAtRight(homeTick);
            if (!center(turret)) return;
            while (!new TurretQuickIdentification(this, turret).run()) {
                if (!opModeIsActive() || motionAborted()
                        || !confirm("快速基线未通过。A 回中后重试，B 结束。") || !center(turret)) return;
            }
            if (confirm("整段行程分 12 段，各段双向测摩擦，每向 4 次。按 A 开始。")) {
                new TurretFrictionCalibration(this).run(turret);
            }
            if (!motionAborted()) verifyEncoderReturn(turret);
        } finally {
            turret.teachingMode(false);
        }
    }

    private void verifyEncoderReturn(Turret turret) throws InterruptedException {
        turret.teachingMode(true);
        if (!confirm("手动回到右端标记，A 核对往返后的编码器零位。", () -> {
            turret.sample(0.025);
            telemetry.addData("相对右端", "%+.2f°", Math.toDegrees(turret.angle));
        })) return;
        turret.sample(0.025);
        double drift = Math.abs(Math.toDegrees(turret.angle));
        telemetry.log().add(drift <= 2 ? "右端零位误差 %.2f°" :
                "右端偏差 %.2f°，请检查齿隙或 Hub 编码器口是否漏计数。", drift);
    }

    private boolean teach(Turret turret) throws InterruptedException {
        while (opModeIsActive()) {
            turret.teachingMode(true);
            if (!confirm("手动从最右端向左转 20–45°，A 识别编码器方向。")) return false;
            double test = (turret.rawTicks() - homeTick) / RobotConfig.TICKS_PER_TURRET_RADIAN;
            if (Math.abs(test) < Math.toRadians(8) || Math.abs(test) > Math.toRadians(80)) {
                telemetry.log().add("位移不符。检查 Expansion Hub 编码器 1 口，再调整角度。");
                continue;
            }
            double sign = Math.signum(test);
            if (!confirm("手动摆到云台与车头同向的位置，A 记录。")) return false;
            double forward = relative(turret, sign);
            if (!confirm("继续手动向左转到允许的最左端，A 记录。留意线束。")) return false;
            double span = relative(turret, sign);
            if (span < Math.PI / 2 || span > Maths.TURN + Math.toRadians(12)
                    || forward < -0.02 || forward > span + 0.02) {
                telemetry.log().add("角度不符：正前 %.0f° / 左端 %.0f°，请重新摆位。",
                        Math.toDegrees(forward), Math.toDegrees(span));
                continue;
            }
            double usedSpan = Math.min(span, Maths.TURN);
            if (!confirm("手动摆到约一半行程。下一步会轻脉冲电机判断旋向。", () ->
                    telemetry.addData("距右端", "%.1f°", Math.toDegrees(relative(turret, sign))))) return false;
            double center = relative(turret, sign);
            if (center < usedSpan * 0.31 || center > usedSpan * 0.69) {
                telemetry.log().add("离限位过近，回到中间再试。");
                if (!confirm("A 重试行程教学，B 保留已有标定。")) return false;
                continue;
            }
            turret.teachingMode(false);
            double driveSign = directionPulse(turret, sign, usedSpan);
            if (driveSign == 0) {
                telemetry.log().add("电机脉冲没有可靠位移。检查电机口、编码器、摩擦或按 B 跳过。");
                if (!confirm("A 重试，B 保留已有标定。")) return false;
                continue;
            }
            if (!opModeIsActive()) return false;
            telemetry.log().add("记录行程 %.1f° / 正前 %.1f° / 电机方向 %+.0f",
                    Math.toDegrees(usedSpan), Math.toDegrees(forward), driveSign);
            return save(CalibrationStore.HOME, sign, driveSign, usedSpan, Math.max(0, forward));
        }
        return false;
    }

    private double relative(Turret turret, double sign) {
        return sign * (turret.rawTicks() - homeTick) / RobotConfig.TICKS_PER_TURRET_RADIAN;
    }

    private double directionPulse(Turret turret, double sign, double span) throws InterruptedException {
        for (double power : new double[]{0.09, -0.09, 0.15, -0.15, 0.22, -0.22}) {
            double start = relative(turret, sign);
            boolean complete;
            try {
                complete = runFor(0.19, (t, dt) -> {
                    double position = relative(turret, sign);
                    if (position < span * 0.16 || position > span * 0.84) return false;
                    turret.motorPulse(power);
                    return true;
                });
            } finally {
                turret.stop();
            }
            double movement = relative(turret, sign) - start;
            if (Math.abs(movement) > Math.toRadians(0.6) && !motionAborted() && opModeIsActive()) {
                return Math.signum(movement * power);
            }
            if (!complete) break;
            sleep(110);
        }
        return 0;
    }

    private boolean center(Turret turret) throws InterruptedException {
        turret.teachingMode(true);
        while (confirm("将云台移到行程中间，停稳后 A。", () -> {
            turret.sample(0.025);
            telemetry.addData("行程比例", "%.0f%%", 100 * turret.angle / turret.span());
        })) {
            turret.homeAtRight(homeTick);
            if (turret.encoderReady() && Math.abs(turret.angle - turret.span() / 2) < turret.span() * 0.17) {
                turret.teachingMode(false);
                return true;
            }
            telemetry.log().add("当前角度不在中央，检查最右端基准。");
        }
        turret.teachingMode(false);
        return false;
    }
}
