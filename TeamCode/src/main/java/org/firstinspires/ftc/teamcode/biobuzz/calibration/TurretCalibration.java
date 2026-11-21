package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.biobuzz.CalibrationStore;
import org.firstinspires.ftc.teamcode.biobuzz.Drive;
import org.firstinspires.ftc.teamcode.biobuzz.Maths;
import org.firstinspires.ftc.teamcode.biobuzz.PinpointPose;
import org.firstinspires.ftc.teamcode.biobuzz.RobotConfig;
import org.firstinspires.ftc.teamcode.biobuzz.Turret;
import org.firstinspires.ftc.teamcode.biobuzz.TurretPlant;

@TeleOp(name = "02 云台全套 12V补偿·联动·PID", group = "BIOBUZZ 自测")
public final class TurretCalibration extends CalibrationOp {
    private int homeTick;
    private String geometry = "未测", plant = "未测", friction = "未测", coupling = "未测", pid = "未测";
    private TurretPidTuning tuning;

    @Override String intro() {
        return "一次完成行程、12V 动力学与摩擦、底盘联动、云台 PID 及回转验收。";
    }

    @Override void calibrate() throws InterruptedException {
        Turret turret = new Turret(hardwareMap, store, battery);
        try {
            turret.teachingMode(true);
            if (!confirm("把云台轻轻摆到最右端，停稳后 A。")) return;
            homeTick = turret.rawTicks();
            if (!turret.calibrated() || confirm("A 重测方向、正前和行程；B 沿用有效几何并继续电压校准。")) {
                if (!teach(turret)) {
                    geometry = "教学未完成";
                    return;
                }
                geometry = "已重测";
                turret = new Turret(hardwareMap, store, battery);
            } else {
                geometry = "沿用已保存";
            }
            if (!turret.calibrated() || !opModeIsActive()) {
                telemetry.log().add("需要完成方向、正前方与限位标定才能自动上电。");
                return;
            }
            turret.homeAtRight(homeTick);
            if (!center(turret)) return;
            while (!new TurretQuickIdentification(this, turret).run()) {
                plant = "未通过，保留上次值";
                if (!opModeIsActive() || motionAborted()
                        || !confirm("12V 动力学未通过。A 回中后重试，B 结束。") || !center(turret)) return;
            }
            plant = "已保存";
            TurretFrictionCalibration measurement = new TurretFrictionCalibration(this);
            friction = measurement.run(turret);
            if (!opModeIsActive() || motionAborted() || measurement.feedbackLost()) return;

            turret = new Turret(hardwareMap, store, battery);
            turret.homeAtRight(homeTick);
            turret.teachingMode(false);
            if (!turret.encoderReady() || !turret.motorResponding()) return;
            if (!confirm("轮子落地、清空周围并保持底盘静止。A 开始联动、PID 与回转，流程会自动前进。")) return;
            PinpointPose pose = new PinpointPose(hardwareMap, battery);
            if (!initializePinpoint(pose)) {
                coupling = "Pinpoint 航向未就绪";
                return;
            }
            Drive drive = new Drive(hardwareMap);
            try {
                double[] startingGains = turret.pidGains();
                TurretPlant model = TurretPlant.load(store);
                turret.setPidGains(model.pidSeed(model.baseline()));
                coupling = new TurretYawCalibration(this).run(drive, pose, turret)
                        ? "双向验收并保存" : "未通过，沿用上次值";
                if (!opModeIsActive() || motionAborted() || !turret.motorResponding()) return;
                turret.setPidGains(startingGains);
                tuning = new TurretPidTuning(this);
                pid = tuning.tune(turret, drive, pose) ? "已验收并保存" : "未验收，沿用上次值";
            } finally {
                drive.stop();
            }
            if ("已重测".equals(geometry) && opModeIsActive() && !motionAborted()) verifyEncoderReturn(turret);
        } finally {
            turret.teachingMode(false);
        }
    }

    @Override void showResults() {
        telemetry.addData("行程 / 12V 动力学 / 摩擦", "%s / %s / %s", geometry, plant, friction);
        telemetry.addData("联动 / PID", "%s / %s", coupling, pid);
        if (tuning != null) tuning.showResults();
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
        while (confirm("将云台移到行程中间并移开手。A 连续测 12V 短脉冲、刹车和六点摩擦。", () -> {
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
