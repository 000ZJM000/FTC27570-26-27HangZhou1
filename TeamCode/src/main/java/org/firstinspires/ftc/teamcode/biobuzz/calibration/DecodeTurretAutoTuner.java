package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.biobuzz.config.CalibrationStore;
import org.firstinspires.ftc.teamcode.biobuzz.config.RobotConfig;
import org.firstinspires.ftc.teamcode.biobuzz.core.BatteryVoltage;
import org.firstinspires.ftc.teamcode.biobuzz.core.ControlLoop;
import org.firstinspires.ftc.teamcode.biobuzz.core.Maths;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.DecodeTurret;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.DecodeTurretConfig;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.RearStatusLed;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Ported DECODE TurretFullAutoTuner adapted to BIOBUZZ wiring, shared encoder, and asymmetric travel limits.
 * Measures kS, kV, kA, and predictive braking (kLin, kQuad), normalizes to 12 V, saves to CalibrationStore,
 * and supports optional closed-loop step verification after completion.
 */
@Config
@TeleOp(name = "02B 云台 DECODE 全自动调参 (FF+PB)", group = "BIOBUZZ 自测")
public final class DecodeTurretAutoTuner extends LinearOpMode {
    public static double MAX_SAFE_ANGLE = 170.0;
    public static double BRAKE_BUFFER_ANGLE = 30.0;
    public static double RETURN_POWER = 0.30;
    public static int REST_TIME_MS = 500;

    public static double[] KV_TEST_POWERS = {0.3, 0.45, 0.6, 0.75, 0.8, 0.9, 1.0};
    public static double[] KA_TEST_POWERS = {0.35, 0.5, 0.7, 0.9, 1.0};
    public static double[] PB_TEST_POWERS = {1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3};
    public static double PB_BRAKING_POWER = 0.30;
    public static int PB_SPIN_TIME_MS = 3000;
    public static double VEL_FILTER_ALPHA = 0.8;
    public static double ACCEL_FILTER_ALPHA = 0.8;
    public static double NOMINAL_VOLTAGE = 12.0;

    private enum State {
        INIT_CENTER,
        KS_RUN, KS_RETURN, KS_WAIT,
        KV_RETURN, KV_WAIT, KV_RUN,
        KA_RUN, KA_RETURN, KA_WAIT,
        PB_RETURN, PB_WAIT, PB_RUN, PB_BRAKE,
        CALCULATE, DONE
    }

    @Override public void runOpMode() throws InterruptedException {
        CalibrationStore store = new CalibrationStore(hardwareMap.appContext);
        BatteryVoltage battery = new BatteryVoltage(hardwareMap);
        RearStatusLed lights = RearStatusLed.open(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, RobotConfig.TURRET_MOTOR);
        DcMotorEx encoder = hardwareMap.get(DcMotorEx.class, RobotConfig.TURRET_ENCODER);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        double[] geometry = store.read(CalibrationStore.HOME, 0, 0, 0, 0);
        boolean geometryValid = Math.abs(geometry[0]) == 1 && Math.abs(geometry[1]) == 1
                && geometry[2] >= Math.PI / 2 && geometry[2] <= Maths.TURN
                && geometry[3] >= 0 && geometry[3] <= geometry[2] + Math.toRadians(10);

        try {
            lights.show(geometryValid ? RearStatusLed.State.INIT : RearStatusLed.State.FAULT);
            while (opModeInInit()) {
                battery.update(Maths.time());
                telemetry.addLine("DECODE 云台全自动调参 (FF + PB + 12V 补偿)");
                if (!geometryValid) {
                    telemetry.addLine("<font color='#ff4444'>尚未完成 02 几何教学（方向/正前/行程），请先运行 02 云台全套！</font>");
                } else {
                    telemetry.addData("已加载行程", "跨度 %.1f° | 正前 %.1f° | 编码/电机极性 %+.0f/%+.0f",
                            Math.toDegrees(geometry[2]), Math.toDegrees(geometry[3]), geometry[0], geometry[1]);
                    telemetry.addLine("请先手动将云台轻摆到最右端标记，再按 START。");
                }
                telemetry.addData("电池电压", battery.measured() ? String.format(Locale.US, "%.2f V", battery.volts()) : "等待读数");
                telemetry.update();
                sleep(25);
            }
            if (!opModeIsActive()) return;
            if (!geometryValid) {
                telemetry.log().add("02B 未启动：缺少有效的 02 几何标定，请先运行 02 云台全套。");
                return;
            }

            try (ControlLoop loop = new ControlLoop(hardwareMap)) {
                runTuning(loop, store, battery, lights, motor, encoder, geometry);
            }
        } finally {
            ControlLoop.safeStop("02B云台", () -> {
                motor.setPower(0);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            });
            ControlLoop.safeStop("尾灯", lights::stop);
        }
    }

    private void runTuning(ControlLoop loop, CalibrationStore store, BatteryVoltage battery, RearStatusLed lights,
                           DcMotorEx motor, DcMotorEx encoder, double[] geometry) throws InterruptedException {
        double encoderSign = geometry[0];
        double driveSign = geometry[1];
        double spanDeg = Math.toDegrees(geometry[2]);

        lights.show(RearStatusLed.State.CAL_WAIT);
        if (!confirmRightStop(loop, battery, encoder, encoderSign)) return;

        int homeTick;
        try {
            homeTick = readRawTicks(encoder);
        } catch (RuntimeException unavailable) {
            lights.show(RearStatusLed.State.FAULT);
            telemetry.log().add("02B 读取右端编码器失败：%s", unavailable.getMessage());
            return;
        }

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lights.show(RearStatusLed.State.CAL_RUNNING);

        double centerDeg = spanDeg * 0.5;
        double marginDeg = Math.toDegrees(RobotConfig.TURRET_MARGIN);
        double safeHalfSpanDeg = Math.min(MAX_SAFE_ANGLE, Math.max(25.0, centerDeg - (marginDeg + 8.0)));
        double effectiveBrakeBuffer = Math.min(BRAKE_BUFFER_ANGLE, safeHalfSpanDeg * 0.35);
        double limitThreshold = safeHalfSpanDeg - effectiveBrakeBuffer;

        ElapsedTime stateTimer = new ElapsedTime();
        ElapsedTime voltageTimer = new ElapsedTime();

        State state = State.INIT_CENTER;
        stateTimer.reset();
        voltageTimer.reset();

        double lastTime = Maths.time();
        double lastAngle = 0.0;
        double filteredVel = 0.0;
        double lastFilteredVel = 0.0;
        double filteredAccel = 0.0;
        boolean initialized = false;

        double currentVoltage = battery.volts();
        double sumVoltage = 0.0;
        int voltageSamples = 0;

        int iteration = 0;
        int currentDirection = 1;
        double currentTestPower = 0.0;
        double pbStartBrakeAngle = 0.0;
        double pbMeasuredBrakeVel = 0.0;
        boolean kvStarted = false;
        boolean pbStarted = false;
        int pbDiscarded = 0;

        List<Double> ksSamples = new ArrayList<>();
        List<double[]> kvData = new ArrayList<>();
        List<double[]> kaData = new ArrayList<>();
        List<double[]> pbData = new ArrayList<>();

        double rawKs = 0, rawKv = 0, rawKa = 0, rawKLin = 0, rawKQuad = 0;
        double compKs = 0, compKv = 0, compKa = 0, compKLin = 0, compKQuad = 0;
        double avgTestVoltage = NOMINAL_VOLTAGE;
        boolean saved = false;
        boolean aborted = false;
        String abortReason = "";

        while (opModeIsActive() && state != State.DONE) {
            if (gamepad1.b) {
                aborted = true;
                abortReason = "用户按 B 中止，已断电且未修改原参数";
                break;
            }

            double now = loop.beginFrame();
            battery.update(now);

            double dt = now - lastTime;
            lastTime = now;
            if (dt <= 0.00001) {
                long rem = loop.remainingMillis(now);
                if (rem > 0) sleep(rem);
                continue;
            }

            double currentAngle;
            try {
                currentAngle = readAngleDeg(encoder, encoderSign, homeTick);
            } catch (RuntimeException unavailable) {
                aborted = true;
                abortReason = "编码器读取中断，安全停机";
                break;
            }

            if (dt > 0.25) {
                aborted = true;
                abortReason = String.format(Locale.US, "循环卡顿 dt=%.0f ms，安全停机", dt * 1000.0);
                break;
            }

            if (voltageTimer.milliseconds() > 250 && state != State.CALCULATE) {
                currentVoltage = battery.volts();
                if (battery.measured()) {
                    sumVoltage += currentVoltage;
                    voltageSamples++;
                }
                voltageTimer.reset();
            }

            if (!initialized) {
                lastAngle = currentAngle;
                filteredVel = 0.0;
                lastFilteredVel = 0.0;
                filteredAccel = 0.0;
                initialized = true;
                long rem = loop.remainingMillis(now);
                if (rem > 0) sleep(rem);
                continue;
            }

            double rawVel = (currentAngle - lastAngle) / dt;
            if (Math.abs(rawVel) > 2500.0) {
                aborted = true;
                abortReason = String.format(Locale.US, "检测到编码器异常跳变 %.0f deg/s，安全停机", rawVel);
                break;
            }
            double velAlpha = Maths.clip(VEL_FILTER_ALPHA, 0.05, 1.0);
            double accAlpha = Maths.clip(ACCEL_FILTER_ALPHA, 0.05, 1.0);
            filteredVel = velAlpha * rawVel + (1.0 - velAlpha) * filteredVel;
            double rawAccel = (filteredVel - lastFilteredVel) / dt;
            filteredAccel = accAlpha * rawAccel + (1.0 - accAlpha) * filteredAccel;
            lastAngle = currentAngle;
            lastFilteredVel = filteredVel;

            double relAngle = currentAngle - centerDeg;
            boolean hitSafetyLimit = (relAngle >= limitThreshold && currentDirection == 1)
                    || (relAngle <= -limitThreshold && currentDirection == -1);

            if (Math.abs(relAngle) > safeHalfSpanDeg + 6.0 && !isReturningState(state) && state != State.CALCULATE) {
                motor.setPower(0);
                aborted = true;
                abortReason = String.format(Locale.US, "超出安全边界 %.1f°（中心偏差 %+.1f°），安全停机",
                        safeHalfSpanDeg + 6.0, relAngle);
                break;
            }

            switch (state) {
                case INIT_CENTER:
                    double initMovePower = Math.max(RETURN_POWER, 0.30);
                    if (moveToTargetDeg(motor, driveSign, currentAngle, centerDeg, 3.0, initMovePower)
                            || stateTimer.milliseconds() > 4000) {
                        motor.setPower(0);
                        state = State.KS_RUN;
                        stateTimer.reset();
                    }
                    break;

                case KS_RUN:
                    if (iteration >= 2) {
                        double sum = 0.0;
                        for (Double v : ksSamples) sum += v;
                        rawKs = ksSamples.isEmpty() ? 0.0 : (sum / ksSamples.size());
                        iteration = 0;
                        kvStarted = false;
                        state = State.KV_RETURN;
                        stateTimer.reset();
                        break;
                    }
                    currentDirection = (iteration % 2 == 0) ? 1 : -1;
                    currentTestPower = Math.min(0.65, currentTestPower + 0.1 * dt);
                    writeLogicalPower(motor, driveSign, currentTestPower * currentDirection);
                    if (Math.abs(filteredVel) > 2.0 || hitSafetyLimit || currentTestPower >= 0.65) {
                        if (Math.abs(filteredVel) > 2.0) ksSamples.add(currentTestPower);
                        currentTestPower = 0.0;
                        motor.setPower(0);
                        state = State.KS_RETURN;
                        stateTimer.reset();
                    }
                    break;

                case KS_RETURN:
                    if (moveToTargetDeg(motor, driveSign, currentAngle, centerDeg, 4.0, returnPower(rawKs))
                            || stateTimer.milliseconds() > 3000) {
                        motor.setPower(0);
                        state = State.KS_WAIT;
                        stateTimer.reset();
                    }
                    break;

                case KS_WAIT:
                    motor.setPower(0);
                    if (stateTimer.milliseconds() >= REST_TIME_MS) {
                        iteration++;
                        state = State.KS_RUN;
                        stateTimer.reset();
                    }
                    break;

                case KV_RETURN: {
                    int nextIter = kvStarted ? (iteration + 1) : 0;
                    double targetDeg = centerDeg;
                    if (nextIter < KV_TEST_POWERS.length * 2) {
                        int nextDir = (nextIter % 2 == 0) ? 1 : -1;
                        targetDeg = centerDeg - 0.40 * limitThreshold * nextDir;
                    }
                    if (moveToTargetDeg(motor, driveSign, currentAngle, targetDeg, 3.0, returnPower(rawKs))
                            || stateTimer.milliseconds() > 3500) {
                        motor.setPower(0);
                        state = State.KV_WAIT;
                        stateTimer.reset();
                    }
                    break;
                }

                case KV_WAIT:
                    motor.setPower(0);
                    if (stateTimer.milliseconds() >= REST_TIME_MS) {
                        if (!kvStarted) {
                            kvStarted = true;
                            iteration = 0;
                        } else {
                            iteration++;
                        }
                        if (iteration >= KV_TEST_POWERS.length * 2) {
                            iteration = 0;
                            rawKv = calculateLeastSquaresSlope(kvData);
                            state = State.KA_RETURN;
                        } else {
                            currentDirection = (iteration % 2 == 0) ? 1 : -1;
                            state = State.KV_RUN;
                        }
                        stateTimer.reset();
                    }
                    break;

                case KV_RUN: {
                    int kvPowerIndex = iteration / 2;
                    currentDirection = (iteration % 2 == 0) ? 1 : -1;
                    double kvTestPower = KV_TEST_POWERS[kvPowerIndex];
                    writeLogicalPower(motor, driveSign, kvTestPower * currentDirection);
                    if (stateTimer.milliseconds() > 700 || hitSafetyLimit) {
                        if (Math.abs(filteredVel) > 3.0) {
                            double netPower = Math.max(0.0, kvTestPower - rawKs);
                            kvData.add(new double[]{Math.abs(filteredVel), netPower});
                        }
                        motor.setPower(0);
                        state = State.KV_RETURN;
                        stateTimer.reset();
                    }
                    break;
                }

                case KA_RETURN:
                    if (moveToTargetDeg(motor, driveSign, currentAngle, centerDeg, 3.5, returnPower(rawKs))
                            || stateTimer.milliseconds() > 3000) {
                        motor.setPower(0);
                        state = State.KA_WAIT;
                        stateTimer.reset();
                    }
                    break;

                case KA_WAIT:
                    motor.setPower(0);
                    if (stateTimer.milliseconds() >= REST_TIME_MS) {
                        if (iteration >= KA_TEST_POWERS.length * 2) {
                            iteration = 0;
                            pbStarted = false;
                            state = State.PB_RETURN;
                        } else {
                            currentDirection = (iteration % 2 == 0) ? 1 : -1;
                            state = State.KA_RUN;
                        }
                        stateTimer.reset();
                    }
                    break;

                case KA_RUN: {
                    if (iteration >= KA_TEST_POWERS.length * 2) {
                        iteration = 0;
                        pbStarted = false;
                        state = State.PB_RETURN;
                        stateTimer.reset();
                        break;
                    }
                    int kaPowerIndex = iteration / 2;
                    currentDirection = (iteration % 2 == 0) ? 1 : -1;
                    double kaTestPower = KA_TEST_POWERS[kaPowerIndex];
                    writeLogicalPower(motor, driveSign, kaTestPower * currentDirection);
                    if (stateTimer.milliseconds() < 300 && Math.abs(filteredAccel) > 10.0) {
                        double netPower = Math.max(0.0, kaTestPower - rawKs - (rawKv * Math.abs(filteredVel)));
                        kaData.add(new double[]{Math.abs(filteredAccel), netPower});
                    }
                    if (stateTimer.milliseconds() > 400 || hitSafetyLimit) {
                        motor.setPower(0);
                        iteration++;
                        state = State.KA_RETURN;
                        stateTimer.reset();
                    }
                    break;
                }

                case PB_RETURN: {
                    int nextIter = pbStarted ? (iteration + 1) : 0;
                    double targetDeg = centerDeg;
                    if (nextIter < PB_TEST_POWERS.length * 2) {
                        int nextDir = (nextIter % 2 == 0) ? 1 : -1;
                        targetDeg = centerDeg - 0.40 * limitThreshold * nextDir;
                    }
                    if (moveToTargetDeg(motor, driveSign, currentAngle, targetDeg, 3.0, returnPower(rawKs))
                            || stateTimer.milliseconds() > 3500) {
                        motor.setPower(0);
                        state = State.PB_WAIT;
                        stateTimer.reset();
                    }
                    break;
                }

                case PB_WAIT:
                    motor.setPower(0);
                    if (stateTimer.milliseconds() >= REST_TIME_MS) {
                        if (!pbStarted) {
                            pbStarted = true;
                            iteration = 0;
                        } else {
                            iteration++;
                        }
                        if (iteration >= PB_TEST_POWERS.length * 2) {
                            state = State.CALCULATE;
                        } else {
                            currentDirection = (iteration % 2 == 0) ? 1 : -1;
                            state = State.PB_RUN;
                        }
                        stateTimer.reset();
                    }
                    break;

                case PB_RUN: {
                    int pbPowerIndex = iteration / 2;
                    currentDirection = (iteration % 2 == 0) ? 1 : -1;
                    double pbPower = PB_TEST_POWERS[pbPowerIndex] * currentDirection;
                    writeLogicalPower(motor, driveSign, pbPower);
                    if (stateTimer.milliseconds() >= PB_SPIN_TIME_MS || hitSafetyLimit) {
                        pbMeasuredBrakeVel = filteredVel;
                        pbStartBrakeAngle = currentAngle;
                        writeLogicalPower(motor, driveSign, -currentDirection * PB_BRAKING_POWER);
                        state = State.PB_BRAKE;
                        stateTimer.reset();
                    }
                    break;
                }

                case PB_BRAKE:
                    if (Math.abs(relAngle) >= safeHalfSpanDeg + 4.0) {
                        motor.setPower(0);
                        pbDiscarded++;
                        RobotLog.ww("BioBuzzTurret",
                                "02B PB_BRAKE hit guard (relAngle=%+.1f deg), sample discarded", relAngle);
                        state = State.PB_RETURN;
                        stateTimer.reset();
                    } else if ((Math.signum(filteredVel) != currentDirection || Math.abs(filteredVel) < 2.0)
                            && stateTimer.milliseconds() > 150 || stateTimer.milliseconds() > 3000) {
                        motor.setPower(0);
                        double brakingDistance = Math.abs(currentAngle - pbStartBrakeAngle);
                        double absVel = Math.abs(pbMeasuredBrakeVel);
                        if (absVel >= 5.0) {
                            pbData.add(new double[]{absVel, brakingDistance});
                        }
                        state = State.PB_RETURN;
                        stateTimer.reset();
                    }
                    break;

                case CALCULATE:
                    motor.setPower(0);
                    avgTestVoltage = voltageSamples > 0 ? (sumVoltage / voltageSamples) : battery.volts();
                    if (!Double.isFinite(avgTestVoltage) || avgTestVoltage < 8.0) avgTestVoltage = NOMINAL_VOLTAGE;

                    rawKv = calculateLeastSquaresSlope(kvData);
                    rawKa = calculateLeastSquaresSlope(kaData);
                    if (rawKa < 0) rawKa = 0.0;
                    double[] pbCoeffs = calculatePbCoefficients(pbData);
                    rawKLin = pbCoeffs[0];
                    rawKQuad = pbCoeffs[1];

                    double scale = avgTestVoltage / NOMINAL_VOLTAGE;
                    compKs = Math.max(0.0, rawKs * scale);
                    compKv = Math.max(0.0, rawKv * scale);
                    compKa = Math.max(0.0, rawKa * scale);
                    compKLin = Math.max(0.0, rawKLin * scale);
                    compKQuad = Math.max(0.0, rawKQuad * scale);

                    if (Maths.finite(compKs, compKv, compKa, compKLin, compKQuad) && compKv > 0.0) {
                        saved = store.save(CalibrationStore.DECODE_TURRET_PARAMS,
                                compKs, compKv, compKa, compKLin, compKQuad, NOMINAL_VOLTAGE);
                        if (saved) {
                            DecodeTurretConfig.TURRET_kS = compKs;
                            DecodeTurretConfig.TURRET_kV = compKv;
                            DecodeTurretConfig.TURRET_kA = compKa;
                            DecodeTurretConfig.TURRET_kLinearBraking = compKLin;
                            DecodeTurretConfig.TURRET_kQuadraticFriction = compKQuad;
                            DecodeTurretConfig.TUNING_VOLTAGE = NOMINAL_VOLTAGE;
                        }
                    }
                    RobotLog.ii("BioBuzzTurret",
                            "02B tuned (saved=%b, V=%.2f): kS=%.6f kV=%.6f kA=%.6f kLin=%.6f kQuad=%.6f",
                            saved, avgTestVoltage, compKs, compKv, compKa, compKLin, compKQuad);
                    state = State.DONE;
                    break;

                default:
                    break;
            }

            if (state != State.DONE) {
                telemetry.addData("阶段 / 状态", "%s / %s", state.name().split("_")[0], state.name());
                telemetry.addData("行程 / 安全半幅 / 阈值", "%.1f° / ±%.1f° / ±%.1f°",
                        spanDeg, safeHalfSpanDeg, limitThreshold);
                telemetry.addData("当前角度 / 相对中心", "%.1f° / %+.1f°", currentAngle, relAngle);
                telemetry.addData("角速度 / 加速度", "%+.1f°/s / %+.0f°/s²", filteredVel, filteredAccel);
                telemetry.addData("电压", "%.2f V", currentVoltage);
                if (state.name().startsWith("KV")) telemetry.addData("进度", "%d / %d", Math.min(iteration + 1, KV_TEST_POWERS.length * 2), KV_TEST_POWERS.length * 2);
                else if (state.name().startsWith("KA")) telemetry.addData("进度", "%d / %d", Math.min(iteration + 1, KA_TEST_POWERS.length * 2), KA_TEST_POWERS.length * 2);
                else if (state.name().startsWith("PB")) telemetry.addData("进度", "%d / %d（丢弃 %d）", Math.min(iteration + 1, PB_TEST_POWERS.length * 2), PB_TEST_POWERS.length * 2, pbDiscarded);
                telemetry.addLine("按手柄 1 B 立即急停并放弃保存");
                telemetry.update();
            }

            long rem = loop.remainingMillis(now);
            if (rem > 0) sleep(rem);
        }

        motor.setPower(0);
        if (aborted || !opModeIsActive()) {
            lights.show(RearStatusLed.State.FAULT);
            if (opModeIsActive()) {
                while (opModeIsActive()) {
                    telemetry.addLine("02B 自动调参已中止（原有参数未受影响）");
                    telemetry.addData("原因", abortReason);
                    telemetry.addLine("请按 STOP 退出。");
                    telemetry.update();
                    sleep(100);
                }
            }
            return;
        }

        lights.show(saved ? RearStatusLed.State.CAL_DONE : RearStatusLed.State.FAULT);
        String verifySummary = "未运行闭环验证";
        boolean lastA = gamepad1.a;
        while (opModeIsActive()) {
            boolean currA = gamepad1.a;
            if (saved && currA && !lastA) {
                while (opModeIsActive() && gamepad1.a) sleep(20);
                lights.show(RearStatusLed.State.CAL_RUNNING);
                verifySummary = runStepVerification(loop, store, battery, homeTick, centerDeg, safeHalfSpanDeg);
                lights.show(RearStatusLed.State.CAL_DONE);
            }
            lastA = currA;

            telemetry.addLine(saved ? "=== 02B DECODE 调参完成并已保存 (12V 归一化) ==="
                    : "=== 02B 拟合异常或写入失败（未覆盖原参数） ===");
            telemetry.addData("采样点数", "kS:%d | kV:%d | kA:%d | PB:%d (丢弃 %d)",
                    ksSamples.size(), kvData.size(), kaData.size(), pbData.size(), pbDiscarded);
            telemetry.addData("测试均压", "%.2f V → 归一化 %.1f V", avgTestVoltage, NOMINAL_VOLTAGE);
            telemetry.addData("TURRET_kS", "%.6f", compKs);
            telemetry.addData("TURRET_kV", "%.6f", compKv);
            telemetry.addData("TURRET_kA", "%.6f", compKa);
            telemetry.addData("TURRET_kLinearBraking", "%.6f", compKLin);
            telemetry.addData("TURRET_kQuadraticFriction", "%.6f", compKQuad);
            telemetry.addData("闭环阶跃验证", verifySummary);
            if (saved) telemetry.addLine("按手柄 1 A 运行 4 步闭环阶跃验证，或按 STOP 退出。");
            else telemetry.addLine("按 STOP 退出。");
            telemetry.update();
            sleep(50);
        }
    }

    private String runStepVerification(ControlLoop loop, CalibrationStore store, BatteryVoltage battery,
                                         int homeTick, double centerDeg, double safeHalfSpanDeg)
            throws InterruptedException {
        DecodeTurret turret = new DecodeTurret(hardwareMap, store, battery);
        turret.homeAtRight(homeTick);
        double[] offsetsDeg = {0.0, Math.min(45.0, safeHalfSpanDeg), -Math.min(45.0, safeHalfSpanDeg),
                Math.min(110.0, safeHalfSpanDeg), 0.0};
        double maxEndErr = 0.0;
        double peakPower = 0.0;
        try {
            double prev = Maths.time();
            for (int step = 0; step < offsetsDeg.length && opModeIsActive(); step++) {
                double targetDeg = centerDeg + offsetsDeg[step];
                double targetRad = Math.toRadians(targetDeg);
                double endTime = Maths.time() + 1.6;
                while (opModeIsActive() && Maths.time() < endTime) {
                    if (gamepad1.b) {
                        turret.stop();
                        return "闭环验证已由 B 中止";
                    }
                    double now = loop.beginFrame();
                    double dt = Maths.clip(now - prev, 0.001, 0.25);
                    prev = now;
                    turret.sample(dt);
                    if (!turret.encoderReady() || !turret.motorResponding()) {
                        turret.stop();
                        return "闭环验证中断：编码器或电机不可用";
                    }
                    turret.controlAngleRad(targetRad, dt);
                    peakPower = Math.max(peakPower, Math.abs(turret.output()));
                    telemetry.addData("闭环验证步", "%d / %d（目标 %.1f°）", step + 1, offsetsDeg.length, targetDeg);
                    telemetry.addData("当前角 / 误差 / 功率", "%.1f° / %+.2f° / %+.2f",
                            Math.toDegrees(turret.angle), Math.toDegrees(targetRad - turret.angle), turret.output());
                    telemetry.addLine("按 B 中止验证");
                    telemetry.update();
                    long rem = loop.remainingMillis(now);
                    if (rem > 0) sleep(rem);
                }
                if (step > 0) {
                    maxEndErr = Math.max(maxEndErr, Math.abs(Math.toDegrees(targetRad - turret.angle)));
                }
            }
            return String.format(Locale.US, "通过，最大稳态误差 %.2f°，峰值功率 %.2f", maxEndErr, peakPower);
        } finally {
            turret.stop();
        }
    }

    private boolean confirmRightStop(ControlLoop loop, BatteryVoltage battery, DcMotorEx encoder, double encoderSign)
            throws InterruptedException {
        while (opModeIsActive() && (gamepad1.a || gamepad1.b)) sleep(20);
        int baseTick = readRawTicks(encoder);
        while (opModeIsActive()) {
            loop.beginFrame();
            battery.update(Maths.time());
            double deltaDeg = Math.toDegrees(encoderSign * (readRawTicks(encoder) - baseTick) / RobotConfig.TICKS_PER_TURRET_RADIAN);
            telemetry.addLine("步骤 1：将云台轻摆到最右端机械标记并松手");
            telemetry.addData("相对 START 位移", "%+.1f°", deltaDeg);
            telemetry.addLine("按手柄 1 A 记录右端零位并开始自动调参，按 B 退出");
            telemetry.update();
            if (gamepad1.b) {
                while (opModeIsActive() && gamepad1.b) sleep(20);
                return false;
            }
            if (gamepad1.a) {
                while (opModeIsActive() && gamepad1.a) sleep(20);
                return true;
            }
            sleep(25);
        }
        return false;
    }

    private static int readRawTicks(DcMotorEx encoder) {
        return encoder.getController().getMotorCurrentPosition(encoder.getPortNumber());
    }

    private static double readAngleDeg(DcMotorEx encoder, double encoderSign, int homeTick) {
        return Math.toDegrees(encoderSign * (readRawTicks(encoder) - homeTick) / RobotConfig.TICKS_PER_TURRET_RADIAN);
    }

    private static boolean isReturningState(State s) {
        return s == State.INIT_CENTER || s == State.KS_RETURN || s == State.KV_RETURN
                || s == State.KA_RETURN || s == State.PB_RETURN || s == State.PB_BRAKE;
    }

    private static double returnPower(double rawKs) {
        return Math.max(RETURN_POWER, Math.min(0.45, rawKs + 0.08));
    }

    private static boolean moveToTargetDeg(DcMotorEx motor, double driveSign,
                                           double currentAngle, double targetAngle,
                                           double toleranceDeg, double power) {
        double error = targetAngle - currentAngle;
        if (Math.abs(error) > toleranceDeg) {
            writeLogicalPower(motor, driveSign, Math.signum(error) * power);
            return false;
        }
        motor.setPower(0);
        return true;
    }

    private static void writeLogicalPower(DcMotorEx motor, double driveSign, double logicalPower) {
        motor.setPower(Maths.clip(logicalPower, -1.0, 1.0) * driveSign);
    }

    private static double calculateLeastSquaresSlope(List<double[]> data) {
        if (data.isEmpty()) return 0.0;
        double sumXY = 0.0, sumXX = 0.0;
        for (double[] point : data) {
            sumXY += point[0] * point[1];
            sumXX += point[0] * point[0];
        }
        return sumXX == 0.0 ? 0.0 : sumXY / sumXX;
    }

    private static double[] calculatePbCoefficients(List<double[]> pbData) {
        double sumX = 0.0, sumY = 0.0, sumXY = 0.0, sumXX = 0.0;
        int n = 0;
        for (double[] point : pbData) {
            double v = point[0];
            double d = point[1];
            if (v < 5.0) continue;
            double x = v;
            double y = d / v;
            sumX += x;
            sumY += y;
            sumXY += x * y;
            sumXX += x * x;
            n++;
        }
        double kLin = 0.0;
        double kQuad = 0.0;
        if (n > 1) {
            double denominator = (n * sumXX) - (sumX * sumX);
            if (Math.abs(denominator) > 1e-9) {
                kQuad = ((n * sumXY) - (sumX * sumY)) / denominator;
                kLin = ((sumY * sumXX) - (sumX * sumXY)) / denominator;
                if (kQuad < 0.0) kQuad = 0.0;
                if (kLin < 0.0) kLin = 0.0;
            } else if (sumX > 1e-6) {
                kLin = Math.max(0.0, sumY / n);
            }
        }
        return new double[]{kLin, kQuad};
    }
}
