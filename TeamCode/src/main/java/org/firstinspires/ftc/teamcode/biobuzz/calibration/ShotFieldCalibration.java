package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.biobuzz.config.CalibrationStore;
import org.firstinspires.ftc.teamcode.biobuzz.config.RobotConfig;
import org.firstinspires.ftc.teamcode.biobuzz.core.BatteryVoltage;
import org.firstinspires.ftc.teamcode.biobuzz.core.ControlLoop;
import org.firstinspires.ftc.teamcode.biobuzz.core.Maths;
import org.firstinspires.ftc.teamcode.biobuzz.shooting.Aiming.FieldTargets;
import org.firstinspires.ftc.teamcode.biobuzz.shooting.Aiming.ShotSequence;
import org.firstinspires.ftc.teamcode.biobuzz.shooting.ShotMap;
import org.firstinspires.ftc.teamcode.biobuzz.shooting.ShotVectorField;
import org.firstinspires.ftc.teamcode.biobuzz.shooting.ShotVectorField.PointData;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Intake;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.RearStatusLed;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Turret;

@TeleOp(name = "06 半场射击校准·数据", group = "BIOBUZZ 自测")
public final class ShotFieldCalibration extends LinearOpMode {
    private static final double TRIGGER_THRESHOLD = 0.10;
    private static final double SPIN_UP_TIMEOUT = 6.0;
    private static final double RPM_SETTLE_SECONDS = 0.12;
    private static final double SPINUP_TOLERANCE = 0.035;
    private static final int COMPACT_ROWS_PER_PAGE = 10;
    private static final int COMPACT_LINE_WIDTH = 44;
    private static final int POINTS_PER_ROW = 3;
    private static final int DETAIL_POINTS_PER_PAGE = 3;

    private Turret turret;
    private Flywheel flywheel;
    private Intake intake;
    private ShotSequence shot;
    private CalibrationStore store;
    private BatteryVoltage battery;
    private Servo pitch;
    private RearStatusLed lights;
    private boolean rightHalf;
    private double trialVoltage, trialPitch;
    private double holdTarget = Double.NaN, holdUntil;
    private boolean previewMove;
    private ShotVectorField field;
    private boolean brake;
    private int column, row;
    private double targetRpm, maxRpm;
    private List<ShotVectorField.Refinement> refinements;
    private int refinementIndex;
    private double coarseRpm;

    private boolean shooting, firedThisPress, cancelWaitRelease, blockerOpen;
    private boolean rpmLockEnabled = false; // 默认高速直射，与 Biobuzz角落出发 默认一致
    private boolean hotSpinIdle = false;    // 默认松手停转（安全），副手 Y 可切持续热转（帧级同手动热态）
    private double lockedTurretAngle, spinupDeadline, feedSeconds, shotRpmAtRelease = Double.NaN;
    private double shotMinRpm = Double.NaN, shotEndRpm = Double.NaN;
    private double loopHz;
    private ShotVectorField.Vector trial, pending;
    private String status = "选格、摆位、调速后扣 RT";

    private boolean dataViewMode, showAllPoints, detailView;
    private int dataPage;
    private double confirmClearAllUntil = Double.NEGATIVE_INFINITY;
    private String modelStatus = "";
    private final List<PointData> allPoints = new ArrayList<>();
    private final List<PointData> recordedPoints = new ArrayList<>();
    private final List<String> allCompactRows = new ArrayList<>();
    private final List<String> recordedCompactRows = new ArrayList<>();

    @Override public void runOpMode() {
        store = new CalibrationStore(hardwareMap.appContext);
        battery = new BatteryVoltage(hardwareMap);
        lights = RearStatusLed.open(hardwareMap);
        try (ControlLoop loop = new ControlLoop(hardwareMap)) {
            turret = new Turret(hardwareMap, store, battery);
            flywheel = new Flywheel(hardwareMap, store, battery);
            intake = new Intake(hardwareMap);
            shot = new ShotSequence(turret, flywheel, intake, true);
            pitch = hardwareMap.tryGet(Servo.class, RobotConfig.PITCH_SERVO);
            if (pitch != null) pitch.setPosition(RobotConfig.ST_HOME);

            turret.teachingMode(true);
            lights.show(RearStatusLed.State.INIT);
            refreshDataView();
            telemetry.setMsTransmissionInterval(150);

            Gamepad initPrevious = new Gamepad();
            Gamepad initCurrent = new Gamepad();
            initPrevious.copy(gamepad1);
            while (opModeInInit()) {
                double now = loop.beginFrame();
                battery.update(now);
                initCurrent.copy(gamepad1);
                if (!initCurrent.start) {
                    if (initCurrent.dpad_left) rightHalf = false;
                    if (initCurrent.dpad_right) rightHalf = true;
                }
                handleFileShortcuts(initCurrent, initPrevious, now);
                Gamepad swap = initPrevious;
                initPrevious = initCurrent;
                initCurrent = swap;

                telemetry.addLine("方向键左 / 右选半场。将云台轻摆到最右端后按 START。");
                telemetry.addData("3 × 6", rightHalf ? "右半场（右坐标 3–6 格）" : "左半场（右坐标 0–3 格）");
                telemetry.addData("射界文件", jsonFilesStatus());
                if (store.turretStale()) {
                    telemetry.addLine("云台参数已过期（" + store.turretStaleSummary() + "），建议先重跑对应校准");
                }
                if (flywheel.rpmCalibrated()) {
                    double[] gains = flywheel.pidGains();
                    telemetry.addData("飞轮 PID", "%s  %.3f / %.3f / %.3f",
                            flywheel.pidSource(), gains[0], gains[1], gains[2]);
                }
                telemetry.addLine("START+X 一键删除 records/model JSON | START后 L3 切数据查看");
                telemetry.addLine(status);
                if (!turret.calibrated()) telemetry.addLine("试射前先完成 02 云台行程和正前方向标定。");
                if (!flywheel.rpmCalibrated()) telemetry.addLine("试射前先完成 04 飞轮 RPM 标定。");
                telemetry.update();
                sleep(20);
            }
            if (!opModeIsActive()) return;
            loop.beginFrame();
            if (turret.calibrated()) turret.homeAtRight();
            field = new ShotVectorField(store, rightHalf);
            ShotMap.exportRecords(store);
            refreshDataView();
            maxRpm = flywheel.maxTargetRpmAtBattery();
            targetRpm = Math.min(RobotConfig.FLYWHEEL_TARGET_RPM, maxRpm);
            lights.show(RearStatusLed.State.CAL_WAIT);
            runCalibration(loop);
        } finally {
            try {
                stopFeed();
            } finally {
                if (turret != null) ControlLoop.safeStop("云台", () -> turret.teachingMode(false));
                if (lights != null) ControlLoop.safeStop("尾灯", lights::stop);
            }
        }
    }

    private void runCalibration(ControlLoop loop) {
        Gamepad prevG1 = new Gamepad(), currG1 = new Gamepad();
        Gamepad prevG2 = new Gamepad(), currG2 = new Gamepad();
        prevG1.copy(gamepad1);
        prevG2.copy(gamepad2);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        double lastTime = Maths.time(), nextDisplay = lastTime, nextDashboard = lastTime;
        while (opModeIsActive()) {
            double now = loop.beginFrame();
            battery.update(now);
            maxRpm = flywheel.maxTargetRpmAtBattery();
            currG1.copy(gamepad1);
            currG2.copy(gamepad2);
            double rawDt = now - lastTime;
            if (rawDt > 0) loopHz += 0.1 * (1.0 / rawDt - loopHz);
            double dt = Maths.clip(rawDt, 0.001, 0.25);
            lastTime = now;

            turret.sample(dt);
            if (isStopRequested()) break;

            handleControls(currG1, prevG1, currG2, prevG2, now);
            updateTurretMotion(now, dt);
            updateShootingLoop(currG1, now, dt);
            updateLights();

            if (dashboard != null && now >= nextDashboard) {
                nextDashboard = now + 0.067;
                try {
                    publishDashboard(dashboard);
                } catch (RuntimeException ignored) {
                    // Dashboard failures must not disturb the calibration loop.
                }
            }
            if (now >= nextDisplay) {
                showCalibration();
                nextDisplay = now + 0.15;
            }
            Gamepad swap1 = prevG1;
            prevG1 = currG1;
            currG1 = swap1;
            Gamepad swap2 = prevG2;
            prevG2 = currG2;
            currG2 = swap2;
        }
    }

    private void handleControls(Gamepad currG1, Gamepad prevG1, Gamepad currG2, Gamepad prevG2, double now) {
        if (!currG1.start && currG1.b) {
            if (shooting) {
                finishShot(false, "试射已取消", now);
                cancelWaitRelease = currG1.right_trigger > TRIGGER_THRESHOLD;
                return;
            }
            if (turret.softHoming()) {
                turret.cancelSoftHome();
                status = "已取消右端寻零";
                return;
            }
            if (Double.isFinite(holdTarget)) {
                endHoldMove(previewMove ? "预瞄已取消" : "微调已停止");
                return;
            }
            if (!prevG1.b && dataViewMode) {
                setDataViewMode(false);
                status = "已返回试射界面";
                return;
            }
        }
        if (shooting) return;
        if (handleFileShortcuts(currG1, prevG1, now)) return;

        if (!currG1.start && currG1.left_stick_button && !prevG1.left_stick_button) {
            setDataViewMode(!dataViewMode);
            status = dataViewMode ? "已进入校准数据查看" : "已返回试射界面";
            return;
        }

        if (dataViewMode) {
            handleDataViewControls(currG1, prevG1);
            return;
        }

        // Deputy B: toggle one-button soft right-stop homing (matching TeleOp)
        if (!currG2.start && currG2.b && !prevG2.b && turret != null) {
            if (turret.softHoming()) {
                turret.cancelSoftHome();
                status = "已取消右端寻零";
                gamepad2.rumble(60);
            } else if (turret.calibrated()) {
                holdTarget = Double.NaN;
                setBrake(true);
                if (turret.startSoftHome()) {
                    status = "正在回右端自动寻零（再按副手 B 取消）";
                    gamepad2.rumble(80);
                }
            }
            return;
        }
        if (turret.softHoming()) return;

        // Deputy RPM fine trim & yaw fine/large nudges (matching TeleOp without drive)
        double g2RpmStep = 0;
        if (!currG2.start) {
            if (currG2.dpad_up && !prevG2.dpad_up) g2RpmStep += RobotConfig.G2_RPM_FINE_STEP;
            if (currG2.dpad_down && !prevG2.dpad_down) g2RpmStep -= RobotConfig.G2_RPM_FINE_STEP;
        }
        if (!currG1.start) {
            if (currG1.left_bumper && !prevG1.left_bumper) g2RpmStep -= RobotConfig.G2_RPM_FINE_STEP;
            if (currG1.right_bumper && !prevG1.right_bumper) g2RpmStep += RobotConfig.G2_RPM_FINE_STEP;
        }
        if (g2RpmStep != 0) {
            targetRpm = Maths.clip(targetRpm + g2RpmStep, 0, maxRpm);
        }

        if (!currG2.start) {
            if (currG2.x && !prevG2.x) {
                rpmLockEnabled = !rpmLockEnabled;
                status = rpmLockEnabled
                        ? "发射模式：锁飞轮转速（同步手动 R3）"
                        : "发射模式：高速直射（同步手动默认）";
                gamepad2.rumble(80);
            }
            if (currG2.y && !prevG2.y) {
                hotSpinIdle = !hotSpinIdle;
                status = hotSpinIdle
                        ? "待机模式：持续热转（模拟手动热态）"
                        : "待机模式：松手停转（安全默认）";
                gamepad2.rumble(80);
            }
            double yawDelta = 0;
            if (currG2.dpad_left && !prevG2.dpad_left) yawDelta += Math.toRadians(RobotConfig.G2_YAW_FINE_DEG);
            if (currG2.dpad_right && !prevG2.dpad_right) yawDelta -= Math.toRadians(RobotConfig.G2_YAW_FINE_DEG);
            if (currG2.left_bumper && !prevG2.left_bumper) yawDelta += Math.toRadians(RobotConfig.G2_YAW_LARGE_DEG);
            if (currG2.right_bumper && !prevG2.right_bumper) yawDelta -= Math.toRadians(RobotConfig.G2_YAW_LARGE_DEG);
            if (yawDelta != 0) {
                nudgeTurret(yawDelta, now);
                return;
            }
            if (currG2.a && !prevG2.a) {
                moveTurretTo(turret.forwardAngle(), now, false, "正在回正前零位");
                return;
            }
        }

        if (currG1.start) return;

        if (currG1.back && !prevG1.back) {
            toggleRefinements();
            return;
        }
        boolean left = currG1.dpad_left && !prevG1.dpad_left;
        boolean right = currG1.dpad_right && !prevG1.dpad_right;
        boolean down = currG1.dpad_down && !prevG1.dpad_down;
        boolean up = currG1.dpad_up && !prevG1.dpad_up;
        if (refinements == null) {
            int nextColumn = move(column, ShotVectorField.COLUMNS, left, right);
            int nextRow = move(row, ShotVectorField.ROWS, down, up);
            if (nextColumn != column || nextRow != row) {
                column = nextColumn;
                row = nextRow;
                pending = null;
                status = "已切换格子";
            }
        } else {
            int next = move(refinementIndex, refinements.size(), left || down, right || up);
            if (next != refinementIndex) selectRefinement(next);
        }

        if (currG1.x && !prevG1.x) {
            holdTarget = Double.NaN;
            previewMove = false;
            setBrake(!brake);
            status = brake ? "云台已刹车锁位" : "云台已松刹车，可手摆微调";
        }

        ShotVectorField.Refinement point = selectedRefinement();
        if (currG1.y && !prevG1.y) {
            boolean unreachable = !selectedUnreachable();
            boolean saved = point == null ? field.setUnreachable(column, row, unreachable)
                    : field.setUnreachable(point, unreachable);
            if (saved) {
                pending = null;
                status = unreachable ? "已标为不可达" : "已解除不可达标记";
                if (!ShotMap.exportRecords(store)) status += "，JSON 导出失败";
                refreshDataView();
            } else status = "保存不可达状态失败，请重试";
            return;
        }
        if (currG1.a && !prevG1.a) {
            if (pending == null) status = "请先在当前点完成一次试射";
            else if (point == null ? field.save(column, row, pending, trialVoltage, trialPitch)
                    : field.save(point, pending, trialVoltage, trialPitch)) {
                pending = null;
                status = ShotMap.exportRecords(store) ? "已保存当前点向量及 JSON" : "向量已保存，JSON 导出失败";
                refreshDataView();
            } else status = "保存向量失败，请重试";
            return;
        }
        if (currG1.right_stick_button && !prevG1.right_stick_button) {
            startPreview(point, now);
            return;
        }

        boolean triggerPressed = currG1.right_trigger > TRIGGER_THRESHOLD;
        if (!triggerPressed) {
            cancelWaitRelease = false;
            return;
        }
        if (!cancelWaitRelease && prevG1.right_trigger <= TRIGGER_THRESHOLD) {
            beginShot(now);
        }
    }

    private void nudgeTurret(double deltaRad, double now) {
        if (!turret.calibrated() || !turret.encoderReady() || !turret.motorResponding()) {
            status = "云台反馈未就绪";
            return;
        }
        double base = Double.isFinite(holdTarget) ? holdTarget : turret.angle;
        double desired = Maths.clip(base + deltaRad,
                RobotConfig.TURRET_MARGIN, turret.span() - RobotConfig.TURRET_MARGIN);
        String msg = String.format(Locale.ROOT, "微调云台至 %+.1f°（左为正）",
                Math.toDegrees(Maths.wrap(desired - turret.forwardAngle())));
        moveTurretTo(desired, now, false, msg);
    }

    private void moveTurretTo(double targetAngle, double now, boolean preview, String message) {
        if (!turret.calibrated() || !turret.encoderReady() || !turret.motorResponding()) {
            status = "云台反馈未就绪";
            return;
        }
        setBrake(true);
        holdTarget = Maths.clip(targetAngle, RobotConfig.TURRET_MARGIN, turret.span() - RobotConfig.TURRET_MARGIN);
        holdUntil = now + (preview ? 3.5 : 2.5);
        previewMove = preview;
        pending = null;
        status = message;
    }

    private void updateTurretMotion(double now, double dt) {
        if (shooting) return;
        if (turret.softHoming()) {
            Turret.SoftHomeResult result = turret.updateSoftHome(dt);
            if (result == Turret.SoftHomeResult.SUCCEEDED) {
                status = "云台右端寻零完成（按 X 松刹车可手摆）";
                gamepad2.rumble(150);
                gamepad1.rumble(100);
            } else if (result == Turret.SoftHomeResult.FAILED) {
                status = "右端寻零失败，请检查编码器或手动摆零";
                gamepad2.rumble(0.5, 0.5, 250);
            }
            return;
        }
        if (!Double.isFinite(holdTarget)) return;
        if (now >= holdUntil || !turret.encoderReady() || !turret.motorResponding()) {
            endHoldMove(previewMove ? "预瞄未到位，可手动微调" : "微调超时，可按 X 手调");
        } else if (Math.abs(turret.angle - holdTarget) <= Math.toRadians(0.9) && turret.stableForShot()) {
            endHoldMove(previewMove ? "预瞄完成，按 X 松刹车微调" : "云台微调到位");
        } else {
            turret.control(holdTarget, 0, dt);
        }
    }

    private void endHoldMove(String message) {
        turret.stop();
        holdTarget = Double.NaN;
        previewMove = false;
        status = message;
    }

    private void beginShot(double now) {
        String blocked = shotBlocker();
        if (blocked != null) {
            status = blocked;
            cancelWaitRelease = true;
            return;
        }
        if (turret.softHoming()) turret.cancelSoftHome();
        holdTarget = Double.NaN;
        previewMove = false;
        setBrake(true);
        if (!brake) {
            status = "云台刹车上锁失败，检查电机";
            cancelWaitRelease = true;
            return;
        }
        lockedTurretAngle = turret.angle;
        shot.releaseTrigger();
        pending = null;
        trial = null;
        shooting = true;
        firedThisPress = false;
        feedSeconds = 0;
        shotRpmAtRelease = Double.NaN;
        shotMinRpm = Double.NaN;
        shotEndRpm = Double.NaN;
        spinupDeadline = now + SPIN_UP_TIMEOUT;
        status = hotSpinIdle ? "已锁云台刹车，等待飞轮入闸" : "已锁云台刹车，等待飞轮从零达速稳态";
        RobotLog.ii("BioBuzzShot", "06 RT pressed: targetRpm=%.0f turret=%+.2f deg mode=%s idle=%s",
                targetRpm, Math.toDegrees(turret.boreRelative()),
                rpmLockEnabled ? "rpm-lock" : "fast-direct", hotSpinIdle ? "hot-spin" : "stop");
    }

    private void updateShootingLoop(Gamepad currG1, double now, double dt) {
        boolean triggerPressed = !dataViewMode && !currG1.start && currG1.right_trigger > TRIGGER_THRESHOLD;
        boolean keepSpinning = hotSpinIdle && !dataViewMode && !turret.softHoming()
                && flywheel.rpmCalibrated() && !flywheel.faulted() && targetRpm > 0;
        boolean idleWhenNotFiring = !keepSpinning || rpmLockEnabled;
        boolean stopWhenIdle = !keepSpinning;
        double settleSec = keepSpinning ? 0.0 : RPM_SETTLE_SECONDS;
        double strictTol = keepSpinning ? 0.0 : SPINUP_TOLERANCE;
        if (!shooting) {
            shot.update(false, false, false, false, false, targetRpm, rpmLockEnabled, idleWhenNotFiring, true,
                    settleSec, stopWhenIdle, strictTol);
            flywheel.feed(false);
            intake.set(false, false, false);
            blockerOpen = false;
            return;
        }
        if (!triggerPressed) {
            finishShot(firedThisPress, firedThisPress ? null : "已松开 RT 取消试射", now);
            return;
        }
        String blocked = shotBlocker();
        if (blocked != null) {
            finishShot(false, blocked, now);
            cancelWaitRelease = true;
            return;
        }

        // Actively brake & hold turret at the aimed angle while spinning up and feeding
        turret.control(lockedTurretAngle, 0, dt);

        boolean releaseShot = shot.update(true, false, false, false, false, targetRpm, rpmLockEnabled,
                idleWhenNotFiring, true, settleSec, false, strictTol);
        boolean firing = flywheel.feed(releaseShot);
        intake.set(false, firing, false);
        if (blockerOpen != firing) {
            RobotLog.ii("BioBuzzShot", "06 SB %s (rpm=%.0f target=%.0f turret=%+.2f deg)",
                    firing ? "opened" : "closed", flywheel.rpm(), targetRpm,
                    Math.toDegrees(turret.boreRelative()));
            blockerOpen = firing;
        }

        if (firing) {
            double curRpm = flywheel.rpm();
            if (!firedThisPress) {
                firedThisPress = true;
                shotRpmAtRelease = curRpm;
                trial = ShotVectorField.Vector.fromShot(turret.boreRelative(), targetRpm);
                trialVoltage = battery.measured() ? battery.volts() : -1;
                trialPitch = pitch != null ? pitch.getPosition() : -1;
                gamepad1.rumble(120);
            }
            shotMinRpm = Double.isFinite(shotMinRpm) ? Math.min(shotMinRpm, curRpm) : curRpm;
            shotEndRpm = curRpm;
            feedSeconds += dt;
        }

        if (Math.abs(turret.angle - lockedTurretAngle) > RobotConfig.SHOT_AIM_TOLERANCE) {
            finishShot(false, "云台偏移过大，已中止供弹", now);
            cancelWaitRelease = true;
        } else if (!firedThisPress && now >= spinupDeadline) {
            finishShot(false, "等待云台停稳或飞轮从零达速超时，请再试", now);
            cancelWaitRelease = true;
        }
    }

    private void finishShot(boolean keepTrial, String customMessage, double now) {
        if (keepTrial && trial != null) {
            pending = trial;
            status = String.format(Locale.ROOT, "供弹 %.2fs（放行 %.0f / 最低 %.0f / 末 %.0f / 目标 %.0f），命中按 A 保存",
                    feedSeconds, shotRpmAtRelease, shotMinRpm, shotEndRpm, targetRpm);
        } else {
            status = customMessage != null ? customMessage : "试射结束";
        }
        shot.releaseTrigger();
        if (hotSpinIdle && !dataViewMode && !isStopRequested() && (flywheel == null || !flywheel.faulted())) {
            if (flywheel != null) flywheel.feed(false);
            if (intake != null) intake.set(false, false, false);
        } else {
            stopFeed();
        }
        if (turret != null && brake) turret.stop();
        blockerOpen = false;
        shooting = false;
        firedThisPress = false;
        trial = null;
        if (keepTrial) {
            RobotLog.ii("BioBuzzShot",
                    "06 RT released: feedSeconds=%.2f releaseRpm=%.0f minRpm=%.0f endRpm=%.0f target=%.0f",
                    feedSeconds, shotRpmAtRelease, shotMinRpm, shotEndRpm, targetRpm);
        }
        shotMinRpm = Double.NaN;
        shotEndRpm = Double.NaN;
    }

    private void updateLights() {
        if (lights == null) return;
        if (shooting) {
            lights.show(blockerOpen ? RearStatusLed.State.AIMED_LOCKED : RearStatusLed.State.CAL_RUNNING);
            return;
        }
        boolean fault = !turret.calibrated() || !turret.isHomed() || !turret.encoderReady()
                || !turret.motorResponding() || !flywheel.rpmCalibrated() || flywheel.faulted() || !flywheel.hasBlocker();
        if (fault) lights.update(RearStatusLed.State.FAULT);
        else lights.update(brake ? RearStatusLed.State.CAL_WAIT : RearStatusLed.State.DRIVE);
    }

    private void publishDashboard(FtcDashboard dashboard) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("loop_hz", loopHz);
        packet.put("battery_V", battery.volts());
        packet.put("feeding", blockerOpen ? 1 : 0);
        packet.put("rpm_lock", rpmLockEnabled ? 1 : 0);
        packet.put("hot_spin_idle", hotSpinIdle ? 1 : 0);
        if (turret != null) {
            packet.put("turret_angle_deg", Math.toDegrees(turret.angle));
            packet.put("turret_target_deg", Math.toDegrees(Double.isFinite(holdTarget) ? holdTarget
                    : shooting ? lockedTurretAngle : turret.angle));
            packet.put("turret_rate_deg_s", Math.toDegrees(turret.speed));
            packet.put("turret_voltage", turret.output() * BatteryVoltage.NOMINAL_VOLTS);
            packet.put("turret_duty", turret.actualDuty());
            packet.put("turret_brake_voltage", turret.brakeDuty() * BatteryVoltage.NOMINAL_VOLTS);
            packet.put("turret_observer_bias", turret.observerBias());
        }
        if (flywheel != null && flywheel.rpmCalibrated()) {
            packet.put("flywheel_rpm", flywheel.rpm());
            packet.put("flywheel_target_rpm", flywheel.target * 60 / flywheel.ticksPerRev);
            packet.put("flywheel_voltage", flywheel.deliveredVolts());
            packet.put("flywheel_pid_integral_voltage", flywheel.integralVolts());
        }
        dashboard.sendTelemetryPacket(packet);
    }

    private boolean handleFileShortcuts(Gamepad current, Gamepad previous, double now) {
        if (current.start && current.x && !(previous.start && previous.x)) {
            boolean deleted = store.deleteShotFiles();
            refreshDataView();
            status = deleted ? "已一键删除 shot_records.json 和 shot_model.json" : "删除 JSON 文件失败";
            return true;
        }
        if (current.start && current.y && !(previous.start && previous.y)) {
            if (store.clearLegacyShotPoints()) {
                if (field != null) {
                    ShotVectorField.Refinement selected = selectedRefinement();
                    field = new ShotVectorField(store, rightHalf);
                    if (selected != null) {
                        refinements = field.refinements();
                        pending = null;
                    }
                }
                status = ShotMap.exportRecords(store) ? "两半场旧 v1 精测已清空并更新 JSON" : "已清空旧 v1，JSON 导出失败";
                refreshDataView();
            } else status = "旧精测清空失败";
            return true;
        }
        if (current.start && current.back && !(previous.start && previous.back)) {
            if (now <= confirmClearAllUntil) {
                confirmClearAllUntil = Double.NEGATIVE_INFINITY;
                if (store.clearAllShotData()) {
                    if (field != null) field = new ShotVectorField(store, rightHalf);
                    refinements = null;
                    pending = null;
                    dataPage = 0;
                    refreshDataView();
                    status = "已清空全部 06 测点并删除 shot_records / shot_model";
                } else status = "清空全部 06 数据失败";
            } else {
                confirmClearAllUntil = now + 3.0;
                status = "3秒内再按 START+Back 确认清空全部06测点与JSON";
            }
            return true;
        }
        return false;
    }

    private void handleDataViewControls(Gamepad current, Gamepad previous) {
        if (current.start) return;
        if (current.a && !previous.a) {
            showAllPoints = !showAllPoints;
            dataPage = 0;
            return;
        }
        if (current.x && !previous.x) {
            detailView = !detailView;
            dataPage = 0;
            telemetry.setDisplayFormat(detailView ? Telemetry.DisplayFormat.CLASSIC : Telemetry.DisplayFormat.MONOSPACE);
            return;
        }
        if (current.y && !previous.y) {
            status = ShotMap.exportRecords(store) ? ShotMap.RECORDS_FILE + " 已重新导出" : "JSON 导出失败";
            refreshDataView();
            return;
        }
        List<PointData> visible = showAllPoints ? allPoints : recordedPoints;
        List<String> rows = showAllPoints ? allCompactRows : recordedCompactRows;
        int perPage = detailView ? DETAIL_POINTS_PER_PAGE : COMPACT_ROWS_PER_PAGE;
        int count = detailView ? visible.size() : rows.size();
        int pages = (count + perPage - 1) / perPage;
        boolean left = current.dpad_left && !previous.dpad_left;
        boolean right = current.dpad_right && !previous.dpad_right;
        if (left != right && pages > 0) {
            dataPage = Math.max(0, Math.min(dataPage + (right ? 1 : -1), pages - 1));
        }
    }

    private void setDataViewMode(boolean enabled) {
        dataViewMode = enabled;
        dataPage = 0;
        if (enabled) refreshDataView();
        telemetry.setDisplayFormat(enabled && !detailView
                ? Telemetry.DisplayFormat.MONOSPACE : Telemetry.DisplayFormat.CLASSIC);
    }

    private void refreshDataView() {
        allPoints.clear();
        allPoints.addAll(new ShotVectorField(store, false).pointsForReview());
        allPoints.addAll(new ShotVectorField(store, true).pointsForReview());
        recordedPoints.clear();
        for (PointData point : allPoints) if (point.unreachable || point.shot != null) recordedPoints.add(point);
        allCompactRows.clear();
        allCompactRows.addAll(compactRows(allPoints));
        recordedCompactRows.clear();
        recordedCompactRows.addAll(compactRows(recordedPoints));
        modelStatus = new ShotMap(store).description();
    }

    private String jsonFilesStatus() {
        return String.format(Locale.ROOT, "records:%s | model:%s (%s)",
                store.hasJsonFile(ShotMap.RECORDS_FILE) ? "有" : "无",
                store.hasJsonFile(ShotMap.FILE) ? "有" : "无",
                modelStatus);
    }

    private void startPreview(ShotVectorField.Refinement point, double now) {
        if (!turret.calibrated() || !turret.encoderReady() || !turret.motorResponding()) {
            status = "云台反馈未就绪";
            return;
        }
        double x = point == null ? field.centerRightTiles(column) : point.rightTiles;
        double y = point == null ? field.centerForwardTiles(row) : point.forwardTiles;
        ShotMap model = new ShotMap(store);
        ShotVectorField.Vector guess = model.preview(x, y);
        double bearing = guess != null ? guess.bearing()
                : FieldTargets.fromPose(y * RobotConfig.TILE_MM, -x * RobotConfig.TILE_MM, 0).bearing;
        double angle = turret.previewAngle(bearing);
        if (!Double.isFinite(angle)) {
            status = "预瞄目标超出云台安全行程";
            return;
        }
        moveTurretTo(angle, now, true, "云台预瞄中，B 取消");
    }

    private void toggleRefinements() {
        if (refinements != null) {
            ShotVectorField.Refinement point = selectedRefinement();
            column = point.column;
            row = point.row;
            refinements = null;
            targetRpm = coarseRpm;
            pending = null;
            status = "已回到格心校准";
            return;
        }
        List<ShotVectorField.Refinement> points = field.refinements();
        if (points.isEmpty()) {
            status = "先保存至少一个可达格心再进入精测";
            return;
        }
        refinements = points;
        coarseRpm = targetRpm;
        int first = 0;
        for (int i = 0; i < points.size(); i++) {
            ShotVectorField.Refinement point = points.get(i);
            if (point.column == column && point.row == row) {
                first = i;
                break;
            }
        }
        selectRefinement(first);
    }

    private void selectRefinement(int index) {
        refinementIndex = index;
        pending = null;
        ShotVectorField.Refinement point = selectedRefinement();
        ShotVectorField.Vector shotVec = field.at(point);
        if (shotVec == null) shotVec = field.at(point.column, point.row);
        targetRpm = Math.min(shotVec.rpm(), maxRpm);
        status = "照摆车提示对齐两条地垫缝";
    }

    private ShotVectorField.Refinement selectedRefinement() {
        return refinements == null ? null : refinements.get(refinementIndex);
    }

    private boolean selectedUnreachable() {
        ShotVectorField.Refinement point = selectedRefinement();
        return point == null ? field.isUnreachable(column, row) : field.isUnreachable(point);
    }

    private String shotBlocker() {
        if (selectedUnreachable()) return "当前点不可达，Y 解除标记";
        if (!turret.calibrated()) return "需要先运行 02 标定云台";
        if (!turret.isHomed()) return "云台零位读取失败，重新 INIT 并摆到最右端";
        if (!turret.encoderReady() || !turret.motorResponding()) return "云台编码器或电机不可用";
        if (turret.angle < RobotConfig.TURRET_MARGIN || turret.angle > turret.span() - RobotConfig.TURRET_MARGIN) {
            return "云台太靠近行程端点，请手动调角";
        }
        if (!flywheel.rpmCalibrated()) return "需要先运行 04 标定飞轮 RPM";
        if (!flywheel.hasBlocker() || flywheel.faulted()) return "检查拨球舵机 SB 和飞轮";
        if (targetRpm <= 0) return "目标 RPM 为 0，请提高转速";
        return null;
    }

    private void setBrake(boolean engaged) {
        turret.teachingMode(!engaged);
        brake = engaged && turret.motorResponding();
    }

    private void stopFeed() {
        try {
            if (flywheel != null) flywheel.stop();
        } finally {
            if (intake != null) intake.stop();
        }
    }

    private void showCalibration() {
        if (dataViewMode) {
            showDataView();
            return;
        }
        ShotVectorField.Refinement point = selectedRefinement();
        if (point == null) {
            telemetry.addData("格心 右 / 前", "%.1f / %.1f 格，车头朝正前",
                    field.centerRightTiles(column), field.centerForwardTiles(row));
        } else {
            telemetry.addData("精测点", "%d / %d，%s", refinementIndex + 1, refinements.size(),
                    field.isUnreachable(point) ? "不可达" : field.at(point) != null ? "已保存" : "待测");
            telemetry.addData("地垫", "第 %d 行 · 第 %d 列（从场地左后角数）", point.row + 1, point.fieldColumn);
            telemetry.addData("摆车", "车身%s角对地垫%s角", point.corner.label, point.corner.label);
            telemetry.addLine("车头朝场地正前，车身两条边沿着接缝，整车放在这块地垫内。");
        }
        telemetry.addData("云台", "%s · %s",
                turret.motorResponding() ? (turret.softHoming() ? "回零中" : brake ? "刹车" : "可手摆") : "电机不可用",
                turret.encoderReady() ? String.format("%+.1f°（左为正）", Math.toDegrees(turret.boreRelative())) : "角度不可用");
        if (flywheel.rpmCalibrated()) {
            double[] gains = flywheel.pidGains();
            telemetry.addData("飞轮 实际 / 设定 / 上限", hotSpinIdle
                            ? "%.0f / %.0f / %.0f RPM（持续热转中）"
                            : "%.0f / %.0f / %.0f RPM（平时停转，按RT起转）",
                    flywheel.rpm(), targetRpm, maxRpm);
            telemetry.addData("飞轮 PID", "%s %.3f/%.3f/%.3f | %.0f Hz",
                    flywheel.pidSource(), gains[0], gains[1], gains[2], loopHz);
        }
        telemetry.addData("发射模式", "%s · %s（副手 X 切模式 / Y 切热转）",
                rpmLockEnabled ? "锁飞轮转速 [同步手动R3]" : "高速直射 [同步手动默认]",
                hotSpinIdle ? "持续热转" : "松手停转");
        String shotLine = status;
        if (shooting) {
            if (blockerOpen) {
                shotLine = String.format(Locale.ROOT, "正在供弹 %.2fs（放行 %.0f RPM），松开 RT 结束",
                        feedSeconds, shotRpmAtRelease);
            } else {
                String waiting = shot.waitingStatus();
                shotLine = waiting != null ? waiting : (hotSpinIdle ? "正在准备入闸" : "正在从零起转飞轮，等待稳态");
            }
        }
        telemetry.addData("试射", shotLine);
        telemetry.addData("射界文件", jsonFilesStatus());
        showVector("待保存", pending);
        showVector("当前点已保存", point == null ? field.at(column, row) : field.at(point));
        if (point == null) showGrid();
        telemetry.addLine(point == null ? "方向键选格 | Back 四角精测 | L3 查看数据" : "方向键左/右换角 | Back 回格心 | L3 查看数据");
        telemetry.addLine("LB/RB −/+25 | X 刹车/松开 | R3 预瞄 | 副手 上下±25RPM 左右±0.5° LB/RB±5° A回前 B寻零 X切模式 Y切热转");
        telemetry.addLine("按住 RT 锁台试射（高速直射同步手动默认）| A 保存 | Y 不可达 | B 取消");
        telemetry.addLine("START+X 删records/model JSON | START+Y 清旧v1 | START+Back 全清");
        telemetry.update();
    }

    private void showDataView() {
        List<PointData> visible = showAllPoints ? allPoints : recordedPoints;
        List<String> rows = showAllPoints ? allCompactRows : recordedCompactRows;
        int perPage = detailView ? DETAIL_POINTS_PER_PAGE : COMPACT_ROWS_PER_PAGE;
        int count = detailView ? visible.size() : rows.size();
        int pages = (count + perPage - 1) / perPage;
        dataPage = pages == 0 ? 0 : Math.min(dataPage, pages - 1);
        if (detailView) showDetails(visible, dataPage, pages);
        else showCompact(rows, visible.size(), dataPage, pages);
        telemetry.addData("文件", jsonFilesStatus());
        telemetry.addLine(status);
        telemetry.addLine("左右翻页 | A 全部/已录 | X 密集/详情 | Y 导出JSON | L3/B 回试射");
        telemetry.addLine("START+X 删records/model JSON | START+Y 清旧v1 | START+Back 全清");
        telemetry.update();
    }

    private void showCompact(List<String> rows, int pointCount, int page, int pages) {
        telemetry.addLine(String.format(Locale.ROOT, "%s%d点 %d/%d",
                showAllPoints ? "全部" : "记录", pointCount, pages == 0 ? 0 : page + 1, pages));
        if (rows.isEmpty()) telemetry.addLine("暂无已保存或标记的点");
        int start = page * COMPACT_ROWS_PER_PAGE;
        for (int i = start; i < Math.min(start + COMPACT_ROWS_PER_PAGE, rows.size()); i++) {
            telemetry.addLine(rows.get(i));
        }
    }

    private void showDetails(List<PointData> points, int page, int pages) {
        telemetry.addData(showAllPoints ? "全部点位" : "已记录点位", "%d 点 · %d/%d 页",
                points.size(), pages == 0 ? 0 : page + 1, pages);
        if (points.isEmpty()) {
            telemetry.addLine("暂无已保存或标记的点");
        } else {
            telemetry.addLine("左后角为原点，射向左为正，RPM 为目标转速");
            int start = page * DETAIL_POINTS_PER_PAGE;
            for (int i = start; i < Math.min(start + DETAIL_POINTS_PER_PAGE, points.size()); i++) {
                showPoint(points.get(i));
            }
        }
    }

    private void showPoint(PointData point) {
        String stateText = point.unreachable ? "不可达" : point.inactive ? "已停用"
                : point.shot == null ? "未测" : "已保存";
        telemetry.addLine(String.format(Locale.ROOT, "%d行%d列 %s · %s",
                point.row, point.column, point.label, stateText));
        String position = String.format(Locale.ROOT, "右 %.3f / 前 %.3f 格", point.rightTiles, point.forwardTiles);
        if (point.shot != null) {
            position += String.format(Locale.ROOT, " · %+.2f° · %.1f RPM",
                    Math.toDegrees(point.shot.bearing()), point.shot.rpm());
        }
        telemetry.addLine(position);
    }

    private static List<String> compactRows(List<PointData> points) {
        List<String> rows = new ArrayList<>();
        StringBuilder row = new StringBuilder();
        int pointsOnRow = 0;
        for (PointData point : points) {
            String entry = compactPoint(point);
            if (pointsOnRow > 0 && (pointsOnRow == POINTS_PER_ROW
                    || row.length() + 1 + entry.length() > COMPACT_LINE_WIDTH)) {
                rows.add(row.toString());
                row.setLength(0);
                pointsOnRow = 0;
            }
            if (pointsOnRow > 0) row.append(' ');
            row.append(entry);
            pointsOnRow++;
        }
        if (pointsOnRow > 0) rows.add(row.toString());
        return rows;
    }

    private static String compactPoint(PointData point) {
        String id = String.format(Locale.ROOT, "%d%d%c", point.row, point.column, point.code);
        if (point.code >= 'a' || point.inactive) {
            id += String.format(Locale.ROOT, "@%.3f,%.3f", point.rightTiles, point.forwardTiles);
        }
        String flags = (point.unreachable ? "!" : "") + (point.inactive ? "~" : "");
        if (point.shot == null) return id + ":?" + flags;

        long angle = Math.round(Math.toDegrees(point.shot.bearing()) * 100);
        long rpmTenths = Math.round(point.shot.rpm() * 10);
        String rpm = rpmTenths % 10 == 0 ? Long.toString(rpmTenths / 10)
                : String.format(Locale.ROOT, "%.1f", rpmTenths / 10.0);
        return id + ":" + angle + "/" + rpm + flags;
    }

    private void showGrid() {
        for (int r = ShotVectorField.ROWS - 1; r >= 0; r--) {
            StringBuilder cells = new StringBuilder();
            for (int c = 0; c < ShotVectorField.COLUMNS; c++) {
                String cell = field.isUnreachable(c, r) ? "×" : field.at(c, r) != null ? "✓" : "·";
                cells.append(c == column && r == row ? "[" + cell + "] " : " " + cell + "  ");
            }
            telemetry.addData("前 " + field.centerForwardTiles(r), cells.toString());
        }
        telemetry.addLine("· 待测  ✓ 已保存  × 不可达");
    }

    private void showVector(String caption, ShotVectorField.Vector vector) {
        if (vector != null) telemetry.addData(caption, "%+.1f° / %.0f RPM", Math.toDegrees(vector.bearing()), vector.rpm());
    }

    private static int move(int index, int size, boolean negative, boolean positive) {
        return Math.max(0, Math.min(size - 1, index + step(positive) - step(negative)));
    }

    private static int step(boolean pressed) { return pressed ? 1 : 0; }
}
