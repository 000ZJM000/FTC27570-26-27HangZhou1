package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;
import org.firstinspires.ftc.teamcode.biobuzz.CalibrationStore;
import org.firstinspires.ftc.teamcode.biobuzz.Flywheel;
import org.firstinspires.ftc.teamcode.biobuzz.Intake;
import org.firstinspires.ftc.teamcode.biobuzz.Maths;
import org.firstinspires.ftc.teamcode.biobuzz.RobotConfig;
import org.firstinspires.ftc.teamcode.biobuzz.ShotVectorField;
import org.firstinspires.ftc.teamcode.biobuzz.Turret;

@TeleOp(name = "06 半场射击校准", group = "BIOBUZZ 自测")
public final class ShotFieldCalibration extends LinearOpMode {
    private static final double TRIGGER_THRESHOLD = 0.10;
    private static final double SPIN_UP_TIMEOUT = 5.0;
    private static final double FEED_SECONDS = 0.65;
    private static final int RPM_STEP = 25;
    private enum TrialState { IDLE, SPINNING_UP, FEEDING }

    private Turret turret;
    private Flywheel flywheel;
    private Intake intake;
    private ShotVectorField field;
    private boolean brake;
    private int column, row;
    private double targetRpm, maxRpm, deadline;
    private List<ShotVectorField.Refinement> refinements;
    private int refinementIndex;
    private double coarseRpm;
    private TrialState state = TrialState.IDLE;
    private ShotVectorField.Vector trial, pending;
    private String status = "选格、摆位、调速后扣 RT";

    @Override public void runOpMode() {
        CalibrationStore store = new CalibrationStore(hardwareMap.appContext);
        try {
            turret = new Turret(hardwareMap, store);
            flywheel = new Flywheel(hardwareMap, store);
            intake = new Intake(hardwareMap);
            Servo pitch = hardwareMap.tryGet(Servo.class, RobotConfig.PITCH_SERVO);
            if (pitch != null) pitch.setPosition(RobotConfig.ST_HOME);

            turret.teachingMode(true);
            boolean rightHalf = false;
            telemetry.setMsTransmissionInterval(150);
            while (opModeInInit()) {
                if (gamepad1.dpad_left) rightHalf = false;
                if (gamepad1.dpad_right) rightHalf = true;
                telemetry.addLine("方向键左 / 右选半场。将云台轻摆到最右端后按 START。");
                telemetry.addData("3 × 6", rightHalf ? "右半场（右坐标 3–6 格）" : "左半场（右坐标 0–3 格）");
                telemetry.addLine("每点将麦轮中心放在格子中心，车头朝场地正前方。");
                if (!turret.calibrated()) telemetry.addLine("试射前先完成 02 云台行程和正前方向标定。");
                if (!flywheel.rpmCalibrated()) telemetry.addLine("试射前先完成 04 飞轮 RPM 标定。");
                telemetry.update();
                sleep(20);
            }
            if (!opModeIsActive()) return;
            if (turret.calibrated()) turret.homeAtRight();
            field = new ShotVectorField(store, rightHalf);
            maxRpm = flywheel.maxTargetRpm();
            targetRpm = Math.min(RobotConfig.FLYWHEEL_TARGET_RPM, maxRpm);
            runCalibration();
        } finally {
            try {
                stopFeed();
            } finally {
                if (turret != null) turret.teachingMode(false);
            }
        }
    }

    private void runCalibration() {
        Gamepad previous = new Gamepad();
        Gamepad current = new Gamepad();
        previous.copy(gamepad1);
        double lastTime = Maths.time(), nextDisplay = lastTime;
        while (opModeIsActive()) {
            current.copy(gamepad1);
            double now = Maths.time();
            double dt = Maths.clip(now - lastTime, 0.005, 0.25);
            lastTime = now;
            turret.sample(dt);
            flywheel.sample();
            if (isStopRequested()) break;
            handleControls(current, previous, now);
            updateTrial(now, dt);
            if (now >= nextDisplay) {
                showCalibration();
                nextDisplay = now + 0.15;
            }
            Gamepad swap = previous;
            previous = current;
            current = swap;
            sleep(15);
        }
    }

    private void handleControls(Gamepad current, Gamepad previous, double now) {
        if (current.b) {
            if (state != TrialState.IDLE) endTrial("试射已取消");
            return;
        }
        if (state != TrialState.IDLE) return;

        if (current.back && !previous.back) {
            toggleRefinements();
            return;
        }
        boolean left = current.dpad_left && !previous.dpad_left;
        boolean right = current.dpad_right && !previous.dpad_right;
        boolean down = current.dpad_down && !previous.dpad_down;
        boolean up = current.dpad_up && !previous.dpad_up;
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

        if (current.x && !previous.x) setBrake(!brake);
        boolean slower = current.left_bumper && !previous.left_bumper;
        boolean faster = current.right_bumper && !previous.right_bumper;
        targetRpm = Maths.clip(targetRpm + (step(faster) - step(slower)) * RPM_STEP, 0, maxRpm);

        ShotVectorField.Refinement point = selectedRefinement();
        if (current.y && !previous.y) {
            boolean unreachable = !selectedUnreachable();
            boolean saved = point == null ? field.setUnreachable(column, row, unreachable)
                    : field.setUnreachable(point, unreachable);
            if (saved) {
                pending = null;
                status = unreachable ? "已标为不可达" : "已解除不可达标记";
            } else status = "保存不可达状态失败，请重试";
            return;
        }
        if (current.a && !previous.a) {
            if (pending == null) status = "请先在当前点完成一次试射";
            else if (point == null ? field.save(column, row, pending) : field.save(point, pending)) {
                pending = null;
                status = "已保存当前点向量";
            } else status = "保存向量失败，请重试";
            return;
        }
        if (current.right_trigger > TRIGGER_THRESHOLD && previous.right_trigger <= TRIGGER_THRESHOLD) {
            String blocked = shotBlocker();
            if (blocked != null) {
                status = blocked;
                return;
            }
            if (!brake) setBrake(true);
            pending = null;
            state = TrialState.SPINNING_UP;
            deadline = now + SPIN_UP_TIMEOUT;
        }
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
        ShotVectorField.Vector shot = field.at(point);
        if (shot == null) shot = field.at(point.column, point.row);
        targetRpm = Math.min(shot.rpm(), maxRpm);
        status = "将麦轮中心摆到精测坐标，车头朝正前";
    }

    private ShotVectorField.Refinement selectedRefinement() {
        return refinements == null ? null : refinements.get(refinementIndex);
    }

    private boolean selectedUnreachable() {
        ShotVectorField.Refinement point = selectedRefinement();
        return point == null ? field.isUnreachable(column, row) : field.isUnreachable(point);
    }

    private void updateTrial(double now, double dt) {
        if (state == TrialState.IDLE) return;
        String blocked = shotBlocker();
        if (blocked != null) {
            endTrial(blocked);
            return;
        }
        flywheel.requestRpm(targetRpm);
        if (state == TrialState.SPINNING_UP) {
            if (now >= deadline) {
                endTrial("等待云台停稳或飞轮到速超时，请再试");
                return;
            }
            if (!turret.stableForShot() || !flywheel.readyForShot()) return;
            trial = ShotVectorField.Vector.fromShot(turret.boreRelative(), targetRpm);
            state = TrialState.FEEDING;
            deadline = now + FEED_SECONDS;
        }

        double aimChange = Maths.wrap(turret.boreRelative() - trial.bearing());
        if (!turret.stableForShot() || Math.abs(aimChange) > RobotConfig.SHOT_AIM_TOLERANCE) {
            endTrial("云台移动，已中止供弹");
        } else if (now >= deadline) {
            pending = trial;
            endTrial("供弹结束，确认命中后按 A 保存");
        } else if (flywheel.feed(true)) {
            intake.set(false, true, false, dt);
        } else {
            endTrial("供弹未放行");
        }
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

    private void endTrial(String result) {
        stopFeed();
        state = TrialState.IDLE;
        trial = null;
        status = result;
    }

    private void stopFeed() {
        try {
            if (flywheel != null) flywheel.stop();
        } finally {
            if (intake != null) intake.set(false, false, false, 0);
        }
    }

    private void showCalibration() {
        ShotVectorField.Refinement point = selectedRefinement();
        if (point == null) {
            telemetry.addData("格心 右 / 前", "%.1f / %.1f 格，车头朝正前",
                    field.centerRightTiles(column), field.centerForwardTiles(row));
        } else {
            telemetry.addData("精测点", "%d / %d，%s", refinementIndex + 1, refinements.size(),
                    field.isUnreachable(point) ? "不可达" : field.at(point) != null ? "已保存" : "待测");
            telemetry.addData("场地 右 / 前", "%.0f / %.0f mm，车头朝正前",
                    point.rightTiles * RobotConfig.TILE_MM, point.forwardTiles * RobotConfig.TILE_MM);
            telemetry.addData("距父格中心 右 / 前", "%+.0f / %+.0f mm",
                    (point.rightTiles - field.centerRightTiles(point.column)) * RobotConfig.TILE_MM,
                    (point.forwardTiles - field.centerForwardTiles(point.row)) * RobotConfig.TILE_MM);
        }
        telemetry.addData("云台", "%s · %s", turret.motorResponding() ? (brake ? "刹车" : "可手摆") : "电机不可用",
                turret.encoderReady() ? String.format("%+.1f°（左为正）", Math.toDegrees(turret.boreRelative())) : "角度不可用");
        if (flywheel.rpmCalibrated()) {
            telemetry.addData("飞轮 实际 / 目标 / 上限", "%.0f / %.0f / %.0f RPM",
                    flywheel.speed * 60 / flywheel.ticksPerRev, targetRpm, maxRpm);
        }
        telemetry.addData("试射", state == TrialState.IDLE ? status
                : state == TrialState.SPINNING_UP ? "等待云台停稳和飞轮到速" : "正在单次供弹");
        showVector("待保存", pending);
        showVector("当前点已保存", point == null ? field.at(column, row) : field.at(point));
        if (point == null) showGrid();
        telemetry.addLine(point == null ? "方向键选格 | Back 进入精测" : "方向键左/右换点 | Back 回格心");
        telemetry.addLine("LB/RB −/+" + RPM_STEP + " RPM | X 刹车/松开");
        telemetry.addLine("RT 单次试射 | A 保存试射 | Y 不可达/恢复 | B 取消试射");
        telemetry.update();
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
