package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
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
import org.firstinspires.ftc.teamcode.biobuzz.shooting.ShotMap;
import org.firstinspires.ftc.teamcode.biobuzz.shooting.ShotVectorField;
import org.firstinspires.ftc.teamcode.biobuzz.shooting.ShotVectorField.PointData;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Intake;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Turret;

@TeleOp(name = "06 半场射击校准·数据", group = "BIOBUZZ 自测")
public final class ShotFieldCalibration extends LinearOpMode {
    private static final double TRIGGER_THRESHOLD = 0.10;
    private static final double SPIN_UP_TIMEOUT = 5.0;
    private static final double FEED_SECONDS = 0.65;
    private static final int RPM_STEP = 25;
    private static final int COMPACT_ROWS_PER_PAGE = 10;
    private static final int COMPACT_LINE_WIDTH = 44;
    private static final int POINTS_PER_ROW = 3;
    private static final int DETAIL_POINTS_PER_PAGE = 3;
    private enum TrialState { IDLE, SPINNING_UP, FEEDING }

    private Turret turret;
    private Flywheel flywheel;
    private Intake intake;
    private CalibrationStore store;
    private BatteryVoltage battery;
    private Servo pitch;
    private boolean rightHalf;
    private double trialVoltage, trialPitch;
    private double previewAngle = Double.NaN, previewUntil;
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
        try (ControlLoop loop = new ControlLoop(hardwareMap)) {
            turret = new Turret(hardwareMap, store, battery);
            flywheel = new Flywheel(hardwareMap, store, battery);
            intake = new Intake(hardwareMap);
            pitch = hardwareMap.tryGet(Servo.class, RobotConfig.PITCH_SERVO);
            if (pitch != null) pitch.setPosition(RobotConfig.ST_HOME);

            turret.teachingMode(true);
            refreshDataView();
            telemetry.setMsTransmissionInterval(150);

            Gamepad initPrevious = new Gamepad();
            Gamepad initCurrent = new Gamepad();
            initPrevious.copy(gamepad1);
            while (opModeInInit()) {
                initCurrent.copy(gamepad1);
                if (!initCurrent.start) {
                    if (initCurrent.dpad_left) rightHalf = false;
                    if (initCurrent.dpad_right) rightHalf = true;
                }
                handleFileShortcuts(initCurrent, initPrevious, Maths.time());
                Gamepad swap = initPrevious;
                initPrevious = initCurrent;
                initCurrent = swap;

                telemetry.addLine("方向键左 / 右选半场。将云台轻摆到最右端后按 START。");
                telemetry.addData("3 × 6", rightHalf ? "右半场（右坐标 3–6 格）" : "左半场（右坐标 0–3 格）");
                telemetry.addData("射界文件", jsonFilesStatus());
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
            maxRpm = flywheel.maxTargetRpm();
            targetRpm = Math.min(RobotConfig.FLYWHEEL_TARGET_RPM, maxRpm);
            runCalibration(loop);
        } finally {
            try {
                stopFeed();
            } finally {
                if (turret != null) turret.teachingMode(false);
            }
        }
    }

    private void runCalibration(ControlLoop loop) {
        Gamepad previous = new Gamepad();
        Gamepad current = new Gamepad();
        previous.copy(gamepad1);
        double lastTime = Maths.time(), nextDisplay = lastTime;
        while (opModeIsActive()) {
            double now = loop.beginFrame();
            current.copy(gamepad1);
            double dt = Maths.clip(now - lastTime, 0.005, 0.25);
            lastTime = now;
            turret.sample(dt);
            flywheel.sample();
            if (isStopRequested()) break;
            handleControls(current, previous, now);
            updatePreview(now, dt);
            updateTrial(now);
            if (now >= nextDisplay) {
                showCalibration();
                nextDisplay = now + 0.15;
            }
            Gamepad swap = previous;
            previous = current;
            current = swap;
            long remaining = loop.remainingMillis(now);
            if (remaining > 0) sleep(remaining);
        }
    }

    private void handleControls(Gamepad current, Gamepad previous, double now) {
        if (!current.start && current.b && !previous.b) {
            if (state != TrialState.IDLE) {
                endTrial("试射已取消");
                return;
            }
            if (Double.isFinite(previewAngle)) {
                endPreview("预瞄已取消");
                return;
            }
            if (dataViewMode) {
                setDataViewMode(false);
                status = "已返回试射界面";
                return;
            }
        } else if (!current.start && current.b) {
            if (state != TrialState.IDLE) endTrial("试射已取消");
            if (Double.isFinite(previewAngle)) endPreview("预瞄已取消");
            return;
        }
        if (state != TrialState.IDLE || Double.isFinite(previewAngle)) return;
        if (handleFileShortcuts(current, previous, now)) return;

        if (current.left_stick_button && !previous.left_stick_button) {
            setDataViewMode(!dataViewMode);
            status = dataViewMode ? "已进入校准数据查看" : "已返回试射界面";
            return;
        }

        if (dataViewMode) {
            handleDataViewControls(current, previous);
            return;
        }
        if (current.start) return;

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
                if (!ShotMap.exportRecords(store)) status += "，JSON 导出失败";
                refreshDataView();
            } else status = "保存不可达状态失败，请重试";
            return;
        }
        if (current.a && !previous.a) {
            if (pending == null) status = "请先在当前点完成一次试射";
            else if (point == null ? field.save(column, row, pending, trialVoltage, trialPitch)
                    : field.save(point, pending, trialVoltage, trialPitch)) {
                pending = null;
                status = ShotMap.exportRecords(store) ? "已保存当前点向量及 JSON" : "向量已保存，JSON 导出失败";
                refreshDataView();
            } else status = "保存向量失败，请重试";
            return;
        }
        if (current.right_stick_button && !previous.right_stick_button) {
            startPreview(point, now);
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
        previewAngle = turret.previewAngle(bearing);
        if (!Double.isFinite(previewAngle)) {
            status = "预瞄目标超出云台安全行程";
            return;
        }
        setBrake(true);
        previewUntil = now + 3.5;
        status = "云台预瞄中，B 取消";
    }

    private void updatePreview(double now, double dt) {
        if (!Double.isFinite(previewAngle)) return;
        if (now >= previewUntil || !turret.encoderReady() || !turret.motorResponding()) {
            endPreview("预瞄未到位，可手动微调");
        } else if (Math.abs(turret.angle - previewAngle) <= Math.toRadians(1.5) && turret.stableForShot()) {
            endPreview("预瞄完成，按 X 松刹车微调");
        } else turret.control(previewAngle, 0, dt);
    }

    private void endPreview(String message) {
        turret.stop();
        previewAngle = Double.NaN;
        status = message;
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
        status = "照摆车提示对齐两条地垫缝";
    }

    private ShotVectorField.Refinement selectedRefinement() {
        return refinements == null ? null : refinements.get(refinementIndex);
    }

    private boolean selectedUnreachable() {
        ShotVectorField.Refinement point = selectedRefinement();
        return point == null ? field.isUnreachable(column, row) : field.isUnreachable(point);
    }

    private void updateTrial(double now) {
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
            if (!turret.stableForShot() || !flywheel.readyForShot() || !intake.readyToFeed()) return;
            trial = ShotVectorField.Vector.fromShot(turret.boreRelative(), targetRpm);
            trialVoltage = battery.measured() ? battery.volts() : -1;
            trialPitch = pitch != null ? pitch.getPosition() : -1;
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
            intake.set(false, true, false);
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
        telemetry.addData("云台", "%s · %s", turret.motorResponding() ? (brake ? "刹车" : "可手摆") : "电机不可用",
                turret.encoderReady() ? String.format("%+.1f°（左为正）", Math.toDegrees(turret.boreRelative())) : "角度不可用");
        if (flywheel.rpmCalibrated()) {
            telemetry.addData("飞轮 实际 / 目标 / 上限", "%.0f / %.0f / %.0f RPM",
                    flywheel.speed * 60 / flywheel.ticksPerRev, targetRpm, maxRpm);
        }
        telemetry.addData("试射", state == TrialState.IDLE ? status
                : state == TrialState.SPINNING_UP ? "等待云台停稳和飞轮到速" : "正在单次供弹");
        telemetry.addData("射界文件", jsonFilesStatus());
        showVector("待保存", pending);
        showVector("当前点已保存", point == null ? field.at(column, row) : field.at(point));
        if (point == null) showGrid();
        telemetry.addLine(point == null ? "方向键选格 | Back 四角精测 | L3 查看数据" : "方向键左/右换角 | Back 回格心 | L3 查看数据");
        telemetry.addLine("LB/RB −/+" + RPM_STEP + " RPM | X 刹车/松开 | R3 预瞄");
        telemetry.addLine("RT 单次试射 | A 保存试射 | Y 不可达/恢复 | B 取消试射");
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
