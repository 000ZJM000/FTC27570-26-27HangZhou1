package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.biobuzz.CalibrationStore;
import org.firstinspires.ftc.teamcode.biobuzz.CalibratedShotMap;
import org.firstinspires.ftc.teamcode.biobuzz.ShotVectorField;
import org.firstinspires.ftc.teamcode.biobuzz.ShotRecords;
import org.firstinspires.ftc.teamcode.biobuzz.ShotVectorField.PointData;

@TeleOp(name = "07 校准数据查看", group = "BIOBUZZ 自测")
public final class ShotFieldDataView extends LinearOpMode {
    private static final int COMPACT_ROWS_PER_PAGE = 12;
    private static final int COMPACT_LINE_WIDTH = 44;
    private static final int POINTS_PER_ROW = 3;
    private static final int DETAIL_POINTS_PER_PAGE = 3;

    @Override public void runOpMode() {
        CalibrationStore store = new CalibrationStore(hardwareMap.appContext);
        List<PointData> all = new ArrayList<>();
        List<PointData> recorded = new ArrayList<>();
        List<String> allRows = new ArrayList<>(), recordedRows = new ArrayList<>();
        reload(store, all, recorded, allRows, recordedRows);

        boolean showAll = false, details = false;
        boolean exported = false, lastClear = false;
        String exportStatus = "", modelStatus = "";
        int page = 0;
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(150);
        while (!isStopRequested()) {
            boolean clear = gamepad1.start && gamepad1.y;
            boolean export = gamepad1.yWasPressed();
            if (isStarted() && ((!exported) || (export && !clear))) {
                exported = true;
                exportStatus = ShotRecords.export(store) ? ShotRecords.FILE + " 已导出" : "JSON 导出失败";
                modelStatus = new CalibratedShotMap(store).description();
                RobotLog.ii("BioBuzzShot", "%s", modelStatus);
            }
            if (isStarted() && clear && !lastClear) {
                if (store.clearLegacyShotPoints()) {
                    reload(store, all, recorded, allRows, recordedRows);
                    page = 0;
                    exportStatus = ShotRecords.export(store) ? "旧 v1 精测已清空并导出" : "已清空，JSON 导出失败";
                } else exportStatus = "清空旧精测失败";
            }
            lastClear = clear;
            if (gamepad1.aWasPressed()) {
                showAll = !showAll;
                page = 0;
            }
            if (gamepad1.xWasPressed()) {
                details = !details;
                page = 0;
                telemetry.setDisplayFormat(details ? Telemetry.DisplayFormat.CLASSIC : Telemetry.DisplayFormat.MONOSPACE);
            }
            List<PointData> visible = showAll ? all : recorded;
            List<String> rows = showAll ? allRows : recordedRows;
            int perPage = details ? DETAIL_POINTS_PER_PAGE : COMPACT_ROWS_PER_PAGE;
            int count = details ? visible.size() : rows.size();
            int pages = (count + perPage - 1) / perPage;
            boolean left = gamepad1.dpadLeftWasPressed();
            boolean right = gamepad1.dpadRightWasPressed();
            if (left != right) {
                page = Math.max(0, Math.min(page + (right ? 1 : -1), pages - 1));
            }
            if (details) showDetails(visible, showAll, page, pages);
            else showCompact(rows, visible.size(), showAll, page, pages);
            telemetry.addLine("Y 导出 JSON | START + Y 清空旧 v1 精测");
            if (!exportStatus.isEmpty()) telemetry.addLine(exportStatus);
            if (!modelStatus.isEmpty()) telemetry.addData("射界模型", modelStatus);
            telemetry.update();
            sleep(50);
        }
    }

    private static void reload(CalibrationStore store, List<PointData> all, List<PointData> recorded,
                               List<String> allRows, List<String> recordedRows) {
        all.clear();
        all.addAll(new ShotVectorField(store, false).pointsForReview());
        all.addAll(new ShotVectorField(store, true).pointsForReview());
        recorded.clear();
        for (PointData point : all) if (point.unreachable || point.shot != null) recorded.add(point);
        allRows.clear();
        allRows.addAll(compactRows(all));
        recordedRows.clear();
        recordedRows.addAll(compactRows(recorded));
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
        String status = (point.unreachable ? "!" : "") + (point.inactive ? "~" : "");
        if (point.shot == null) return id + ":?" + status;

        long angle = Math.round(Math.toDegrees(point.shot.bearing()) * 100);
        long rpmTenths = Math.round(point.shot.rpm() * 10);
        String rpm = rpmTenths % 10 == 0 ? Long.toString(rpmTenths / 10)
                : String.format(Locale.ROOT, "%.1f", rpmTenths / 10.0);
        return id + ":" + angle + "/" + rpm + status;
    }

    private void showCompact(List<String> rows, int pointCount, boolean showAll, int page, int pages) {
        telemetry.addLine(String.format(Locale.ROOT, "%s%d点 %d/%d  左右翻页 A筛选 X详情",
                showAll ? "全部" : "记录", pointCount, pages == 0 ? 0 : page + 1, pages));
        if (rows.isEmpty()) telemetry.addLine("暂无已保存或标记的点");
        int start = page * COMPACT_ROWS_PER_PAGE;
        for (int i = start; i < Math.min(start + COMPACT_ROWS_PER_PAGE, rows.size()); i++) {
            telemetry.addLine(rows.get(i));
        }
    }

    private void showDetails(List<PointData> points, boolean showAll, int page, int pages) {
        telemetry.addLine("方向键左右翻页 · A 筛选 · X 密集");
        telemetry.addData(showAll ? "全部点位" : "已记录点位", "%d 点 · %d/%d 页",
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
        String status = point.unreachable ? "不可达" : point.inactive ? "已停用"
                : point.shot == null ? "未测" : "已保存";
        telemetry.addLine(String.format(Locale.ROOT, "%d行%d列 %s · %s",
                point.row, point.column, point.label, status));
        String position = String.format(Locale.ROOT, "右 %.3f / 前 %.3f 格", point.rightTiles, point.forwardTiles);
        if (point.shot != null) {
            position += String.format(Locale.ROOT, " · %+.2f° · %.1f RPM",
                    Math.toDegrees(point.shot.bearing()), point.shot.rpm());
        }
        telemetry.addLine(position);
    }
}
