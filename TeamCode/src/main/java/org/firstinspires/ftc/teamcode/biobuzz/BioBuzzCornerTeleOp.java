package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Biobuzz角落出发", group = "BIOBUZZ")
public final class BioBuzzCornerTeleOp extends BioBuzzTeleOp {
    @Override protected boolean usesShotCalibration() { return true; }
}
