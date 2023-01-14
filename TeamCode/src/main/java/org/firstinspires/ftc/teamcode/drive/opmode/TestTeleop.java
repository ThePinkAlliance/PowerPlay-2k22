package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.PinkOpMode;

@Disabled
@TeleOp
public class TestTeleop extends PinkOpMode {
    @Override
    public void init() {
        initializeHardware(hardwareMap);
    }

    @Override
    public void loop() {
        this.hardware.backLeft.setPower(1);
    }
}
