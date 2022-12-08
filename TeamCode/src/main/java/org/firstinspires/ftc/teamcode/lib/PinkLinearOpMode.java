package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Hardware;

public abstract class PinkLinearOpMode extends LinearOpMode implements OpType {
    protected Hardware hardware;

    @Override
    public void initializeHardware(HardwareMap map) {
        this.hardware = new Hardware(map);
    }
}
