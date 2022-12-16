package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.Subsystem;

public class Claw extends Subsystem {
    Servo claw;
    DcMotor extensionMotor;

    public Claw(Hardware hardware) {
        super(hardware);

        claw = hardware.claw;
        //extensionMotor = hardware.extensionMotor;
    }

    public void moveClaw(double position) {
        claw.setPosition(position);
    }
}