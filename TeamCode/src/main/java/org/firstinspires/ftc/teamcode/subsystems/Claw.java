package org.firstinspires.ftc.pinkcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.pinkcode.lib.Subsystem;

public class Claw extends Subsystem {
    Servo claw;
    DcMotor extensionMotor;

    // TODO: Make a method for the barMover.
    public Claw(Hardware hardware) {
        super(hardware);

        claw = hardware.claw;
        extensionMotor = hardware.extensionMotor;
    }

    public void moveBar(double power) {
        extensionMotor.setPower(power);
    }

    public void moveClaw(double position) {
        claw.setPosition(position);
    }
}