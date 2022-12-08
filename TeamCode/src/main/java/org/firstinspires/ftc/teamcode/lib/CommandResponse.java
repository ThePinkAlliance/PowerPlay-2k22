package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.DcMotor;

public class CommandResponse {
    private CommandStatus status;
    private DcMotor motor;

    public CommandResponse(DcMotor motor, CommandStatus status) {
        this.motor = motor;
        this.status = status;
    }

    public CommandStatus getStatus() {
        return status;
    }

    public boolean isBusy() {
        return motor.isBusy();
    }
}
