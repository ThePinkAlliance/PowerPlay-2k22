package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.lib.CommandResponse;
import org.firstinspires.ftc.teamcode.lib.CommandStatus;
import org.firstinspires.ftc.teamcode.lib.PinkOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class TeleOp extends PinkOpMode {
    // The sum of PPR (Pulses Per Revolution) * 4 is ticks per rotation.
    private final double TICKS_PER_ROT = 1538;

    private final float gearRatio = 27 / 3f;
    private final double degreesPerRotation = (360 / (1 / gearRatio));

    private Lift lift;

    double targetPosition = 0;

    public CommandResponse setTurretAngle(double targetAngle) {
        double _targetPosition = (targetAngle / degreesPerRotation) * TICKS_PER_ROT;

        if (lift.hasClearedMinimumRotateHeight()) {
            this.hardware.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.hardware.turretMotor.setTargetPosition((int) _targetPosition);
            this.hardware.turretMotor.setPower(0.5);
            while (this.hardware.turretMotor.isBusy())
                telemetry.addData("Turret", "Moving");
            this.hardware.turretMotor.setPower(0);

            return new CommandResponse(this.hardware.turretMotor, CommandStatus.ACCEPTED);
        }

        return new CommandResponse(this.hardware.turretMotor, CommandStatus.ACCEPTED);
    }

    @Override
    public void init() {
        initializeHardware(hardwareMap);

        lift = new Lift(hardware);
    }

    @Override
    public void loop() {
    }
}
