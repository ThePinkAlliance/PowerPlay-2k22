
package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.CommandResponse;
import org.firstinspires.ftc.teamcode.lib.CommandStatus;
import org.firstinspires.ftc.teamcode.lib.PinkOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "drive")

public class TeleOp extends PinkOpMode {

    // The sum of PPR (Pulses Per Revolution) * 4 is ticks per rotation.
    private final double TICKS_PER_ROT = 1538;
    private SampleMecanumDrive drive;
    private final float gearRatio = 27 / 3f;
    private final double degreesPerRotation = (360 / (1 / gearRatio));

    private Lift lift;
    private Claw claw;

    double currentAngle = 0;

    /* public CommandResponse setTurretAngle(double targetAngle) {
        double _targetPosition = (targetAngle / degreesPerRotation) * TICKS_PER_ROT;

        if (lift.hasClearedMinimumRotateHeight()) {
            this.hardware.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.hardware.turretMotor.setTargetPosition((int) _targetPosition);
            this.hardware.turretMotor.setPower(0.5);
//            this.hardware.turretMotor.setPower(Math.copySign(0.5, _targetPosition));
            currentAngle = targetAngle;
            while (this.hardware.turretMotor.isBusy())
                telemetry.addData("Turret", "Moving");
            this.hardware.turretMotor.setPower(0);

            return new CommandResponse(this.hardware.turretMotor, CommandStatus.ACCEPTED);
        }

        return new CommandResponse(this.hardware.turretMotor, CommandStatus.ACCEPTED);
    } */





    @Override
    public void init() {
        initializeHardware(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        lift = new Lift(hardware);
        claw = new Claw(hardware);


    }

    @Override
    public void loop() {
        //if (gamepad1.dpad_left) setTurretAngle(currentAngle - 90);
        //if (gamepad1.dpad_right) setTurretAngle(currentAngle + 90);

        if(gamepad1.a) claw.moveClaw(0.5);
        if(gamepad1.b) claw.moveClaw(0);

        //stop button for lift is right_bumper as defined in Lift class
        if(gamepad2.b) lift.setLiftHeight(0, gamepad2);
        if(gamepad2.a) lift.setLiftHeight(13.5, gamepad2);
        if(gamepad2.x) lift.setLiftHeight(23.5, gamepad2);
        if(gamepad2.y) lift.setLiftHeight(34, gamepad2);
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y * DriveConstants.FORWARD_MULTIPLIER,
                        -gamepad1.left_stick_x * DriveConstants.STRAFE_MULTIPLIER,
                        -gamepad1.right_stick_x * DriveConstants.ROTATE_MULTIPLIER
                )
        );
        //if(gamepad2.left_bumper) lift.liftUpByRotation(999, gamepad2);

        //lift manual controls
        if(gamepad2.dpad_up) lift.liftUp(gamepad2);
        if(gamepad2.dpad_down) lift.liftDown(gamepad2);

    }
}
