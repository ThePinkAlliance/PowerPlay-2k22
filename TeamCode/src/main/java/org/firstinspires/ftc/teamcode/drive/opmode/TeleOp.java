
package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.CommandResponse;
import org.firstinspires.ftc.teamcode.lib.CommandStatus;
import org.firstinspires.ftc.teamcode.lib.PinkOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "drive")

@Config
public class TeleOp extends PinkOpMode {
    private SampleMecanumDrive drive;
    private final float gearRatio = 27 / 3f;
    private final double degreesPerRotation = (360 / (1 / gearRatio));

    private Lift lift;
    private Claw claw;

    // Unit Inches
    private double currentElevatorHeightTicks = 0; // updated in code to keep track of height
    private double ticksPerElevatorRevolution = 751; // static, defined by motor encoder specifications

    public static double clawHeightDifference = 2.55; // adjustment to set the minimum height of the elevator if needed
    public static double maxElevatorHeightInches = 33; // measured by hand
    public static double inchesPerElevatorRotation = 13.15; // measured by hand
    public static double maxElevatorHeightTicks = 1938.0;

    public static PIDCoefficients coefficients = new PIDCoefficients(0.0090, 0, 0.00006);
    public static PIDFController controller = new PIDFController(coefficients);

    private double targetPosition = 0;

    double currentAngle = 0;

    @Override
    public void init() {
        initializeHardware(hardwareMap);

        drive = new SampleMecanumDrive(hardwareMap);
        lift = new Lift(hardware, telemetry);
        claw = new Claw(hardware);

        controller.setOutputBounds(-1, 1);
    }

    @Override
    public void loop() {
        if(gamepad2.left_trigger > 0.1) claw.moveClaw(0.5);
        if(gamepad2.right_trigger > 0.1) claw.moveClaw(0);

        // stop button for lift is right_bumper as defined in Lift class
        if(gamepad2.b) controller.setTargetPosition(1);
        if(gamepad2.a) setHeight(13.5, gamepad2);
        if(gamepad2.x) setHeight(23.5, gamepad2);
        if(gamepad2.y) setHeight(34, gamepad2);

        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y * DriveConstants.FORWARD_MULTIPLIER,
                        -gamepad1.left_stick_x * DriveConstants.STRAFE_MULTIPLIER,
                        -gamepad1.right_stick_x * DriveConstants.ROTATE_MULTIPLIER
                )
        );

        Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();

        telemetry.addData("Front Left", this.hardware.frontLeft.getVelocity());
        telemetry.addData("Front Right", this.hardware.frontRight.getVelocity());
        telemetry.addData("Back Left", this.hardware.backLeft.getVelocity());
        telemetry.addData("Back Right", this.hardware.backRight.getVelocity());
        telemetry.addData("Lift Vel", this.lift.getLiftVelocity());
        telemetry.addData("Lift Position", this.lift.getLiftPosition());

        // lift manual controls
        if(gamepad2.dpad_up) lift.liftUp(gamepad2);
        if(gamepad2.dpad_down) lift.liftDown(gamepad2);

        double output = controller.update(this.hardware.liftMotor.getCurrentPosition());
        this.hardware.liftMotor.setPower(output);

        telemetry.addData("Lift Difference", controller.getTargetPosition() / this.hardware.liftMotor.getCurrentPosition());
        telemetry.addData("Lift Output", output);
        telemetry.addData("Lift Target", controller.getTargetPosition());

        telemetry.update();
    }

    /**
     * Commands the lift to move to a specific height with the height of the claw in mind.
     */
    public void setHeight(double height, Gamepad gamepad) { // input = height in inches
        // convert desired location in linear inches from bottom to the # of rotations required
        double rotations = (height / inchesPerElevatorRotation); //- (clawHeightDifference / inchesPerElevatorRotation);

        // convert the required rotations to encoder ticks, and set a max safe distance
        double targetPosition = Range.clip(rotations * ticksPerElevatorRevolution, 0, maxElevatorHeightTicks);

        controller.setTargetPosition(targetPosition);
    }
}
