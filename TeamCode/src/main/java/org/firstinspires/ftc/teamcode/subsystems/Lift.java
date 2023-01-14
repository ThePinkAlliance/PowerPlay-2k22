package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.lib.Subsystem;

@Config
public class Lift extends Subsystem {
    // Unit Inches
    private double currentElevatorHeightTicks = 0; // updated in code to keep track of height
    private double ticksPerElevatorRevolution = 751; // static, defined by motor encoder specifications

    private Telemetry telemetry;

    private final double clawHeightDifference = 2.1; // adjustment to set the minimum height of the elevator if needed
    private final double maxElevatorHeightInches = 33; // measured by hand
    private final double inchesPerElevatorRotation = 12.25; // measured by hand
    private final double maxElevatorHeightTicks = maxElevatorHeightInches * ( ticksPerElevatorRevolution / inchesPerElevatorRotation );

    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0.2, 0, 0);
    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0.2, 0, 0, 0);
//    public static PIDFController controller = new PIDFController(pidfCoefficients);

    public Thread lastThread;

    public Lift(Hardware hardware, Telemetry telemetry) {
        super(hardware);

        this.telemetry = telemetry;

        this.hardware.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        this.hardware.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.hardware.liftMotor.setTargetPositionTolerance(2);

        //this.hardware.extensionMotor.setTargetPositionTolerance(2);
    }

    /**
     * Commands the lift to move to a specific height with the height of the claw in mind.
     */
    public void setLiftHeight(double height, Gamepad gamepad) { // input = height in inches
        // convert desired location in linear inches from bottom to the # of rotations required
        double rotations = (height - clawHeightDifference) / inchesPerElevatorRotation;

        // convert the required rotations to encoder ticks, and set a max safe distance
        double position = Range.clip(rotations * ticksPerElevatorRevolution, 0, maxElevatorHeightTicks);

        // tell the control hub the number of ticks we need to move
        this.hardware.liftMotor.setTargetPosition((int) position);
        this.hardware.liftMotor.setTargetPositionTolerance(100);

        // calculate height difference
        double currentHeightDifferenceTicks = rotations * ticksPerElevatorRevolution - this.hardware.liftMotor.getCurrentPosition();

        // tell control hub to start moving the motor. It will stop at the desired position on its own
        this.hardware.liftMotor.setPower(0.4 * currentHeightDifferenceTicks / Math.abs(currentHeightDifferenceTicks));

        //make sure the rest of the program does not continue until lift is done moving
        while (this.hardware.liftMotor.isBusy()) {
            this.telemetry.addData("Lift Position", this.hardware.liftMotor.getCurrentPosition());
            this.telemetry.addData("Lift Target Position", position);
            this.telemetry.update();

            telemetry.addData("Lift", "Moving");
            //stop moving the motor if this button is pressed
            if(gamepad.right_bumper) {
                break;
            }
        }
        // this stops the motor when while loop exist, either because motor is done moving or stop command was recieved
    }

    /**
     * Commands the lift to move to a specific height with the height of the claw in mind.
     */
    public void setHeight(double height, Gamepad gamepad) { // input = height in inches
        // convert desired location in linear inches from bottom to the # of rotations required
        double rotations = (height + clawHeightDifference) / inchesPerElevatorRotation;

        // convert the required rotations to encoder ticks, and set a max safe distance
        double targetPosition = Range.clip(rotations * ticksPerElevatorRevolution, 0, maxElevatorHeightTicks);

        // tell the control hub the number of ticks we need to move
        this.hardware.liftMotor.setTargetPosition((int) targetPosition);
        this.hardware.liftMotor.setTargetPositionTolerance(100);

        if (lastThread != null && lastThread.isAlive()) {
            lastThread.interrupt();
        }

        lastThread = new Thread(() -> {
            // calculate height difference
            double currentHeightDifferenceTicks = rotations * ticksPerElevatorRevolution - this.hardware.liftMotor.getCurrentPosition();

            // tell control hub to start moving the motor. It will stop at the desired position on its own
            this.hardware.liftMotor.setPower(0.4 * currentHeightDifferenceTicks / currentHeightDifferenceTicks);
        });

        lastThread.start();
    }

    public double getLiftPosition() {
        return this.hardware.liftMotor.getCurrentPosition();
    }

    public double getLiftVelocity() {
        return this.hardware.liftMotor.getVelocity(AngleUnit.DEGREES);
    }

    public void stopLift() {
        this.hardware.liftMotor.setPower(0);
    }

    // to test the distance one motor rotation will raise the lift, this will move the lift up one rotation
    public void liftUpByRotation(double rotations, Gamepad gamepad) {
        //tell the control hub the number of ticks we need to move
        this.hardware.liftMotor.setTargetPosition((int) (rotations * ticksPerElevatorRevolution));

        // tell control hub to start moving the motor. It will stop at the desired position on its own
        this.hardware.liftMotor.setPower(0.4);

        //make sure the rest of the program does not continue until lift is done moving
        while (this.hardware.liftMotor.isBusy()) {
            //telemetry.addData("Lift", "Moving");
            //record the current position as motor is moving
            this.currentElevatorHeightTicks = this.hardware.liftMotor.getCurrentPosition();
            //stop moving the motor if this button is pressed
            if(gamepad.right_bumper) {
                break;
            }
        }
        //this stops the motor when while loop exist, either because motor is done moving or stop command was issued
        this.hardware.liftMotor.setPower(0);
    }

    //manual controls

    public void liftUp(Gamepad gamepad) {
        //tell the control hub the number of ticks we need to move
        this.hardware.liftMotor.setTargetPosition((int) (this.hardware.liftMotor.getCurrentPosition()  + 0.25 * ticksPerElevatorRevolution));
        // tell control hub to start moving the motor. It will stop at the desired position on its own
        this.hardware.liftMotor.setPower(1);
        //make sure the rest of the program does not continue until lift is done moving
        while (this.hardware.liftMotor.isBusy()) {
            //telemetry.addData("Lift", "Moving");
            //record the current position as motor is moving
            this.currentElevatorHeightTicks = this.hardware.liftMotor.getCurrentPosition();
            //stop moving the motor if this button is pressed
            if(gamepad.right_bumper) {
                break;
            }
        }
        //this stops the motor when while loop exist, either because motor is done moving or stop command was issued
        this.hardware.liftMotor.setPower(0);
    }

    public void liftDown(Gamepad gamepad) {
        //tell the control hub the number of ticks we need to move
        this.hardware.liftMotor.setTargetPosition((int) (this.hardware.liftMotor.getCurrentPosition() - 0.25 * ticksPerElevatorRevolution));
        // tell control hub to start moving the motor. It will stop at the desired position on its own
        this.hardware.liftMotor.setPower(-0.4);
        //make sure the rest of the program does not continue until lift is done moving
        while (this.hardware.liftMotor.isBusy()) {
            //telemetry.addData("Lift", "Moving");
            //record the current position as motor is moving
            this.currentElevatorHeightTicks = this.hardware.liftMotor.getCurrentPosition();
            //stop moving the motor if this button is pressed
            if(gamepad.right_bumper) {
                break;
            }
        }
        //this stops the motor when while loop exist, either because motor is done moving or stop command was issued
        this.hardware.liftMotor.setPower(0);
    }

}
