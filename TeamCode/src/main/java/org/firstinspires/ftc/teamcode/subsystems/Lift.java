package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.lib.Subsystem;

public class Lift extends Subsystem {
    // Unit Inches
    private double currentElevatorHeightTicks = 0; // updated in code to keep track of height
    private double ticksPerElevatorRevolution = 1425.1; // static, defined by motor encoder specifications

    //private final double extensionMaxTicks = 100;
    //private final double extensionMaxInches = 100;
    //private final double extensionInchesPerElevatorRotation = extensionMaxInches / (extensionMaxTicks / TICKS_PER_REV);
    //private double currentExtensionDistance = 0;

    private final double clawHeightDifference = 0; // adjustment to set the minimum height of the elevator if needed
    private final double maxElevatorHeightInches = 33; // measured by hand
     private final double inchesPerElevatorRotation = 4.6875; // measured by hand
    //private final double safeRotationHeightInches = 8;
    private final double maxElevatorHeightTicks = maxElevatorHeightInches * ( ticksPerElevatorRevolution / inchesPerElevatorRotation );

    public Lift(Hardware hardware) {
        super(hardware);

        this.hardware.liftMotor.setTargetPositionTolerance(2);

        //this.hardware.extensionMotor.setTargetPositionTolerance(2);
    }

    /**
     * Commands the lift to move to a specific height with the height of the claw in mind.
     */
    public void setLiftHeight(double height, Gamepad gamepad) { //input = height in inches
        //convert desired location in linear inches from bottom to the # of rotations required
        double rotations = (height + clawHeightDifference) / inchesPerElevatorRotation;
        //convert the required rotations to encoder ticks, and set a max safe distance
        double position = Range.clip(rotations * ticksPerElevatorRevolution, 0, maxElevatorHeightTicks);
        //tell the control hub the number of ticks we need to move
        this.hardware.liftMotor.setTargetPosition((int) position);
        //calculate height difference
        double currentHeightDifferenceTicks = rotations * ticksPerElevatorRevolution - this.hardware.liftMotor.getCurrentPosition();
        // tell control hub to start moving the motor. It will stop at the desired position on its own
        this.hardware.liftMotor.setPower(0.4 * currentHeightDifferenceTicks/Math.abs(currentHeightDifferenceTicks));
        //make sure the rest of the program does not continue until lift is done moving
        while (this.hardware.liftMotor.isBusy()) {
            //telemetry.addData("Lift", "Moving");
            //stop moving the motor if this button is pressed
            if(gamepad.right_bumper) {
                break;
            }
        }
        //this stops the motor when while loop exist, either because motor is done moving or stop command was recieved
        this.hardware.liftMotor.setPower(0);
        //this.currentElevatorHeight = height + clawHeightDifference;
    }

    public void stopLift() {
        this.hardware.liftMotor.setPower(0);
    }

/*    public void setExtensionDistance(double distance) {
        double rotations = distance / inchesPerElevatorRotation;
        int position = (int) Range.clip(rotations * TICKS_PER_REV, 0, maxElevatorHeightTicks);
        while (this.hardware.extensionMotor.isBusy())
            telemetry.addData("Turret", "Moving");
        this.hardware.extensionMotor.setPower(0);

        this.hardware.extensionMotor.setTargetPosition(position);
        this.currentExtensionDistance = distance;
    }

    public void setExtensionPower(double power) {
        double processed_input = Range.clip(power, -0.3, 0.3);

        this.hardware.extensionMotor.setPower(processed_input);
    }

    public void stopExtensionMotor() {
        this.hardware.extensionMotor.setPower(0);
    } */

    /**
     * Returns the current claw height from the ground in inches.
     */
    /* public double getClawHeight() {
        return currentElevatorHeightTicks - clawHeightDifference;
     } */

    /**
     * Returns if the current elevator height has cleared the zone that's dangerous to rotate.
     */
    /* public boolean hasClearedMinimumRotateHeight() {
        return getClawHeight() > safeRotationHeightInches;
    } */

    /**
     * Move the lift to the minimum height that is safe for the turret to rotate.
     */
    /* public void goToSafeHeight() {
        this.setLiftHeight(safeRotationHeightInches, false);
    } */

    /**
     * Returns the minimum height that's safe for the turret to rotate.
     */
    /* public double getSafeRotationHeight() {
        return safeRotationHeightInches;
    } */

    /**
     * Returns the current elevator height in ticks.
     */
    /* public double getElevatorHeight() {
        return currentElevatorHeightTicks;
    } */

    //to test the distance one motor rotation will raise the lift, this will move the lift up one rotation
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
