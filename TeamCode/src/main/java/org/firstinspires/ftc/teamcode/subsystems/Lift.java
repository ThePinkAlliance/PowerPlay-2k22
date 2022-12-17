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
    private double ticksPerElevatorRevolution = 751.8 ; // static, defined by motor encoder specifications

    //private final double extensionMaxTicks = 100;
    //private final double extensionMaxInches = 100;
    //private final double extensionInchesPerElevatorRotation = extensionMaxInches / (extensionMaxTicks / TICKS_PER_REV);
    //private double currentExtensionDistance = 0;

    private final double clawHeightDifference = 0; // adjustment to set the minimum height of the elevator if needed
    private final double maxElevatorHeightInches = 33; // measured by hand
     private final double inchesPerElevatorRotation = 11; // measured by hand
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
    public void setLiftHeight(double height) { //input = height in inches
        //convert desired location in linear inches from bottom to the # of rotations required
        double rotations = (height + clawHeightDifference) / inchesPerElevatorRotation;
        //convert the required rotations to encoder ticks, and set a max safe distance
        double position = -Range.clip(rotations * ticksPerElevatorRevolution, 0, maxElevatorHeightTicks);
        //tell the control hub the number of ticks we need to move
        this.hardware.liftMotor.setTargetPosition((int) position);
        //calculate height difference
        double currentHeightDifferenceTicks = position - this.hardware.liftMotor.getCurrentPosition();
        // tell control hub to start moving the motor. It will stop at the desired position on its own
        this.hardware.liftMotor.setPower(0.4 * currentHeightDifferenceTicks/Math.abs(currentHeightDifferenceTicks));
        //motor will be stopped by stopLiftWhenIdle()
        //this.currentElevatorHeight = height + clawHeightDifference;
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

    //manual controls

    public void upOneRotation() {
        this.hardware.liftMotor.setTargetPosition((int) ticksPerElevatorRevolution);
        this.hardware.liftMotor.setPower(-0.3);
    }

    public void liftUp() {
        //tell the control hub the number of ticks we need to move
        this.hardware.liftMotor.setTargetPosition((int) (this.hardware.liftMotor.getCurrentPosition()  - 0.25 * ticksPerElevatorRevolution));
        // tell control hub to start moving the motor. It will stop at the desired position on its own
        this.hardware.liftMotor.setPower(-0.3);
        //motor will be stopped by stopLiftWhenIdle()
    }

    public void liftDown() {
        //tell the control hub the number of ticks we need to move
        this.hardware.liftMotor.setTargetPosition((int) (this.hardware.liftMotor.getCurrentPosition() + 0.25 * ticksPerElevatorRevolution));
        // tell control hub to start moving the motor. It will stop at the desired position on its own
        this.hardware.liftMotor.setPower(0.05);
        //motor will be stopped by stopLiftWhenIdle()
    }

    public void stopLiftWhenIdle(boolean gamepad) {
        //when the motor is not trying to move into position or if cancel button pressed,
        if (!this.hardware.liftMotor.isBusy() || gamepad) {
            //set motor speed to 0 and set the target position to current position to stop lift motion
            //this.hardware.liftMotor.setTargetPosition(this.hardware.liftMotor.getCurrentPosition());
            this.hardware.liftMotor.setPower(0);
            this.hardware.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void DOGE() {
        System.out.println(this.hardware.liftMotor.getCurrentPosition());
    }

}
