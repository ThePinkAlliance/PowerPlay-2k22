package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.lib.Subsystem;

public class Lift extends Subsystem {
    // Unit Inches
    private double currentElevatorHeight = 0;
    private double TICKS_PER_REV = 0.0;

    private final double extensionMaxTicks = 100;
    private final double extensionMaxInches = 100;
    private final double extensionInchesPerElevatorRotation = extensionMaxInches / (extensionMaxTicks / TICKS_PER_REV);
    private double currentExtensionDistance = 0;

    private final double clawHeightDifference = 0;
    private final double maxElevatorHeightTicks = 100;
    private final double maxElevatorHeightInches = 120;
    private final double inchesPerElevatorRotation = maxElevatorHeightInches / (maxElevatorHeightTicks / TICKS_PER_REV);
    private final double safeRotationHeightInches = 8;

    PIDFCoefficients pidCoefficients = new PIDFCoefficients();
    PIDFCoefficients extensionPidCoefficients = new PIDFCoefficients();

    public Lift(Hardware hardware) {
        super(hardware);

        this.hardware.liftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidCoefficients);
        this.hardware.liftMotor.setTargetPositionTolerance(2);

        this.hardware.extensionMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, extensionPidCoefficients);
        this.hardware.extensionMotor.setTargetPositionTolerance(2);
    }

    /**
     * Commands the lift to move to a specific height with the height of the claw in mind.
     */
    public void setLiftHeight(double height) {
        double rotations = (height + clawHeightDifference) / inchesPerElevatorRotation;
        double position = Range.clip(rotations * TICKS_PER_REV, 0, maxElevatorHeightTicks);

        this.hardware.liftMotor.setTargetPosition((int) position);

        // limit the set target position speed to 40%
        this.hardware.liftMotor.setPower(0.4);
        this.currentElevatorHeight = height + clawHeightDifference;
    }

    public void stopLift() {
        this.hardware.liftMotor.setPower(0);
    }

    public void setExtensionDistance(double distance) {
        double rotations = distance / inchesPerElevatorRotation;
        int position = (int) Range.clip(rotations * TICKS_PER_REV, 0, maxElevatorHeightTicks);

        this.hardware.extensionMotor.setTargetPosition(position);
        this.currentExtensionDistance = distance;
    }

    public void setExtensionPower(double power) {
        double processed_input = Range.clip(power, -0.3, 0.3);

        this.hardware.extensionMotor.setPower(processed_input);
    }

    public void stopExtensionMotor() {
        this.hardware.extensionMotor.setPower(0);
    }

    /**
     * Returns the current claw height from the ground in inches.
     */
    public double getClawHeight() {
        return currentElevatorHeight - clawHeightDifference;
    }

    /**
     * Returns if the current elevator height has cleared the zone that's dangerous to rotate.
     */
    public boolean hasClearedMinimumRotateHeight() {
        return getClawHeight() > safeRotationHeightInches;
    }

    /**
     * Move the lift to the minimum height that is safe for the turret to rotate.
     */
    public void goToSafeHeight() {
        this.setLiftHeight(safeRotationHeightInches);
    }

    /**
     * Returns the minimum height that's safe for the turret to rotate.
     */
    public double getSafeRotationHeight() {
        return safeRotationHeightInches;
    }

    /**
     * Returns the current elevator height in inches.
     */
    public double getElevatorHeight() {
        return currentElevatorHeight;
    }
}
