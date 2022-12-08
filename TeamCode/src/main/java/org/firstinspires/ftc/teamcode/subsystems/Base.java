package org.firstinspires.ftc.pinkcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.pinkcode.lib.Subsystem;

/**
 * @deprecated Please use the mecanum base subsystem.
 */
@Deprecated
public class Base extends Subsystem {
    public Base(Hardware hardware) {
        super(hardware);

        this.hardware.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.hardware.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.hardware.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.hardware.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.hardware.frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this.hardware.backRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Commanding all the motors on both sides of the drivetrain.
     * @param left
     * @param right
     */
    public void drive(double left, double right) {
        this.hardware.frontLeft.setPower(left);
        this.hardware.backLeft.setPower(left);

        this.hardware.frontRight.setPower(right);
        this.hardware.backRight.setPower(right);
    }

    public void mecanumDrive(double x, double y, double rot) {
        // r is the hypotenuse of (x,y) coordinate of left stick, robotAngle = angleTheta of (x,y) coordinate of left stick. rightX = turning speed
        double r = Math.hypot(x, y);
        double robotAngle = Math.atan2(y, x) - Math.PI / 4;
        double rightX = rot;

        // Equations below is motor speed for each wheel
        double v1 = r * Math.cos(robotAngle) - rightX;
        double v2 = r * Math.sin(robotAngle) + rightX;
        double v3 = r * Math.sin(robotAngle) - rightX;
        double v4 = r * Math.cos(robotAngle) + rightX;

        // If not turning give each wheel full power
        if (x == 0) {
            v1 += v1 / 3;
            v2 += v2 / 3;
            v3 += v3 / 3;
            v4 += v4 / 3;
        }

        this.hardware.frontLeft.setPower(v1);
        this.hardware.frontRight.setPower(v2);
        this.hardware.backLeft.setPower(v3);
        this.hardware.backRight.setPower(v4);
    }

    /**
     * This is just as an example please don't use this for comp.
     * @param power
     */
    @Deprecated
    public void move(double power) {
        drive(power, power);
    }
}
