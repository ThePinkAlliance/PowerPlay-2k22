package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

/**
 * Hardware handles finding and creating all the physical IO for the robot.
 * This class should never configure IO by itself that should be left for subsystems/opmodes to deal with.
 */
public class Hardware {
    // Drivetrain
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    // Turret
    public DcMotorEx turretMotor;

    // Lift
    public DcMotorEx liftMotor;
    public Servo claw;

    public WebcamName webcamFront;

    public Hardware(HardwareMap map) {
        // Commented out all the devices to allow for easy tensorflow model testing.
        frontLeft = map.get(DcMotorEx.class, "front-left");
        frontRight = map.get(DcMotorEx.class, "front-right");
        backRight = map.get(DcMotorEx.class, "back-right");
        backLeft = map.get(DcMotorEx.class, "back-left");

        liftMotor = map.get(DcMotorEx.class, "lift-motor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        webcamFront = map.get(WebcamName.class, "Webcam 1");
        claw = map.get(Servo.class, "claw");
    }
}
