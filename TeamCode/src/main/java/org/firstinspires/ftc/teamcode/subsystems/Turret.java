package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.lib.CommandResponse;
import org.firstinspires.ftc.teamcode.lib.CommandStatus;
import org.firstinspires.ftc.teamcode.lib.Subsystem;

public class Turret extends Subsystem {
    // The sum of PPR (Pulses Per Revolution) * 4 is ticks per rotation.
    private final double TICKS_PER_ROT = 1538;

    private final float gearRatio = 27 / 3f;
    private final double degreesPerRotation = (360 / (1 / gearRatio));

    public Turret(Hardware hardware, Lift lift) {
        super(hardware);
    }

    public double getPosition() {
        return this.hardware.turretMotor.getCurrentPosition();
    }

    public double getTurretAngle() {
        return degreesPerRotation * ((this.hardware.turretMotor.getCurrentPosition() / TICKS_PER_ROT) * gearRatio);
    }
}
