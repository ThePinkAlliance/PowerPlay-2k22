package org.firstinspires.ftc.pinkcode.subsystems.junction;

public enum JunctionType {
    GROUND(0),
    LOW(13.5),
    MEDIUM(23.5),
    HIGH(33.5);

    final double height;

    JunctionType(double height) {
        this.height = height;
    }
}
