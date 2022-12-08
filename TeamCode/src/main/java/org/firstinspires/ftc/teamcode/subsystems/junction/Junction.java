package org.firstinspires.ftc.pinkcode.subsystems.junction;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class Junction {
    Vector2d vector2d;
    JunctionType type;

    public Junction(Vector2d pose, JunctionType type) {
        this.vector2d = pose;
        this.type = type;
    }

    public double getJunctionHeight() {
        return this.type.height;
    }

    public JunctionType getType() {
        return type;
    }

    public Vector2d getVector() {
        return vector2d;
    }
}
