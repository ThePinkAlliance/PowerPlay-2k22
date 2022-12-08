package org.firstinspires.ftc.teamcode.lib;

import org.firstinspires.ftc.teamcode.subsystems.Hardware;

public abstract class Subsystem {
    protected Hardware hardware;

    public Subsystem(Hardware hardware) {
        if (hardware == null) {
            throw new Error("hardware class is undefined, it's possible you instantiated this subsystem before the hardware class.");
        }

        this.hardware = hardware;
    }
}
