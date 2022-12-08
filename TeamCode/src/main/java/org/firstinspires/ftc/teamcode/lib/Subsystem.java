package org.firstinspires.ftc.pinkcode.lib;

import org.firstinspires.ftc.pinkcode.subsystems.Hardware;

public abstract class Subsystem {
    protected Hardware hardware;

    public Subsystem(Hardware hardware) {
        if (hardware == null) {
            throw new Error("hardware class is undefined, it's possible you instantiated this subsystem before the hardware class.");
        }

        this.hardware = hardware;
    }
}
