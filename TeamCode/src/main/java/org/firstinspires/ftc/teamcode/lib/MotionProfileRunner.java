package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;

public class MotionProfileRunner {
    private MotionProfile profile;
    private double startTime;
    private NanoClock clock;

    public MotionProfileRunner(MotionProfile profile) {
        this.profile = profile;
        this.clock = NanoClock.system();
    }

    public void setStart() {
        startTime = clock.seconds();
    }

    public MotionState get() {
        return this.profile.get(clock.seconds() - startTime);
    }
}
