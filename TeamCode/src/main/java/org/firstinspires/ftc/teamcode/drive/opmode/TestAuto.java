package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous()
public class TestAuto extends LinearOpMode {
    Pose2d startingPose = new Pose2d(11.5, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory locationTwo = drive
                .trajectoryBuilder(startingPose).lineTo(new Vector2d(startingPose.getX(), 34.5)).build();
        Trajectory locationOne = drive.trajectoryBuilder(startingPose).splineTo(new Vector2d(startingPose.getX(), 34.5), 0).splineTo(new Vector2d(startingPose.getX() + 23, 34.5), 0).build();
//        Trajectory locationThree = drive.trajectoryBuilder(startingPose).lineTo(new Vector2d(startingPose.getX(), 34.5)).lineTo(new Vector2d(startingPose.getX() - 23, 34.5)).build();

        waitForStart();

        drive.followTrajectory(locationOne);
    }
}
