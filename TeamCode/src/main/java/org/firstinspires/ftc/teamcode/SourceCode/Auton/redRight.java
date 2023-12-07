package org.firstinspires.ftc.teamcode.SourceCode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class redRight extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d(-60, -12, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(-39.2, -30, Math.toRadians(90)), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(-41.2, -46.8, Math.toRadians(-90)))
                .build();

        TrajectorySequence stage2 = drive.trajectorySequenceBuilder(new Pose2d(-41.2, -46.8, Math.toRadians(-90)))
                .splineToSplineHeading(new Pose2d(-12.4, -16.4, Math.toRadians(-90)), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-12, 57.2, Math.toRadians(90)), Math.toRadians(90))
                .build();

        waitForStart();
        drive.followTrajectorySequence(right);
        sleep(5000);
    }
}
