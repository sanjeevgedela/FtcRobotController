package org.firstinspires.ftc.teamcode.SourceCode.Auton.OLD;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.trajectorysequence.TrajectorySequence;

//@Autonomous(name = "redLeftAutonTestv2", group = "FRFR")
public class redLeftAutonTestv2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        TrajectorySequence Left = drive.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-38,-28,Math.toRadians(180)))

                .waitSeconds(1)

                .lineToLinearHeading(new Pose2d(-30,-10,Math.toRadians(180)))

                .turn(Math.toRadians(180))

                .forward(60)

                .splineToLinearHeading(new Pose2d(54,-26,Math.toRadians(0)),Math.toRadians(0))

                .waitSeconds(1)
                .build();

        TrajectorySequence Part2 = drive.trajectorySequenceBuilder(new Pose2d(54,-26,Math.toRadians(0)))

                .lineToLinearHeading(new Pose2d(30,-14,Math.toRadians(0)))

                .lineToLinearHeading(new Pose2d(-30,-10,Math.toRadians(180)))

                .lineToLinearHeading(new Pose2d(-60,-13,Math.toRadians(180)))

                .waitSeconds(0.5)
                .strafeRight(3.5)

                .waitSeconds(0.5)

                .waitSeconds(0.5)

                .lineToLinearHeading(new Pose2d(30,-12,Math.toRadians(0)))
                .build();

        waitForStart();
        sleep(1000);

        drive.followTrajectorySequence(Left);
    }
}