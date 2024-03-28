package org.firstinspires.ftc.teamcode.SourceCode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SourceCode.Subsystems.AutonCommands;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.trajectorysequence.TrajectorySequence;

@Autonomous(name = "move")
public class movement extends LinearOpMode {
    SampleMecanumDrive drive;
    AutonCommands commands;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(14.7, -65, Math.toRadians(90)));
        TrajectorySequence temp = drive.trajectorySequenceBuilder(new Pose2d(14.7, -62.4, Math.toRadians(90)))
// score
                .splineToSplineHeading(new Pose2d(32.7, -34, Math.toRadians(180)), Math.toRadians(0))
                .waitSeconds(0.5)
                .splineToSplineHeading(new Pose2d(51.4, -42.2, Math.toRadians(0)), Math.toRadians(-90))
                .waitSeconds(0.3)
                .back(5)


                //CYCLE
                .splineToSplineHeading(new Pose2d(35,-7, Math.toRadians(180)), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-28, -10), Math.toRadians(180))
//                                        .strafeRight(.001)
//                                        .forward(7.75)
                .splineToConstantHeading(new Vector2d(-40, -13), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-58, -13), Math.toRadians(180))
                .waitSeconds(0.5)
                .back(3)
                //pick up
                .splineToSplineHeading(new Pose2d(-30, -13, Math.toRadians(0)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(18,-13), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(46.4, -27), Math.toRadians(0))
                .forward(5)
                .waitSeconds(.7)
                //score

                //CYCLE 2
                .back(5)
                .splineToSplineHeading(new Pose2d(20,-13, Math.toRadians(180)), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-28, -13), Math.toRadians(180))
//                                        .strafeRight(.001)
//                                        .forward(7.75)
                .splineToConstantHeading(new Vector2d(-40, -13), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-58, -13), Math.toRadians(180))
                .waitSeconds(0.5)
                .back(3)
                //pick up
                .splineToSplineHeading(new Pose2d(-30, -13, Math.toRadians(0)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(18,-13), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(46.4, -27), Math.toRadians(0))
                .forward(5)
                .waitSeconds(.7)
                //score

                //CYCLE 3
                .back(5)
                .splineToSplineHeading(new Pose2d(20,-13, Math.toRadians(180)), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-28, -13), Math.toRadians(180))
//                                        .strafeRight(.001)
//                                        .forward(7.75)
                .splineToConstantHeading(new Vector2d(-40, -13), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-51, -13), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-53, -26), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-58, -26), Math.toRadians(180))
                .waitSeconds(0.5)
                .back(3)
                //pick up
                .splineToConstantHeading(new Vector2d(-49, -13), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-30, -13, Math.toRadians(0)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(18,-13), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(46.4, -27), Math.toRadians(0))
                .forward(5)
                .waitSeconds(.7)
                //score
                        .build();
//
//            TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
//                    .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
//                        commands.rotateControl(0);
//                        commands.clawControl(0, 0);
//                        commands.slideMovement(1,215);
//                    })
//                    .splineToSplineHeading(new Pose2d(32.7, -34, Math.toRadians(180)), Math.toRadians(0))
//                    .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
//                        commands.clawControl(0,1);
//                    })
//                    .waitSeconds(0.2)
//                    .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
//                        commands.scorePositionLow();
//                    })
//                    .splineToSplineHeading(new Pose2d(51.4, -42.2, Math.toRadians(0)), Math.toRadians(-90))
//                    .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
//                        commands.clawControl(1,1);
//                    })
//                    .waitSeconds(0.3)
//                    .back(5)
//                    .addDisplacementMarker(() -> drive.followTrajectorySequenceAsync(cycle1))
//                    .build();
//
//            cycle1 = drive.trajectorySequenceBuilder(left.end())
//                    //CYCLE
//                    .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
//                        commands.reset();
//                    })
//                    .splineToSplineHeading(new Pose2d(35,-13, Math.toRadians(180)), Math.toRadians(180))
//                    .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
//                        commands.reset2();
//                    })
//                    .splineToConstantHeading(new Vector2d(-28, -13), Math.toRadians(180))
//                    .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
//                        commands.clawControl(1,0);
//                        commands.slideMovement(.75,220);
//                        commands.rotateControl(.15);
//                    })
////                                        .strafeRight(.001)
////                                        .forward(7.75)
//                    .splineToConstantHeading(new Vector2d(-45, -13), Math.toRadians(180))
//                    .splineToConstantHeading(new Vector2d(-52, -13 + stackDist), Math.toRadians(180))
//                    .forward(6,
//                            SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                    .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
//                        commands.clawControl(0,0);
//                    })
//                    .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
//                        commands.rotateControl(1);
//                    })
//                    .waitSeconds(0.5)
//                    .back(3)
//                    .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
//                        commands.reset();
//                    })
//                    //pick up
//                    .splineToSplineHeading(new Pose2d(-30, -13, Math.toRadians(0)), Math.toRadians(0))
//                    .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
//                        commands.reset2();
//                    })
//                    .splineToConstantHeading(new Vector2d(25,-13), Math.toRadians(0))
//                    .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
//                        commands.scorePositionMid();
//                    })
//                    .splineToConstantHeading(new Vector2d(46.4, -27 + tagDist), Math.toRadians(0))
//                    .forward(5)
//                    .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
//                        commands.clawControl(1,0);
//                    })
//                    .waitSeconds(.3)
//                    .addDisplacementMarker(() -> drive.followTrajectorySequenceAsync(cycle2))
//                    //score
//                    .build();
//
//            cycle2 = drive.trajectorySequenceBuilder(cycle1.end())
//                    //CYCLE 2
//                    .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
//                        commands.reset();
//                    })
//                    .splineToSplineHeading(new Pose2d(35,-13, Math.toRadians(180)), Math.toRadians(180))
//                    .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
//                        commands.reset2();
//                    })
//                    .splineToConstantHeading(new Vector2d(-28, -13), Math.toRadians(180))
//                    .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
//                        commands.clawControl(1,0);
//                        commands.slideMovement(.75,145);
//                        commands.rotateControl(.15);
//                    })
////                                        .strafeRight(.001)
////                                        .forward(7.75)
//                    .splineToConstantHeading(new Vector2d(-45, -13), Math.toRadians(180))
//                    .splineToConstantHeading(new Vector2d(-52, -13 + stackDist), Math.toRadians(180))
//                    .forward(6,
//                            SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                    .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
//                        commands.clawControl(0,0);
//                    })
//                    .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
//                        commands.rotateControl(1);
//                    })
//                    .waitSeconds(0.5)
//                    .back(3)
//                    .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
//                        commands.reset();
//                    })
//                    //pick up
//                    .splineToSplineHeading(new Pose2d(-30, -13, Math.toRadians(0)), Math.toRadians(0))
//                    .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
//                        commands.reset2();
//                    })
//                    .splineToConstantHeading(new Vector2d(25,-13), Math.toRadians(0))
//                    .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
//                        commands.scorePositionMid();
//                    })
//                    .splineToConstantHeading(new Vector2d(46.4, -27 + tagDist), Math.toRadians(0))
//                    .forward(5)
//                    .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
//                        commands.clawControl(1,0);
//                    })
//                    .waitSeconds(.3)
//                    .addDisplacementMarker(() -> drive.followTrajectorySequenceAsync(cycle2))
//                    //score
//                    .build();
//
//            cycle3 = drive.trajectorySequenceBuilder(cycle2.end())
//                    //CYCLE 3
//                    .back(5)
//                    .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
//                        commands.reset();
//                    })
//                    .splineToSplineHeading(new Pose2d(20,-13, Math.toRadians(180)), Math.toRadians(180))
//                    .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
//                        commands.reset2();
//                    })
//                    .splineToConstantHeading(new Vector2d(-28, -13), Math.toRadians(180))
//                    .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
//                        commands.clawControl(1,0);
//                        commands.slideMovement(.75,145);
//                        commands.rotateControl(.15);
//                    })
////                                        .strafeRight(.001)
////                                        .forward(7.75)
//                    .splineToConstantHeading(new Vector2d(-40, -13), Math.toRadians(180))
//                    .splineToConstantHeading(new Vector2d(-51, -13), Math.toRadians(180))
//                    .splineToConstantHeading(new Vector2d(-53, -26), Math.toRadians(180))
//                    .splineToConstantHeading(new Vector2d(-58, -26 + stackDist), Math.toRadians(180))
//                    .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
//                        commands.clawControl(0,0);
//                    })
//                    .waitSeconds(0.5)
//                    .back(3)
//                    .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
//                        commands.reset();
//                    })
//                    //pick up
//                    .splineToConstantHeading(new Vector2d(-49, -13), Math.toRadians(0))
//                    .splineToSplineHeading(new Pose2d(-30, -13, Math.toRadians(0)), Math.toRadians(0))
//                    .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
//                        commands.reset2();
//                    })
//                    .splineToConstantHeading(new Vector2d(25,-13), Math.toRadians(0))
//                    .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
//                        commands.scorePositionMid();
//                    })
//                    .splineToConstantHeading(new Vector2d(46.4, -27 + tagDist), Math.toRadians(0))
//                    .forward(5)
//                    .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
//                        commands.clawControl(1,0);
//                    })
//                    .waitSeconds(.3)
//                    //score
//
//                    .build();

        waitForStart();
        sleep(100);
        if(opModeIsActive()) {
            drive.followTrajectorySequence(temp);
        }
    }
}
