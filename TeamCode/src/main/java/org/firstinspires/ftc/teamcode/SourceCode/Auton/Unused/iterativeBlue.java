package org.firstinspires.ftc.teamcode.SourceCode.Auton.Unused;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.SourceCode.Auton.apriltag;
import org.firstinspires.ftc.teamcode.SourceCode.Subsystems.AutonCommands;
import org.firstinspires.ftc.teamcode.SourceCode.Subsystems.Equipment;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.concurrent.TimeUnit;

public class iterativeBlue extends OpMode {

    Equipment equip = new Equipment();
    AutonCommands commands = new AutonCommands(equip);
    SampleMecanumDrive drive;

    private VisionPortal visionPortal;               // Used to manage the video source.
    WebcamName webcam1;
    apriltag tag;
    double dist;
    double theta;

    TrajectorySequence right;
    TrajectorySequence middle;
    TrajectorySequence left;

    OpenCvCamera webcam;

    public void relocalize(){
        theta = tag.relocalize();
        drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toRadians(0) - Math.toRadians(theta)));
    }

    @Override
    public void init() {
        equip.initialize(Equipment.Mode.AUTON, hardwareMap, webcam);
        drive = new SampleMecanumDrive(hardwareMap);
        tag.initAprilTag(visionPortal, hardwareMap);
        Pose2d startPose = new Pose2d(-21.2,63.4, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        right = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    commands.rotateControl(0);
                    commands.clawControl(0, 0);
                })
                .lineToLinearHeading(new Pose2d(-35, 36, Math.toRadians(270)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(0, 1);
                })
                .back(7)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.rotateControl(0.20);
                    commands.slideMovement(1, 260);
                    commands.clawControl(0,1);
                })
                .lineToLinearHeading(new Pose2d(-30, 37.1, Math.toRadians(180)))
                .forward(14,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    commands.clawControl(0,0);
                })
                .waitSeconds(0.3)
                .back(3)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    commands.reset();
//                    tag.setType(apriltag.DETECT.RIGHT, apriltag.COLOR.BLUE);
//                    tag.findTag(telemetry);
                })
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-30, 60, Math.toRadians(0)))
                .waitSeconds(8)
                .splineToConstantHeading(new Vector2d(25, 59), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    commands.scorePositionLow();
                })
                .splineToConstantHeading(new Vector2d(35.5, 35.5), Math.toRadians(0))
//                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
//                    dist = tag.calculate();
//                })
                .waitSeconds(.5)
                .lineToLinearHeading(new Pose2d(67.5, 35.5 + dist, Math.toRadians(0)))
                //initial drop
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(1, 0);
                })
                .back(3)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.scorePositionMid();
                })
                .strafeLeft(5)
                .forward(4.5)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(0, 1);
                })
                .waitSeconds(0.3)
                .back(6)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    commands.reset();
                })
                .strafeLeft(20)
                .forward(8)
                .build();

        middle = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    commands.rotateControl(0);
                    commands.clawControl(0, 0);
                })
                .lineToLinearHeading(new Pose2d(-26.2, 36.5, Math.toRadians(270)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(0, 1);
                })
                .back(6)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.rotateControl(0.20);
                    commands.slideMovement(1, 280);
//                    tag.initAprilTag(visionPortal, hardwareMap);
                    commands.clawControl(0,1);
                })
                .lineToLinearHeading(new Pose2d(-30, 38.4, Math.toRadians(180)))
                .forward(16,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    commands. clawControl(0,0);
                })
                .waitSeconds(.3)
                .back(3)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    commands.rotateControl(1);
//                    tag.setType(apriltag.DETECT.MIDDLE, apriltag.COLOR.BLUE);
//                    tag.findTag(telemetry);
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    commands.reset();
                })
                .waitSeconds(8)
                .lineToLinearHeading(new Pose2d(-30, 60, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(25, 58), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    commands.scorePositionLow();
                })
                .splineToConstantHeading(new Vector2d(43, 40), Math.toRadians(0))
//                .UNSTABLE_addTemporalMarkerOffset(.7, () -> {
//                    dist = tag.calculate();
//                })
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(66.6, 40 + dist, Math.toRadians(0)))
                //initial drop
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(1, 0);
                })
                .back(3)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.scorePositionMid();
                })
                .strafeLeft(6)
                .forward(4.5)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(0, 1);
                })
                .waitSeconds(0.2)
                .back(7)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    commands.reset();
                })
                .strafeLeft(20)
                .forward(8)
                .build();

        left = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    commands.rotateControl(0.2);
                    commands.clawControl(0, 0);
                })
                .strafeRight(7)
                .lineToLinearHeading(new Pose2d(-22.2, 30.6, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(0, 1);
                })
                .back(4)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.rotateControl(0.15);
                    commands.slideMovement(1, 280);
                    commands.clawControl(0,1);

                })
                .lineToLinearHeading(new Pose2d(-30, 35.7, Math.toRadians(180)))
                .forward(17,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    commands.clawControl(0,0);
                })
                .waitSeconds(0.3)
                .back(3)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    commands.reset();
//                    tag.setType(apriltag.DETECT.LEFT, apriltag.COLOR.BLUE);
//                    tag.findTag(telemetry);
                })
                .waitSeconds(.5)
                .lineToLinearHeading(new Pose2d(-30, 61, Math.toRadians(0)))
                .waitSeconds(8)
                .splineToConstantHeading(new Vector2d(25, 59.5), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    commands.scorePositionLow();
                })
                .splineToConstantHeading(new Vector2d(43, 51), Math.toRadians(0))
//                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
//                    tag.findTag(telemetry);
//                    dist = tag.calculate();
//                })
                .waitSeconds(.5)
                .lineToLinearHeading(new Pose2d(66, 51 + dist, Math.toRadians(0)))
                //initial drop
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(1, 0);
                })
                .back(2)
                //.back(3)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.scorePositionMid();
                })
                .strafeRight(4)
                .forward(2.5)
                //.strafeRight(5)
                //.forward(4.5)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(0, 1);
                })
                .back(4)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.reset();
                })
                .strafeLeft(20)
                .build();
    }

    @Override
    public void start(){
        drive.followTrajectorySequenceAsync(left);
    }

    @Override
    public void loop() {
        commands.pidUPDATE();
        tag.findTag(telemetry);
        dist = tag.calculate();
    }
}
