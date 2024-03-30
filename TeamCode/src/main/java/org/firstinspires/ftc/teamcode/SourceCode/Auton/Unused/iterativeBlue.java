package org.firstinspires.ftc.teamcode.SourceCode.Auton.Unused;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name = "A")
public class iterativeBlue extends OpMode {

    Equipment equip = new Equipment();
    AutonCommands commands;
    SampleMecanumDrive drive;
    TrajectorySequence STACK;

    private VisionPortal visionPortal;               // Used to manage the video source.
    WebcamName webcam1;
    apriltag tag;
    double dist;
    double tagDist = dist;
    double theta;
    double stackDist;

    TrajectorySequence right;
    TrajectorySequence middle;
    TrajectorySequence left;

    OpenCvCamera webcam;
    Pose2d startPose = new Pose2d(14.7, -62.4, Math.toRadians(90));

    public void relocalize(){
        theta = tag.relocalize();
        drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toRadians(0) - Math.toRadians(theta)));
    }

    @Override
    public void init() {
        equip = new Equipment();
        equip.initialize(Equipment.Mode.AUTON, hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        tag = new apriltag();
        tag.initAprilTag(visionPortal, hardwareMap);
        commands = new AutonCommands(equip);
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

        STACK = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.rotateControl(0);
                    commands.clawControl(0, 0);
                    commands.slideMovement(1,215);
                })
                .splineToSplineHeading(new Pose2d(32.7, -34, Math.toRadians(180)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(0,1);
                })
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.scorePositionLow();
                })
                .splineToSplineHeading(new Pose2d(51.4, -42.2, Math.toRadians(0)), Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(1,1);
                })
                .waitSeconds(0.3)
                .back(5)

                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.reset();
                })
                .splineToSplineHeading(new Pose2d(35,-13, Math.toRadians(180)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.reset2();
                })
                .splineToConstantHeading(new Vector2d(-28, -13), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(1,0);
                    commands.slideMovement(.75,220);
                    commands.rotateControl(.15);
                })
//                                        .strafeRight(.001)
//                                        .forward(7.75)
                .splineToConstantHeading(new Vector2d(-45, -13), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-52, -13 + stackDist), Math.toRadians(180))
                .forward(6,
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(0,0);
                })
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    commands.rotateControl(1);
                })
                .waitSeconds(0.5)
                .back(3)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.reset();
                })
                //pick up
                .splineToSplineHeading(new Pose2d(-30, -13, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.reset2();
                })
                .splineToConstantHeading(new Vector2d(25,-13), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.scorePositionMid();
                })
                .splineToConstantHeading(new Vector2d(46.4, -27 + tagDist), Math.toRadians(0))
                .forward(5)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(1,0);
                })
                .waitSeconds(.3)
                //score

                //CYCLE 2
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.reset();
                })
                .splineToSplineHeading(new Pose2d(35,-13, Math.toRadians(180)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.reset2();
                })
                .splineToConstantHeading(new Vector2d(-28, -13), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(1,0);
                    commands.slideMovement(.75,145);
                    commands.rotateControl(.15);
                })
//                                        .strafeRight(.001)
//                                        .forward(7.75)
                .splineToConstantHeading(new Vector2d(-45, -13), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-52, -13 + stackDist), Math.toRadians(180))
                .forward(6,
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(0,0);
                })
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    commands.rotateControl(1);
                })
                .waitSeconds(0.5)
                .back(3)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.reset();
                })
                //pick up
                .splineToSplineHeading(new Pose2d(-30, -13, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.reset2();
                })
                .splineToConstantHeading(new Vector2d(25,-13), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.scorePositionMid();
                })
                .splineToConstantHeading(new Vector2d(46.4, -27 + tagDist), Math.toRadians(0))
                .forward(5)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(1,0);
                })
                .waitSeconds(.3)
                //score

                //CYCLE 3
                .back(5)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.reset();
                })
                .splineToSplineHeading(new Pose2d(20,-13, Math.toRadians(180)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.reset2();
                })
                .splineToConstantHeading(new Vector2d(-28, -13), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(1,0);
                    commands.slideMovement(.75,145);
                    commands.rotateControl(.15);
                })
//                                        .strafeRight(.001)
//                                        .forward(7.75)
                .splineToConstantHeading(new Vector2d(-40, -13), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-51, -13), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-53, -26), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-58, -26 + stackDist), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(0,0);
                })
                .waitSeconds(0.5)
                .back(3)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.reset();
                })
                //pick up
                .splineToConstantHeading(new Vector2d(-49, -13), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-30, -13, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.reset2();
                })
                .splineToConstantHeading(new Vector2d(25,-13), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.scorePositionMid();
                })
                .splineToConstantHeading(new Vector2d(46.4, -27 + tagDist), Math.toRadians(0))
                .forward(5)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(1,0);
                })
                .waitSeconds(.3)
                //score

                .build();
    }

    @Override
    public void start(){
        drive.followTrajectorySequenceAsync(STACK);
    }

    @Override
    public void loop() {
        commands.pidUPDATE();
        tag.findTag(telemetry);
        dist = tag.calculate();
    }
}
