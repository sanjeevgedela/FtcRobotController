package org.firstinspires.ftc.teamcode.SourceCode.Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SourceCode.Subsystems.AutonCommands;
import org.firstinspires.ftc.teamcode.SourceCode.Subsystems.Equipment;
import org.firstinspires.ftc.teamcode.SourceCode.Subsystems.HuskyStackDetection;
import org.firstinspires.ftc.teamcode.SourceCode.TeleOp.PIDvalues;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.PersonalPID;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
@Autonomous(name = "BLUEaudience")
public class BLUEaudience extends LinearOpMode {
    public DcMotorEx leftSlide;
    public DcMotorEx rightSlide;
    public OpenCvCamera webcam;
    //private VisionPortal visionPortal;
    TrajectorySequence stage2;
    Pose2d yellow;

    int dist;
    double stackDist = 0;
    HuskyLens hl;
    HuskyStackDetection detect;
    TrajectorySequence part2;
    TrajectorySequence part3;
    TrajectorySequence part4;
    AutonCommands commands;
    Equipment equip;
    //WebcamName webcam1;
    //apriltag tag;

    public static double p = 0.007, i = 0, d = 0.0001, f = 0.001;
    int target;

    Pose2d stage2start = new Pose2d(-41.2, -46.8, Math.toRadians(-90));



    @Override
    public void runOpMode() throws InterruptedException {
        equip = new Equipment();
        equip.initialize(Equipment.Mode.AUTON, hardwareMap);
        REDaudience.RedPipe10 pipeline = new REDaudience.RedPipe10(telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        commands = new AutonCommands(equip);
        detect = new HuskyStackDetection(telemetry, hl);
        detect.init(hardwareMap);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        double parkStrafe = 25;

        Pose2d startPose = new Pose2d(-39.7,65, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    commands.rotateControl(0);
                    commands.clawControl(0, 0);
                    commands.slideMovement(1, 250);
                })
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-37, 32, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(0, 1);
                })
                //.waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    commands.rotateControl(0);
                    commands.slideMovement(1, 300);
                })
                .lineToLinearHeading(new Pose2d(-42, 14.5, Math.toRadians(180)))
                .forward(17.1,
                        SampleMecanumDrive.getVelocityConstraint(18, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(0,0);
                })
                .UNSTABLE_addTemporalMarkerOffset(.2, () -> {
                    commands.rotateControl(1);
                })
                .waitSeconds(0.5)
                .back(8)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.reset();
                })
                //pick up
                .splineToSplineHeading(new Pose2d(-30, 13, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.reset2();
                })
                .splineToConstantHeading(new Vector2d(10,13), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.scorePositionMid();
                })
                .splineToConstantHeading(new Vector2d(46.3, 44.2), Math.toRadians(0))
                .forward(6.6)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(.8,.5);
                })
                .waitSeconds(1)
                .back(5)
                .build();

        TrajectorySequence middle = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    commands.rotateControl(0);
                    commands.clawControl(0, 0);
                    commands.slideMovement(1, 250);
                })
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-34, 12, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(0, 1);
                })
                .waitSeconds(.3)
                //.waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    commands.rotateControl(0);
                    commands.slideMovement(1, 310);
                })
                .lineToLinearHeading(new Pose2d(-42, 13.5, Math.toRadians(180)))
                .forward(19,
                        SampleMecanumDrive.getVelocityConstraint(16.4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(0,0);
                })
                .UNSTABLE_addTemporalMarkerOffset(.2, () -> {
                    commands.rotateControl(1);
                })
                .waitSeconds(0.5)
                .back(8)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.reset();
                })
                //pick up
                .splineToSplineHeading(new Pose2d(-30, 14, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.reset2();
                })
                .splineToConstantHeading(new Vector2d(10,14), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.scorePositionMid();
                })
                .splineToConstantHeading(new Vector2d(45.4, 42), Math.toRadians(0))
                .forward(6.4)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(.8,.5);
                })
                .waitSeconds(1)
                .back(5)
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    commands.rotateControl(0);
                    commands.clawControl(0, 0);
                    commands.slideMovement(1, 250);
                })
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-37.5, 35.35, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(0, 1);
                })
                //.waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    commands.rotateControl(0);
                    commands.slideMovement(1, 290);
                })
                .lineToLinearHeading(new Pose2d(-42, 19, Math.toRadians(180)))
                .forward(19.5,
                        SampleMecanumDrive.getVelocityConstraint(18, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(0,0);
                })
                .UNSTABLE_addTemporalMarkerOffset(.2, () -> {
                    commands.rotateControl(1);
                })
                .waitSeconds(1.5)
                .back(8)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.reset();
                })
                //pick up
                .splineToSplineHeading(new Pose2d(-30, 14, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.reset2();
                })
                .splineToConstantHeading(new Vector2d(10,10), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.scorePositionMid();
                })
                .splineToConstantHeading(new Vector2d(45.5, 23), Math.toRadians(0))
                .forward(6)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(.8,.5);
                })
                .waitSeconds(1)
                .back(4)
                .build();


        TrajectorySequence park = drive.trajectorySequenceBuilder(new Pose2d(-50, -35.2, Math.toRadians(0)))
                .strafeRight(parkStrafe)
                .forward(10)
                .build();

        TrajectorySequence toStack = drive.trajectorySequenceBuilder(new Pose2d(46.4, 33, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(.6, () -> {
                    commands.reset();
                })
                .back(2.5)
                .splineToSplineHeading(new Pose2d(25,15, Math.toRadians(180)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.reset2();
                })
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(1,0);
                    commands.rotateControl(0);
                })
                //.waitSeconds(0.5)
                .forward(63)
//                    .splineToConstantHeading(new Vector2d(-28, -13), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.slideMovement(1,170);
                })
                .splineToConstantHeading(new Vector2d(-45, 25), Math.toRadians(145))
                //.addDisplacementMarker(() -> drive.followTrajectorySequenceAsync(part2))
                .build();



        while (opModeInInit()){
            FtcDashboard.getInstance().startCameraStream(webcam, 120);
            pipeline.telemetry.update();
        }

        waitForStart();
        sleep(100);
        REDaudience.RedPipe10.Location10 detectedColor = pipeline.getLocation();
        webcam.closeCameraDevice();
        telemetry.update();

        switch (detectedColor) {
            case RIGHT:
                drive.followTrajectorySequence(right);
                break;
            case MIDDLE:
                drive.followTrajectorySequence(middle);
                break;
            case LEFT:
                drive.followTrajectorySequence(left);
                break;
        }

        stackDist = detect.method();
        sleep(400);
        stackDist = detect.method();
        drive.followTrajectorySequence(toStack);
        Trajectory next2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeRight(-.001 + stackDist)
                .build();

        drive.followTrajectory(next2);

        part3 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                //
//                .waitSeconds(.5)
//                .strafeRight(-.001 - stackDist)
//                .waitSeconds(.1)
                .forward(13.1,
                        SampleMecanumDrive.getVelocityConstraint(23, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                    .splineToConstantHeading(new Vector2d(-60, -17 + stackDist), Math.toRadians(180),
//                            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.slideMovement(1,90);

                })
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    commands.clawControl(0,0);
                })
                .waitSeconds(0.7)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.rotateControl(1);
                })
                //.forward(16)
                .back(3)
                .back(3)
                //.forward(16)
                .back(3)
                .back(3)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.reset();
                })
                //pick up
                .splineToSplineHeading(new Pose2d(-30, 13, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.reset2();
                })
                .splineToConstantHeading(new Vector2d(10,13), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.scorePositionMidv2();
                })
                .splineToConstantHeading(new Vector2d(48, 34), Math.toRadians(0))
                .forward(5.5)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(1,0);
                })
                .UNSTABLE_addTemporalMarkerOffset(.2, () -> {
                    commands.scorePositionHigh();
                })
                .waitSeconds(.13)
                .back(2,
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))                    //score
                .build();

        drive.followTrajectorySequence(part3);

      /*  drive.followTrajectorySequence(toStack);

        stackDist = detect.method();
        sleep(600);
        stackDist = detect.method();
        telemetry.addData("dist", stackDist);

        Trajectory next1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeRight(-.0001 + stackDist)
                .build();
        drive.followTrajectory(next1);

        part3 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                //
//                .waitSeconds(.5)
//                .strafeRight(-.001 - stackDist)
//                .waitSeconds(.1)
//                    .forward(17.5,
//                            SampleMecanumDrive.getVelocityConstraint(23, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.slideMovement(1,20);
                })
                .forward(19.6,
                        SampleMecanumDrive.getVelocityConstraint(18, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.slideMovement(1, 80);
                })
                .UNSTABLE_addTemporalMarkerOffset(.2, () -> {
                    commands.clawControl(0,0);
                })
                .UNSTABLE_addTemporalMarkerOffset(.4, () -> {
                    commands.rotateControl(1);
                })
                .waitSeconds(0.5)
                //.forward(16)
                .back(3)
                .back(3)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.reset();
                })
                //pick up
                .splineToSplineHeading(new Pose2d(-30, -13, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.reset2();
                })
                .splineToConstantHeading(new Vector2d(10,-13), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.scorePositionMidv2();
                })
                .splineToConstantHeading(new Vector2d(45.4, -34), Math.toRadians(0))
                .forward(5)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(1,0);
                })
                .UNSTABLE_addTemporalMarkerOffset(.2, () -> {
                    commands.scorePositionHigh();
                })
                .waitSeconds(.4)
                .back(4)                    //score
                .build();

        drive.followTrajectorySequence(part3); */


        PoseStorage.currentPose = drive.getPoseEstimate();
    }
    public static class BluePipe10 extends OpenCvPipeline {
        Telemetry telemetry;
        Mat mat = new Mat();

        public enum Location10{
            RIGHT,
            MIDDLE,
            LEFT
        }

        private volatile Location10 location10;
        static final Rect BMiddle = new Rect(
                new Point(145, 160),
                new Point(295, 60));
        static final Rect BRight = new Rect(
                new Point(430, 200),
                new Point(560, 90));
        static final double PERCENT_COLOR_THRESHOLD = 0.15;
        public BluePipe10(Telemetry t) {telemetry = t;}
        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input,mat,Imgproc.COLOR_RGB2HSV);
            Scalar lowHSV = new Scalar(100, 150, 0);
            Scalar highHSV = new Scalar(120, 255, 255);

            Core.inRange(mat,lowHSV,highHSV,mat);

            Mat middle = mat.submat(BMiddle);
            Mat right = mat.submat(BRight);

            double middleValue = Core.sumElems(middle).val[0] / BMiddle.area() / 255;
            double rightValue = Core.sumElems(right).val[0] / BRight.area() / 255;

            telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
            telemetry.addData("Middle raw value", (int) Core.sumElems(middle).val[0]);
            telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");
            telemetry.addData("Middle percentage", Math.round(middleValue * 100) + "%");


            boolean onRight = rightValue > PERCENT_COLOR_THRESHOLD;
            boolean onMiddle = middleValue > PERCENT_COLOR_THRESHOLD;

            if (onMiddle){
                telemetry.addData("LOCATION!:","MIDDLE");
                location10 = Location10.MIDDLE;
            }
            else if (onRight){
                telemetry.addData("LOCATION!:","RIGHT");
                location10 = Location10.RIGHT;
            }
            else{
                telemetry.addData("LOCATION!:","LEFT");
                location10 = Location10.LEFT;
            }
            telemetry.update();
            Scalar False = new Scalar(0,100,85);
            Scalar True = new Scalar(10,255,255);


            Imgproc.cvtColor(mat,mat,Imgproc.COLOR_GRAY2RGB);
            Imgproc.rectangle(mat,BRight , location10 == Location10.RIGHT? True:False);
            Imgproc.rectangle(mat,BMiddle, location10 == Location10.MIDDLE? True :False);

            middle.release();
            right.release();
            return mat;
        }
        public Location10 getLocation(){
            return location10;
        }
    }
}