package org.firstinspires.ftc.teamcode.SourceCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SourceCode.Auton.BLUEbackdrop;
import org.firstinspires.ftc.teamcode.SourceCode.Auton.REDbackdrop;
import org.firstinspires.ftc.teamcode.SourceCode.Auton.apriltag;
import org.firstinspires.ftc.teamcode.SourceCode.Subsystems.AutonCommands;
import org.firstinspires.ftc.teamcode.SourceCode.Subsystems.Equipment;
import org.firstinspires.ftc.teamcode.SourceCode.Subsystems.HuskyStackDetection;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
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

@Autonomous(name = "moveBLUE")
public class movementBLUE extends LinearOpMode {
    SampleMecanumDrive drive;
    AutonCommands commands;
    Equipment equip;
    double tagDist = 0;
    double stackDist = 0;
    Pose2d startPose = new Pose2d(14.7, 62.4, Math.toRadians(-90));
    HuskyLens hl;
    HuskyStackDetection detect;
    TrajectorySequence part2;
    Pose2d temp2 = new Pose2d(0,0,0);
    TrajectorySequence part3;
    public OpenCvCamera webcam;


//    private VisionPortal visionPortal;               // Used to manage the video source.
//    apriltag tag;


    @Override
    public void runOpMode() throws InterruptedException {
        equip = new Equipment();
        detect = new HuskyStackDetection(telemetry, hl);
        drive = new SampleMecanumDrive(hardwareMap);
//        tag = new apriltag();
        equip.initialize(Equipment.Mode.AUTON, hardwareMap);

        detect.init(hardwareMap);
        commands = new AutonCommands(equip);
//        tag.initAprilTag(visionPortal, hardwareMap);
//        tag.setType(apriltag.DETECT.LEFT, apriltag.COLOR.RED);

        BluePipe0 pipeline = new BluePipe0(telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        drive.setPoseEstimate(new Pose2d(14.7, 65, Math.toRadians(-90)));
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

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.rotateControl(0);
                    commands.clawControl(0, 0);
                    commands.slideMovement(1,215);
                })
                .lineToLinearHeading(new Pose2d(10, 33.6, Math.toRadians(180)))
//                    .splineToConstantHeading(new Vector2d(14, -36), Math.toRadians(0))
//                    .splineToSplineHeading(new Pose2d(6, -29.6, Math.toRadians(180)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(0,1);
                })
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.scorePositionLow();
                })
                .lineToLinearHeading(new Pose2d(51.4, 29.6, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(1,1);
                })
                .waitSeconds(0.3)
                .back(5)
                .UNSTABLE_addTemporalMarkerOffset(.6, () -> {
                    commands.reset();
                })
                .splineToSplineHeading(new Pose2d(25,13, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.reset2();
                })
                //.waitSeconds(0.5)
                .forward(63,
                        SampleMecanumDrive.getVelocityConstraint(63, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                    .splineToConstantHeading(new Vector2d(-28, -13), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(1,0);
                    commands.slideMovement(1,270);
                    commands.rotateControl(0);
                })
                .splineToConstantHeading(new Vector2d(-48, 18), Math.toRadians(180))
                //
                .build();

        TrajectorySequence middle = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.rotateControl(0);
                    commands.clawControl(0, 0);
                    commands.slideMovement(1,215);
                })
//                .splineToSplineHeading(new Pose2d(32.7, -34, Math.toRadians(180)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(14, 36), Math.toRadians(0))
//                    .splineToSplineHeading(new Pose2d(6, -29.6, Math.toRadians(180)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(0,1);
                })
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.scorePositionLow();
                })
                .lineToLinearHeading(new Pose2d(51.4, 34.5, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(1,1);
                })
                .waitSeconds(0.3)
                .back(5)
                .UNSTABLE_addTemporalMarkerOffset(.6, () -> {
                    commands.reset();
                })
                .splineToSplineHeading(new Pose2d(25,13, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.reset2();
                })
                //.waitSeconds(0.5)
                .forward(63,
                        SampleMecanumDrive.getVelocityConstraint(63, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                    .splineToConstantHeading(new Vector2d(-28, -13), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(1,0);
                    commands.slideMovement(1,260);
                    commands.rotateControl(0);
                })
                .splineToConstantHeading(new Vector2d(-48, 18), Math.toRadians(180))
                .waitSeconds(10)
                //
                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.rotateControl(0);
                    commands.clawControl(0, 0);
                    commands.slideMovement(1,215);
                })
//                .splineToSplineHeading(new Pose2d(32.7, -34, Math.toRadians(180)), Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(14, -36), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(32.7, 34, Math.toRadians(180)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(0,1);
                })
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.scorePositionLow();
                })
                .lineToLinearHeading(new Pose2d(51.4, 42.2, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(1,1);
                })
                .waitSeconds(0.3)
                .back(5)
                .UNSTABLE_addTemporalMarkerOffset(.6, () -> {
                    commands.reset();
                })
                .splineToSplineHeading(new Pose2d(25,13, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.reset2();
                })
                //.waitSeconds(0.5)
                .forward(63,
                        SampleMecanumDrive.getVelocityConstraint(63, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                    .splineToConstantHeading(new Vector2d(-28, -13), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(1,0);
                    commands.slideMovement(1,270);
                    commands.rotateControl(0);
                })
                .splineToConstantHeading(new Vector2d(-48, 18), Math.toRadians(180))
                //
                .build();

        TrajectorySequence toStack = drive.trajectorySequenceBuilder(new Pose2d(46.4, 33, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(.6, () -> {
                    commands.reset();
                })
                .back(2.5,
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(25,13, Math.toRadians(180)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.reset2();
                })
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.clawControl(1,0);
                    commands.rotateControl(0);
                })
                //.waitSeconds(0.5)
                .forward(63,
                        SampleMecanumDrive.getVelocityConstraint(63, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                    .splineToConstantHeading(new Vector2d(-28, -13), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    commands.slideMovement(1,130);
                })
                .splineToConstantHeading(new Vector2d(-48, 22), Math.toRadians(210))
                .waitSeconds(10)
                //.addDisplacementMarker(() -> drive.followTrajectorySequenceAsync(part2))
                .build();


        while (!isStopRequested() && !opModeIsActive()) {

        }
        while (opModeInInit()){
            FtcDashboard.getInstance().startCameraStream(webcam, 120);
            pipeline.telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;
        sleep(100);
        BluePipe0.Location0 detected = pipeline.getLocation();
        webcam.closeCameraDevice();
        if(opModeIsActive() && !isStopRequested()) {
            switch (detected) {
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

            //drive.followTrajectorySequence(right);

            stackDist = detect.method();
            sleep(600);
            stackDist = detect.method();
            telemetry.addData("dist", stackDist);

            Trajectory next = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .strafeRight(-.0001 + stackDist)
                    .build();
            drive.followTrajectory(next);

            telemetry.update();

            part2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    //
//                .waitSeconds(.5)
//                .strafeRight(-.001 - stackDist)
//                .waitSeconds(.1)
                    .forward(12,
                            SampleMecanumDrive.getVelocityConstraint(23, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                    .splineToConstantHeading(new Vector2d(-60, -15 + stackDist), Math.toRadians(180),
//                            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                        commands.clawControl(0,0);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
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
                    .splineToSplineHeading(new Pose2d(-30, 13, Math.toRadians(0)), Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                        commands.reset2();
                    })
                    .splineToConstantHeading(new Vector2d(10,13), Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                        commands.scorePositionMid();
                    })
                    .splineToConstantHeading(new Vector2d(45.2, 34 + tagDist), Math.toRadians(0))
                    .forward(5)
                    .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                        commands.clawControl(1,0);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(.2, () -> {
                        commands.scorePositionHigh();
                    })
                    .waitSeconds(.5)
                    //score
                    .build();
            drive.followTrajectorySequence(part2);

            drive.followTrajectorySequence(toStack);

            stackDist = detect.method();
            sleep(400);
            stackDist = detect.method();
            telemetry.addData("stackDist", stackDist);
            telemetry.update();
            Trajectory next2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .strafeRight(-.0001 + stackDist)
                    .build();
            drive.followTrajectory(next2);

            part3 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    //
//                .waitSeconds(.5)
//                .strafeRight(-.001 - stackDist)
//                .waitSeconds(.1)
                    .forward(12,
                            SampleMecanumDrive.getVelocityConstraint(23, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                    .splineToConstantHeading(new Vector2d(-60, -17 + stackDist), Math.toRadians(180),
//                            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                        commands.clawControl(0,0);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
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
                    .splineToSplineHeading(new Pose2d(-30, 13, Math.toRadians(0)), Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                        commands.reset2();
                    })
                    .splineToConstantHeading(new Vector2d(10,13), Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                        commands.scorePositionMidv2();
                    })
                    .splineToConstantHeading(new Vector2d(45.4, 34 + tagDist), Math.toRadians(0))
                    .forward(5.5)
                    .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                        commands.clawControl(1,0);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(.2, () -> {
                        commands.scorePositionHigh();
                    })
                    .waitSeconds(.5)
                    .back(4,
                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))                    //score
                    .build();

            drive.followTrajectorySequence(part3);


            while (!isStopRequested() && opModeIsActive()) {
                drive.update();
                stackDist = detect.method();
//                tag.findTag(telemetry);
                telemetry.addData("stack", stackDist);
                telemetry.addData("tag", tagDist);
                telemetry.update();
                // Put your PID Update Function Here
            }

            //drive.followTrajectorySequenceAsync(part2);
        }
    }
    public static class BluePipe0 extends OpenCvPipeline {
        Telemetry telemetry;
        Mat mat = new Mat();

        public enum Location0{
            RIGHT,
            MIDDLE,
            LEFT
        }

        private volatile Location0 location0;
        static final Rect BMiddle = new Rect(
                new Point(145, 160),
                new Point(295, 60));
        static final Rect BRight = new Rect(
                new Point(430, 200),
                new Point(560, 90));

        static final double PERCENT_COLOR_THRESHOLD = 0.15;
        public BluePipe0(Telemetry t) {telemetry = t;}
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


            boolean onRight = rightValue >PERCENT_COLOR_THRESHOLD;
            boolean onMiddle = middleValue>PERCENT_COLOR_THRESHOLD;

            if (onMiddle){
                telemetry.addData("LOCATION!:","MIDDLE");
                location0 = Location0.MIDDLE;
            }
            else if (onRight){
                telemetry.addData("LOCATION!:","RIGHT");
                location0 = Location0.RIGHT;
            }
            else{
                telemetry.addData("LOCATION!:","LEFT");
                location0 = Location0.LEFT;
            }
            telemetry.update();
            Scalar False = new Scalar(0,100,85);
            Scalar True = new Scalar(10,255,255);


            Imgproc.cvtColor(mat,mat,Imgproc.COLOR_GRAY2RGB);
            Imgproc.rectangle(mat,BRight , location0 == Location0.RIGHT? True:False);
            Imgproc.rectangle(mat,BMiddle, location0 == Location0.MIDDLE? True :False);

            middle.release();
            right.release();
            return mat;
        }
        public Location0 getLocation(){
            return location0;
        }
    }
}
