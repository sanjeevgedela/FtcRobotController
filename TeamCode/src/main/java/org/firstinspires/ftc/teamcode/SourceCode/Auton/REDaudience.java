package org.firstinspires.ftc.teamcode.SourceCode.Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

import java.util.Queue;

@Autonomous(name = "REDaudience")
public class REDaudience extends LinearOpMode {
    public DcMotorEx leftSlide;
    public DcMotorEx rightSlide;
    public OpenCvCamera webcam;

    public static double p = 0.007, i = 0, d = 0.0001, f = 0.001;
    int target;

    Pose2d stage2start = new Pose2d(-41.2, -46.8, Math.toRadians(-90));

    //Define servos
    public Servo rightClaw;
    public Servo leftClaw;
    public Servo rotateClaw;

    public void slideMovement(double power, int encPos) {
        rightSlide.setTargetPosition(encPos);
        leftSlide.setTargetPosition(encPos);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftControl(power);
    }

    public void liftControl(double power) {
        rightSlide.setPower(power);
        leftSlide.setPower(power);
    }

    public void reset() {
        slideMovement(1,0);
        clawControl(0, 0);
        rotateClaw.setPosition(1);
    }

    public void reset2(){
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void rotateControl(double rotate) {
        rotateClaw.setPosition(rotate);
    }

    public void clawControl(double left, double right) {
        leftClaw.setPosition(left);
        rightClaw.setPosition(right);
    }

    public void scorePositionLow() {
        rightSlide.setTargetPosition(600);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftControl(1);
        leftSlide.setTargetPosition(600);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftControl(1);
        rotateControl(1);
    }

    public void scorePositionMid() {
        rightSlide.setTargetPosition(850);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftControl(1);
        leftSlide.setTargetPosition(850);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftControl(1);
        rotateControl(1);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        RedPipe10 pipeline = new RedPipe10(telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
                sleep(1000);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rotateClaw = hardwareMap.get(Servo.class, "rotateClaw");

        //Set Ranges
        leftClaw.scaleRange(0.5, 1);
        rightClaw.scaleRange(0, 0.5);
        rotateClaw.scaleRange(0.65, 1);
        leftClaw.setDirection(Servo.Direction.REVERSE);
        //Define all Slide motors
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Reverse motors
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        //Set up encoders
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double parkStrafe = 25;

        Pose2d startPose = new Pose2d(-38.69,-65, Math.toRadians(90));

        drive.setPoseEstimate(new Pose2d(-38.69,-65, Math.toRadians(90)));

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    rotateControl(0);
                    clawControl(0, 0);
                })
                .strafeLeft(5)
                .lineToLinearHeading(new Pose2d(-36.5, -30.6, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(0, 1);
                })
                .back(4)
                .waitSeconds(10)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(0, 1);
                    slideMovement(1,270);
                    rotateControl(.1);
                })
                .lineToLinearHeading(new Pose2d(-46, -11, Math.toRadians(180)))
                .forward(12)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    clawControl(0,0);
                })
                .waitSeconds(0.5)
                .back(6)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    reset();
                })
                .turn(Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(40, -10, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    scorePositionLow();
                })
                .lineToLinearHeading(new Pose2d(52.4, -38.2, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(1, 0);
                })
                .back(6)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    scorePositionMid();
                })
                .strafeRight(5)
                .forward(6.2)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(0, 1);
                })
                .waitSeconds(0.5)
                .back(10)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    reset();
                })
                .waitSeconds(1)
                .strafeLeft(20)
                .forward(5)
                .build();

        TrajectorySequence middle = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    rotateControl(0);
                    clawControl(0, 0);
                })
                .waitSeconds(10)
                .lineToLinearHeading(new Pose2d(-42.2, -16.1, Math.toRadians(270)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(0, 1);
                })
                .back(5)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(0, 0);
                })
                .turn(-Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(0, 1);
                    rotateControl(.1);
                    slideMovement(1,275);
                })
                .forward(21)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    clawControl(0,0);
                })
                .waitSeconds(0.5)
                .back(6)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    reset();
                })
                .lineToLinearHeading(new Pose2d(16, -10, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    scorePositionLow();
                })
                .lineToLinearHeading(new Pose2d(50.4, -30.2, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(1, 1);
                })
                .back(4)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    reset();
                })
                .waitSeconds(1)
                .strafeLeft(12)
                .forward(5)
                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    rotateControl(0);
                    clawControl(0, 0);
                })
                .lineToLinearHeading(new Pose2d(-39.5, -35.35, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(0, 1);
                })
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-42, -14, Math.toRadians(180)))
                .waitSeconds(10)
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    rotateControl(0.1);
                    slideMovement(1, 280);
                })
                .forward(23)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(0,0);
                })
                .waitSeconds(0.5)
                .back(6)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    reset();
                })
                .turn(Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(35, -6, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    scorePositionLow();
                })
                .lineToLinearHeading(new Pose2d(52.4, -16, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(1, 0);
                })
                .back(2)
                .strafeRight(8)
                .forward(2)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(0, 1);
                })
                .waitSeconds(.3)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    reset();
                })
                .waitSeconds(1)
                .build();

        TrajectorySequence stage2 = drive.trajectorySequenceBuilder(new Pose2d(-38.69,-65, Math.toRadians(90)))
                .forward(15)
                .lineToLinearHeading(new Pose2d(-43, -34, Math.toRadians(90)))
                .forward(8)
                .lineToLinearHeading(new Pose2d(-60, -35.6, Math.toRadians(180)))

                .splineTo(new Vector2d(-20,-60), Math.toRadians(180))
                .splineTo(new Vector2d(10,-60), Math.toRadians(180))
                .splineTo(new Vector2d(38,-36), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(50.4, -43.2, Math.toRadians(180)))
                .waitSeconds(.1)

                .splineTo(new Vector2d(10,-60), Math.toRadians(180))
                .splineTo(new Vector2d(-30,-60), Math.toRadians(180))
                .splineTo(new Vector2d(-45,-35.6), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(-60, -35.6, Math.toRadians(0)))
                .waitSeconds(.1)

                .splineTo(new Vector2d(-20,-60), Math.toRadians(0))
                .splineTo(new Vector2d(10,-60), Math.toRadians(0))
                .splineTo(new Vector2d(38,-36), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(50.4, -43.2, Math.toRadians(180)))
                .waitSeconds(.1)

                .splineTo(new Vector2d(10,-60), Math.toRadians(180))
                .splineTo(new Vector2d(-30,-60), Math.toRadians(180))
                .splineTo(new Vector2d(-45,-35.6), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(-60, -35.6, Math.toRadians(0)))
                .waitSeconds(.1)

                .splineTo(new Vector2d(-20,-60), Math.toRadians(0))
                .splineTo(new Vector2d(10,-60), Math.toRadians(0))
                .splineTo(new Vector2d(38,-36), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(50.4, -43.2, Math.toRadians(180)))
                .waitSeconds(.1)

                //.lineToLinearHeading(new Pose2d(50.4,-43.2, Math.toRadians(0)))
                //.lineToLinearHeading(new Pose2d(20,-12, Math.toRadians(0)))
                //.lineToLinearHeading(new Pose2d(-58.4,-12, Math.toRadians(-180)))
                //.lineToLinearHeading(new Pose2d(20,-12, Math.toRadians(0)))
                //.splineTo(new Vector2d(50, -35.2), 0)
                .strafeLeft(20)
                .build();

        TrajectorySequence stage3 = drive.trajectorySequenceBuilder(new Pose2d(-58.4, -12, Math.toRadians(-180)))
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    reset();
                })
                .lineToLinearHeading(new Pose2d(20, -10, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    scorePositionLow();
                })
                .splineTo(new Vector2d(50, -35.2), 0)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    clawControl(1, 1);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    reset();
                })
                .waitSeconds(3)
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(new Pose2d(-50, -35.2, Math.toRadians(0)))
                .strafeRight(parkStrafe)
                .forward(10)
                .build();

        while (opModeInInit()){
            FtcDashboard.getInstance().startCameraStream(webcam, 120);
            pipeline.telemetry.update();
        }

        waitForStart();
        sleep(100);
        RedPipe10.Location10 detectedColor = pipeline.getLocation();

        while (opModeIsActive()) {

            FtcDashboard.getInstance().startCameraStream(webcam, 120);
            pipeline.telemetry.update();

            if (detectedColor != null) {
                switch (detectedColor) {
                    case RIGHT:
                        drive.followTrajectorySequence(right);
                        sleep(30000000);
                        break;
                    case MIDDLE:
                        drive.followTrajectorySequence(middle);
                        sleep(30000000);
                        break;
                    case LEFT:
                        drive.followTrajectorySequence(left);
                        sleep(30000000);
                        break;
                }
            }
            PoseStorage.currentPose = drive.getPoseEstimate();
        }
    }
    public static class RedPipe10 extends OpenCvPipeline {
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
        public RedPipe10(Telemetry t) {telemetry = t;}
        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input,mat,Imgproc.COLOR_RGB2HSV);
            Scalar lowHSV = new Scalar(0,100,85);
            Scalar highHSV = new Scalar(10,255,255);

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