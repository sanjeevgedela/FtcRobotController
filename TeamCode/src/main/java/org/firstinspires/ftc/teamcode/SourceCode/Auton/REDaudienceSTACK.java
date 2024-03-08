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

import java.util.Queue;

@Autonomous(name = "REDaudienceSTACK")
public class REDaudienceSTACK extends LinearOpMode {
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
        RedPipe11 pipeline = new RedPipe11(telemetry);
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
        DriveConstants.MAX_VEL = 73;
        DriveConstants.MAX_ACCEL = 73;

        Pose2d startPose = new Pose2d(-38.69,-65, Math.toRadians(90));

        drive.setPoseEstimate(new Pose2d(-38.69,-65, Math.toRadians(90)));

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    rotateControl(0);
                    clawControl(0, 0);
                })
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    rotateControl(0.2);
                    slideMovement(1, 260);
                })
                .strafeLeft(5)
                .lineToLinearHeading(new Pose2d(-41, -32.6, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(0, 1);
                })
                //.splineTo(new Vector2d(52.4,-38.4), Math.toRadians(0))
                .waitSeconds(.3)
                .back(5)
                .turn(Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-58, -32.8, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    rotateControl(0.1);
                    slideMovement(1, 275);
                })
                .forward(6)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    clawControl(0,0);
                })
                .waitSeconds(0.3)
                .back(5)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    reset();
                })
                .lineToLinearHeading(new Pose2d(-45,-58.5,Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(25, -59), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    scorePositionLow();
                })
                .splineToConstantHeading(new Vector2d(30, -35), Math.toRadians(0))
                .waitSeconds(.3)
                .lineToLinearHeading(new Pose2d(47.4,-41.3, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(1, 0);
                })
                .back(6)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    scorePositionMid();
                })
                .strafeLeft(5)
                .forward(7)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(0, 1);
                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    reset();
                })
                .lineToLinearHeading(new Pose2d(24.1,-59.5,Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-34,-55.2),Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    slideMovement(1, 220);
                    clawControl(1,1);
                })
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    rotateControl(0);
                })
                .splineToConstantHeading(new Vector2d(-40, -34.4), Math.toRadians(180))
                .waitSeconds(.3)
                .forward(26)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(0,0);
                })
                .waitSeconds(0.3)
                .back(5)
                .lineToLinearHeading(new Pose2d(-45,-57,Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    reset();
                })
                .splineToConstantHeading(new Vector2d(15,-56),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    scorePositionMid();
                })
                .splineToConstantHeading(new Vector2d(35.4, -32), Math.toRadians(0))
                .waitSeconds(0.3)
                .forward(12)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(1, 1);
                })
                .waitSeconds(.3)
                .back(1)
                .waitSeconds(.3)
                .back(7)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    reset();
                })
                .strafeRight(18)
                .forward(3)
                .waitSeconds(1)
                .build();

        TrajectorySequence middle = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    rotateControl(0);
                    clawControl(0, 0);
                })
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    rotateControl(0.1);
                    slideMovement(1, 200);
                })
                .lineToLinearHeading(new Pose2d(-43.2, -37.5, Math.toRadians(90)))
                //.splineTo(new Vector2d(52.4,-38.4), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(0, 1);
                })
                .back(4)
                .lineToLinearHeading(new Pose2d(-58, -34, Math.toRadians(180)))
                .forward(5)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    clawControl(0,0);
                })
                .waitSeconds(0.3)
                .back(5)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    reset();
                })
                .lineToLinearHeading(new Pose2d(-45,-58.3,Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(25, -57.3), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    scorePositionLow();
                })
                .splineToConstantHeading(new Vector2d(30, -35), Math.toRadians(0))
                .waitSeconds(.3)
                .lineToLinearHeading(new Pose2d(50.4,-42, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(1, 0);
                })
                .back(6)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    scorePositionMid();
                })
                .strafeRight(4)
                .forward(6.2)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(0, 1);
                })
                .back(6)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    reset();
                })
                .lineToLinearHeading(new Pose2d(24.1,-57.5, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-34,-52.5), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    rotateControl(.2);
                    slideMovement(1, 100);
                    clawControl(0,1);
                })
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    rotateControl(0);
                })
                .splineToConstantHeading(new Vector2d(-40, -32.4), Math.toRadians(180))
                .waitSeconds(.3)
                .forward(22)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(0,0);
                })
                .waitSeconds(0.3)
                .back(5)
                .lineToLinearHeading(new Pose2d(-45,-56,Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    reset();
                })
                .splineToConstantHeading(new Vector2d(15,-55),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    scorePositionMid();
                })
                .splineToConstantHeading(new Vector2d(37.4, -42), Math.toRadians(0))
                .waitSeconds(0.3)
                .forward(12)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(1, 1);
                })
                .back(2)
                .waitSeconds(.3)
                .back(7)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    reset();
                })
                .strafeRight(10)
                .forward(3)
                .waitSeconds(1)
                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    rotateControl(0.2);
                    clawControl(0, 0);
                })
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    rotateControl(0);
                    slideMovement(1, 275);
                })
                .lineToLinearHeading(new Pose2d(-50, -37, Math.toRadians(90)))
                //.splineTo(new Vector2d(52.4,-38.4), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(0, 1);
                })
                .waitSeconds(.3)
                .lineToLinearHeading(new Pose2d(-50, -50, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-56, -34.8, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    rotateControl(0.1);
                    slideMovement(1, 280);
                })
                .forward(7.75)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    clawControl(0,0);
                })
                .waitSeconds(0.3)
                .back(5)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    reset();
                })
                .lineToLinearHeading(new Pose2d(-45,-58.3,Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(25, -57), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    scorePositionLow();
                })
                .splineToConstantHeading(new Vector2d(30, -35), Math.toRadians(0))
                .waitSeconds(.3)
                .lineToLinearHeading(new Pose2d(47.4,-28, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(1, 0);
                })
                .back(6)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    slideMovement(1,500);
                })
                .strafeRight(10)
                .forward(6.4)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(0, 1);
                })
                .back(2)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    reset();
                })
                .lineToLinearHeading(new Pose2d(24.1,-56.2,Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-34,-52),Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    slideMovement(1, 210);
                    clawControl(1,1);
                })
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    rotateControl(0.2);
                })
                .splineToConstantHeading(new Vector2d(-40, -35.9), Math.toRadians(180))
                .waitSeconds(.3)
                .forward(26)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(0,0);
                })
                .waitSeconds(0.3)
                .back(5)
                .lineToLinearHeading(new Pose2d(-45,-53,Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    reset();
                })
                .splineToConstantHeading(new Vector2d(15,-51),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    scorePositionMid();
                })
                .splineToConstantHeading(new Vector2d(35.4, -32), Math.toRadians(0))
                .waitSeconds(0.3)
                .forward(12)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(1, 1);
                })
                .waitSeconds(0.3)
                .back(1)
                .waitSeconds(.3)
                .back(8)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    reset();
                })
                .strafeRight(18)
                .forward(3)
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
        RedPipe11.Location11 detectedColor = pipeline.getLocation();

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
    public static class RedPipe11 extends OpenCvPipeline {
        Telemetry telemetry;
        Mat mat = new Mat();

        public enum Location11{
            RIGHT,
            MIDDLE,
            LEFT
        }

        private volatile Location11 location11;
        static final Rect BMiddle = new Rect(
                new Point(145, 160),
                new Point(295, 60));
        static final Rect BRight = new Rect(
                new Point(430, 200),
                new Point(560, 90));
        static final double PERCENT_COLOR_THRESHOLD = 0.15;
        public RedPipe11(Telemetry t) {telemetry = t;}
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
                location11 = Location11.MIDDLE;
            }
            else if (onRight){
                telemetry.addData("LOCATION!:","RIGHT");
                location11 = Location11.RIGHT;
            }
            else{
                telemetry.addData("LOCATION!:","LEFT");
                location11 = Location11.LEFT;
            }
            telemetry.update();
            Scalar False = new Scalar(0,100,85);
            Scalar True = new Scalar(10,255,255);


            Imgproc.cvtColor(mat,mat,Imgproc.COLOR_GRAY2RGB);
            Imgproc.rectangle(mat,BRight , location11 == Location11.RIGHT? True:False);
            Imgproc.rectangle(mat,BMiddle, location11 == Location11.MIDDLE? True :False);

            middle.release();
            right.release();
            return mat;
        }
        public Location11 getLocation(){
            return location11;
        }
    }
}