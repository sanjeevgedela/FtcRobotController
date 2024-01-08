package org.firstinspires.ftc.teamcode.SourceCode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.trajectorysequence.TrajectorySequence;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name = "redLeftStack", group = "Tests")
public class redLeftStack extends LinearOpMode {
    public DcMotorEx leftSlide;
    public DcMotorEx rightSlide;

    Pose2d stage2start = new Pose2d(-41.2, -46.8, Math.toRadians(-90));

    //Define servos
    public Servo rightClaw;
    public Servo leftClaw;
    public Servo rotateClaw;

    public void slideMovement(double power, int encPos){
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

    public void reset(){
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightClaw.setPosition(0);
        leftClaw.setPosition(0);
        rotateClaw.setPosition(1);
    }

    public void rotateControl(double rotate){
        rotateClaw.setPosition(rotate);
    }

    public void clawControl(double left, double right){
        leftClaw.setPosition(left);
        rightClaw.setPosition(right);
    }

    public void scorePositionLow(){
        slideMovement(1,250);
        rotateControl(1);
    }

    public void scorePositionMid(){
        slideMovement(1,400);
        rotateControl(1);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        RedPipe6 pipeline = new RedPipe6();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rotateClaw = hardwareMap.get(Servo.class, "rotateClaw");

        //Set Ranges
        rightClaw.scaleRange(0.1, 0.4);
        leftClaw.scaleRange(0, 0.4);
        rotateClaw.scaleRange(0.65, 1);
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

        Pose2d startPose = new Pose2d(6.4, -62.4, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    slideMovement(1,40);
                    rotateControl(0);
                    clawControl(0,0);
                })
                .lineToLinearHeading(new Pose2d(-34.2,-30.6, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(0,1);
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    reset();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    slideMovement(1,60);
                    clawControl(1,1);
                })
                .lineToLinearHeading(new Pose2d(-58.4,-10, Math.toRadians(-180)))
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    clawControl(0,0);
                })
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(20,-10, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    scorePositionLow();
                })
                .splineTo(new Vector2d(50, -43.2), 0)
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    clawControl(1,0);
                })
                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    reset();
                })
                .waitSeconds(1)
                .build();

        TrajectorySequence middle = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    slideMovement(1,40);
                    rotateControl(0);
                    clawControl(0,0);
                })
                .lineToLinearHeading(new Pose2d(-35.2,-13.1, Math.toRadians(270)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(0,1);
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    reset();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    slideMovement(1,60);
                    clawControl(1,1);
                })
                .lineToLinearHeading(new Pose2d(-58.4,-10, Math.toRadians(-180)))
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    clawControl(0,0);
                })
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(20,-10, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    scorePositionLow();
                })
                .splineTo(new Vector2d(50, -43.2), 0)
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    clawControl(1,0);
                })
                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    reset();
                })
                .waitSeconds(1)
                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    slideMovement(1,40);
                    rotateControl(0);
                    clawControl(0,0);
                })
                .lineToLinearHeading(new Pose2d(-56.2,-31, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(0,1);
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    reset();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    slideMovement(1,60);
                    clawControl(1,1);
                })
                .lineToLinearHeading(new Pose2d(-58.4,-10, Math.toRadians(-180)))
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    clawControl(0,0);
                })
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(20,-10, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    scorePositionLow();
                })
                .splineTo(new Vector2d(50, -43.2), 0)
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    clawControl(1,0);
                })
                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    reset();
                })
                .waitSeconds(1)
                .build();

        TrajectorySequence stage2 = drive.trajectorySequenceBuilder(stage2start)
                .lineToLinearHeading(new Pose2d(20, -10, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    slideMovement(1,60);
                    clawControl(1,1);
                })
                .lineToLinearHeading(new Pose2d(-58.4, -10, Math.toRadians(-180)))
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    slideMovement(1,60);
                    clawControl(0,0);
                })
                .waitSeconds(2)
                .build();

        TrajectorySequence stage3 = drive.trajectorySequenceBuilder(new Pose2d(-58.4,-12, Math.toRadians(-180)))
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    reset();
                })
                .lineToLinearHeading(new Pose2d(20, -10, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    scorePositionLow();
                })
                .splineTo(new Vector2d(50, -35.2), 0)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    clawControl(1,1);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    reset();
                })
                .waitSeconds(3)
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(new Pose2d(-50,-35.2, Math.toRadians(0)))
                .strafeRight(parkStrafe)
                .forward(10)
                .build();

        waitForStart();

        String location = pipeline.getLocation();

            if (location == "RIGHT") {
                parkStrafe = 15;
                stage2start = new Pose2d(50.4, -43.2, Math.toRadians(0));
                drive.followTrajectorySequence(right);
                drive.followTrajectorySequence(stage2);
                drive.followTrajectorySequence(stage3);
                drive.followTrajectorySequence(park);
                sleep(30000000);
            } else if (location == "MIDDLE") {
                parkStrafe = 20;
                stage2start = new Pose2d(50, -35.2, Math.toRadians(0));
                drive.followTrajectorySequence(middle);
                drive.followTrajectorySequence(stage2);
                drive.followTrajectorySequence(stage3);
                drive.followTrajectorySequence(park);
                sleep(30000000);
            } else if (location == "LEFT")
                parkStrafe = 25;
                stage2start = new Pose2d(50.4, -29.2, Math.toRadians(0));
                drive.followTrajectorySequence(left);
                drive.followTrajectorySequence(stage2);
                drive.followTrajectorySequence(stage3);
                drive.followTrajectorySequence(park);
                sleep(30000000);
    }
}
class RedPipe6 extends OpenCvPipeline {
    int correctlocation = 3;
    Mat mat = new Mat();

    public enum Location0 {
        RIGHT,
        MIDDLE,
        LEFT
    }

    private Location0 location;
    static final Rect BMiddle = new Rect(
            new Point(145, 270),
            new Point(345, 200));
    static final Rect BRight = new Rect(
            new Point(450, 240),
            new Point(580, 370));
    static final double PERCENT_COLOR_THRESHOLD = 0.15;
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input,mat,Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(0,100,85);
        Scalar highHSV = new Scalar(10,255,255);

        Core.inRange(mat,lowHSV,highHSV,mat);

        Mat middle = mat.submat(BMiddle);
        Mat right = mat.submat(BRight);

        double middleValue = Core.sumElems(middle).val[0] / BMiddle.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / BRight.area() / 255;

        middle.release();
        right.release();


        boolean onRight = rightValue > PERCENT_COLOR_THRESHOLD;
        boolean onMiddle = middleValue > PERCENT_COLOR_THRESHOLD;

        if (onMiddle){
            correctlocation = 2;
        }
        else if (onRight){
            correctlocation = 1;
        }
        else{
            correctlocation = 3;
        }
        Scalar False = new Scalar(0,100,85
        );
        Scalar True = new Scalar(10,255,255);


        Imgproc.cvtColor(mat,mat,Imgproc.COLOR_GRAY2RGB);
        Imgproc.rectangle(mat ,BRight , location == Location0.RIGHT? True :False);
        Imgproc.rectangle(mat,BMiddle, location == Location0.MIDDLE? True :False);
        return mat;
    }
    public String getLocation() {
        processFrame(mat);
        if(correctlocation == 1){
            return "RIGHT";
        } else if (correctlocation == 2) {
            return "Middle";
        } else {
            return "LEFT";
        }
    }
}