package org.firstinspires.ftc.teamcode.SourceCode.TeleOp.Tests;
import android.view.VelocityTracker;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.SourceCode.Auton.REDbackdrop;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.opmode.LocalizationTest;
import org.firstinspires.ftc.teamcode.drive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.PersonalPID;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name="teztslidztele")
public class teztslidztele extends LinearOpMode {

    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftBack = null;
    public DcMotorEx rightBack = null;
    public DcMotorEx leftSlide = null;
    public DcMotorEx rightSlide = null;

    public Servo rightClaw = null;
    public Servo leftClaw = null;
    public Servo rotateClaw = null;
    public Servo plane = null;

    public RevBlinkinLedDriver LED = null;

    public VoltageSensor voltageSensor = null;

    public ColorSensor colorSensor = null;


    double movement;
    double rotation;
    double strafe;
    int target;

    pixelpipeline1 pipeline = null;
    OpenCvCamera webcam = null;
    double FrameCenterX = 180;
    double ClosestPixelX = 0;

    private enum DriveMode {
        NORMAL_CONTROL,
        ALIGN_TO_POINT,
        ALIGN_TO_PIXEL
    }

    private DriveMode currentMode = DriveMode.NORMAL_CONTROL;
    private Vector2d targetPositionRed = new Vector2d(72, -38.2);
    private Vector2d targetPositionBlue = new Vector2d(72, 38.2);
    private  Vector2d targetPosition = new Vector2d();

    double y;
    double x;
    int angle;

    public void calc(){
        x = Math.copySign(Math.pow(-gamepad1.left_stick_y, 1), -gamepad1.left_stick_y);
        y = Math.copySign(Math.pow(-gamepad1.left_stick_x, 1), -gamepad1.left_stick_x);
    }

    public void align(SampleMecanumDrive drive){

        Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();

        telemetry.addData("mode", currentMode);

        switch (currentMode) {
            case NORMAL_CONTROL:
                if (gamepad1.a) {
                    currentMode = DriveMode.ALIGN_TO_POINT;
                }
                if (gamepad1.x) {
                    currentMode = DriveMode.ALIGN_TO_PIXEL;
                }

                driverControl(drive);

                drive.getLocalizer().update();
                break;
            case ALIGN_TO_POINT:
                if (gamepad1.b) {
                    currentMode = DriveMode.NORMAL_CONTROL;
                }
                if (gamepad1.x) {
                    currentMode = DriveMode.ALIGN_TO_PIXEL;
                }


                drive.getLocalizer().update();
                break;

            case ALIGN_TO_PIXEL:
                if (gamepad1.a) {
                    currentMode = DriveMode.ALIGN_TO_POINT;
                }
                if (gamepad1.b) {
                    currentMode = DriveMode.NORMAL_CONTROL;
                }

                TrajectorySequence now = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .turn(Math.toRadians(angle))
                        .forward(14)
                        .build();

                ClosestPixelX = pipeline.getCenterX();

                if(ClosestPixelX > FrameCenterX){
                    angle = (int) ((ClosestPixelX - FrameCenterX) / -15);
                } else if (ClosestPixelX < FrameCenterX){
                    angle = (int) ((FrameCenterX - ClosestPixelX) / 15);
                }
                if((ClosestPixelX < (FrameCenterX + 5)) && (ClosestPixelX > (FrameCenterX - 5))){
                    angle = 0;
                }

                if(gamepad1.y){
                    drive.followTrajectorySequence(now);
                }

                Pose2d driveDirection1 = new Pose2d(x, y, Math.toRadians(angle));
                drive.setWeightedDrivePower(driveDirection1);

                drive.getLocalizer().update();
                break;
        }
    }

    public void driverControl(SampleMecanumDrive drive) {

        movement = gamepad1.left_stick_y;
        rotation = gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;

        double magnitude = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2));
        double direction = Math.atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        boolean precision = gamepad1.right_bumper;

        //INFO Increasing speed to a maximum of 1
        double lf = magnitude * Math.sin(direction + Math.PI / 4) + rotation;
        double lb = magnitude * Math.cos(direction + Math.PI / 4) + rotation;
        double rf = magnitude * Math.cos(direction + Math.PI / 4) - rotation;
        double rb = magnitude * Math.sin(direction + Math.PI / 4) - rotation;

        double hypot = Math.hypot(movement, strafe);
        double ratio;
        if (movement == 0 && strafe == 0)
            ratio = 1;
        else if (precision)
            ratio = hypot / (Math.max(Math.max(Math.max(Math.abs(lf), Math.abs(lb)), Math.abs(rb)), Math.abs(rf))) / 2;
        else
            ratio = hypot / (Math.max(Math.max(Math.max(Math.abs(lf), Math.abs(lb)), Math.abs(rb)), Math.abs(rf)));

        leftFront.setPower(ratio * lf);
        leftBack.setPower(ratio * lb);
        rightFront.setPower(ratio * rf);
        rightBack.setPower(ratio * rb);

        if (gamepad1.y) {
            drive.turn(Math.toRadians(180));
        }

    }

    public void clawControl() {

        if (gamepad2.left_trigger > 0 && rotateClaw.getPosition() == 0){
            rightClaw.setPosition(1);
        } else if (gamepad2.left_trigger > 0 && rotateClaw.getPosition() > 0) {
            rightClaw.setPosition(0.4);
        } else {
            rightClaw.setPosition(0);
        }

        if (gamepad2.right_trigger > 0 && rotateClaw.getPosition() == 0) {
          leftClaw.setPosition(1);
        } else if(gamepad2.right_trigger > 0 && rotateClaw.getPosition() > 0) {
            leftClaw.setPosition(0.65);
        } else {
            leftClaw.setPosition(0.4);
        }

    }

    public void rotateControl() {
        if (gamepad2.dpad_down){
            rotateClaw.setPosition(0);
        } else if (gamepad2.dpad_up){
            rotateClaw.setPosition(1);
        }
    }

    public void manual() {

        double power = -gamepad2.left_stick_y;

        if (Math.abs(power) < 0.1) {
            rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightSlide.setPower(0);
            leftSlide.setPower(0);
        } else {
            rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightSlide.setPower(power);
            leftSlide.setPower(power);
        }

//        if(gamepad2.left_stick_y > 0.1){
//            rightSlide.setPower(-1);
//            leftSlide.setPower(-1);
//
//        }else if(-gamepad2.left_stick_y > 0.1){
//            rightSlide.setPower(1);
//            leftSlide.setPower(1);
//        }else {
//            rightSlide.setPower(0);
//            leftSlide.setPower(0);
//        }
    }

    public void planeControl(){
        if(gamepad2.right_bumper){
            plane.setPosition(0);
        }
    }

    @Override
    public void runOpMode () {
        pipeline = new pixelpipeline1(telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
                webcam.setPipeline(pipeline);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.getLocalizer().setPoseEstimate(PoseStorage.currentPose);

        if(PoseStorage.currentPose.getY() > 0){
            targetPosition = targetPositionBlue;
        } else {
            targetPosition = targetPositionRed;
        }

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightRear");

        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rotateClaw = hardwareMap.get(Servo.class, "rotateClaw");
        plane = hardwareMap.get(Servo.class, "plane");

        // LED = hardwareMap.get(RevBlinkinLedDriver.class, "LED");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        plane.setDirection(Servo.Direction.REVERSE);

        leftClaw.scaleRange(0.55, 1);
        rightClaw.scaleRange(0.15, 0.4);
        rotateClaw.scaleRange(0.65, 1);
        leftClaw.setDirection(Servo.Direction.REVERSE);

        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();
        sleep(100);

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                FtcDashboard.getInstance().startCameraStream(webcam, 120);
                ClosestPixelX = pipeline.getCenterX();
                pipeline.telemetry.update();
                telemetry.addData("EncPosRight", rightSlide.getCurrentPosition());
                telemetry.addData("EncPosLeft", leftSlide.getCurrentPosition());
                telemetry.update();

                calc();
                align(drive);
                manual();
                clawControl();
                rotateControl();
                planeControl();
                driverControl(drive);
            }
        }
    }
    public static class pixelpipeline1 extends OpenCvPipeline {
        Telemetry telemetry;
        private Scalar lowerBound = new Scalar(0, 0, 200);
        private Scalar upperBound = new Scalar(180, 30, 255);
        int midx = 320;
        int potentialAngle = 0;

        public pixelpipeline1(Telemetry t) {
            telemetry = t;
        }

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);

            Core.inRange(input, lowerBound, upperBound, input);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(input, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            List<Integer> areas = new ArrayList<>();
            List<Integer> MidX = new ArrayList<>();

            for (MatOfPoint contour : contours) {
                MatOfPoint2f approxCurve = new MatOfPoint2f();
                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

                double epsilon = 0.02 * Imgproc.arcLength(contour2f, true);
                Imgproc.approxPolyDP(contour2f, approxCurve, epsilon, true);


                Rect rect = Imgproc.boundingRect(new MatOfPoint(approxCurve.toArray()));
                Imgproc.rectangle(input, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new Scalar(255, 0, 0), 2);
                areas.add(rect.width * rect.height);
                MidX.add(rect.x + rect.width / 2);
            }

            for (int i = 0; i < areas.size(); i++) {
                int max = 0;

                if (areas.get(i) > max) {
                    max = areas.get(i);
                    midx = MidX.get(i);

                }
            }

            Imgproc.line(input, new Point(midx, 235), new Point(midx, 245), new Scalar(0, 0, 255), 5);
            Imgproc.line(input, new Point(280, 235), new Point(280, 245), new Scalar(0, 0, 255), 5);

            // Return the processed frame
            potentialAngle = ((midx - 360) / 15);
            telemetry.addData("angle", potentialAngle);

            return input;
        }

        public Integer getCenterX() {
            return midx;
        }
    }
}