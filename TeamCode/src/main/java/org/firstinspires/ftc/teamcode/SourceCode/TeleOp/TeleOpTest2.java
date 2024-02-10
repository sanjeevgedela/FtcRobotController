package org.firstinspires.ftc.teamcode.SourceCode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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
@TeleOp(name="OFFICIAL")
public class TeleOpTest2 extends LinearOpMode {

    private PersonalPID controller;

    public static double p = 0.006, i = 0, d = 0.0003, f = 0.0013;

    //Define motors
    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftBack = null;
    public DcMotorEx rightBack = null;
    public DcMotorEx leftSlide = null;
    public DcMotorEx rightSlide = null;

    //Define servos
    public Servo rightClaw = null;
    public Servo leftClaw = null;
    public Servo rotateClaw = null;
    public Servo plane = null;

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
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);
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
        // Read pose
        Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();

        // Declare a drive direction
        // Pose representing desired x, y, and angular velocity
        Pose2d driveDirection = new Pose2d();

        telemetry.addData("mode", currentMode);

        // Declare telemetry packet for dashboard field drawing
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        switch (currentMode) {
            case NORMAL_CONTROL:
                // Switch into alignment mode if `a` is pressed
                if (gamepad1.a) {
                    currentMode = DriveMode.ALIGN_TO_POINT;
                }
                if (gamepad1.x) {
                    currentMode = DriveMode.ALIGN_TO_PIXEL;
                }

                // Standard teleop control
                // Convert gamepad input into desired pose velocity
                driverControl(drive);
                headingController.update(poseEstimate.getHeading());

                // Update he localizer
                drive.getLocalizer().update();
                break;
            case ALIGN_TO_POINT:
                // Switch back into normal driver control mode if `b` is pressed
                if (gamepad1.b) {
                    currentMode = DriveMode.NORMAL_CONTROL;
                }
                if (gamepad1.x) {
                    currentMode = DriveMode.ALIGN_TO_PIXEL;
                }

                // Create a vector from the gamepad x/y inputs which is the field relative movement
                // Then, rotate that vector by the inverse of that heading for field centric control
                Vector2d fieldFrameInput = new Vector2d(
                        x,
                        y
                );
                Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());

                // Difference between the target vector and the bot's position
                Vector2d difference = targetPosition.minus(poseEstimate.vec());
                // Obtain the target angle for feedback and derivative for feedforward
                double theta = difference.angle();

                // Not technically omega because its power. This is the derivative of atan2
                double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

                // Set the target heading for the heading controller to our desired angle
                headingController.setTargetPosition(theta);

                // Set desired angular velocity to the heading controller output + angular
                // velocity feedforward
                double headingInput = (headingController.update(poseEstimate.getHeading())
                        * DriveConstants.kV + thetaFF)
                        * DriveConstants.TRACK_WIDTH;

                // Combine the field centric x/y velocity with our derived angular velocity
                driveDirection = new Pose2d(
                        robotFrameInput,
                        headingInput
                );
                drive.setWeightedDrivePower(driveDirection);
                headingController.update(poseEstimate.getHeading());

                // Update he localizer
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
                    headingController.setTargetPosition(drive.getPoseEstimate().getHeading() - Math.toRadians(angle));
                } else if (ClosestPixelX < FrameCenterX){
                    angle = (int) ((FrameCenterX - ClosestPixelX) / 15);
                    headingController.setTargetPosition(drive.getPoseEstimate().getHeading() + Math.toRadians(angle));
                }
                if((ClosestPixelX < (FrameCenterX + 5)) && (ClosestPixelX > (FrameCenterX - 5))){
                    angle = 0;
                }

                if(gamepad1.y){
                    drive.followTrajectorySequence(now);
                }

                Pose2d driveDirection1 = new Pose2d(x, y, Math.toRadians(angle));
                drive.setWeightedDrivePower(driveDirection1);
                headingController.update(poseEstimate.getHeading());

                // Update the localizer
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

        if ((gamepad2.left_trigger > 0)) {
            rightClaw.setPosition(1);
        } else {
            rightClaw.setPosition(0);
        }
        if ((gamepad2.right_trigger > 0)) {
            leftClaw.setPosition(1);
        } else {
            leftClaw.setPosition(0);
        }
    }

    public void rotateControl() {
        if (gamepad2.dpad_down){
            rotateClaw.setPosition(0);
        }
    }

    //controls lift motors
    //controls lift motors
    private void liftControl(double power) {
        rightSlide.setPower(power);
        leftSlide.setPower(power);
    }

    private void slideControl(){
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //    //Lifts the lift
    private void cascadinglift() {

        controller.setPIDF(p, i, d, f);
        int armPos = rightSlide.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        rightSlide.setPower(pid);
        leftSlide.setPower(pid);

        telemetry.addData("pid", pid);
        telemetry.addData("target", target);

        if((rightSlide.getTargetPosition() == rightSlide.getCurrentPosition()) && (rightSlide.getCurrentPosition() == 0)){
          //  rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      }

        if (gamepad2.y) {
            target = 600;
            rotateClaw.setPosition(1);

        } else if (gamepad2.b) {
            target = 1200;
            rotateClaw.setPosition(1);

        } else if (gamepad2.a) {
            target = 1800;
            rotateClaw.setPosition(1);

        } else if (gamepad2.x) {
            target = 0;
            rotateClaw.setPosition(1);

        } else if (gamepad2.left_stick_y > 0.2) {
            pid = pid + 0.01;

        } else if (gamepad2.left_stick_y < -0.2) {
            pid = pid + 0.01;
        }
    }

    public void planeControl(){
        if(gamepad2.right_bumper){
            plane.setPosition(0);
        }
    }

    @Override
    public void runOpMode () {
        controller = new PersonalPID(p, i, d, f);
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

        //Create Mecanum Drive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.getLocalizer().setPoseEstimate(PoseStorage.currentPose);

        if(PoseStorage.currentPose.getY() > 0){
            targetPosition = targetPositionBlue;
        } else {
            targetPosition = targetPositionRed;
        }

        // Set input bounds for the heading controller
        // Automatically handles overflow
        headingController.setInputBounds(-Math.PI, Math.PI);

        //Define all movement motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightRear");

        //Define all Slide motors
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        //Define All servos
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rotateClaw = hardwareMap.get(Servo.class, "rotateClaw");
        plane = hardwareMap.get(Servo.class, "plane");


        //Set Zero Power Behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Reverse motors
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        plane.setDirection(Servo.Direction.FORWARD);


        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set Ranges
        rightClaw.scaleRange(0.1, 0.4);
        leftClaw.scaleRange(0.1, 0.4);
        rotateClaw.scaleRange(0.65, 1);

        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();
        sleep(100);

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                FtcDashboard.getInstance().startCameraStream(webcam, 120);
                ClosestPixelX = pipeline.getCenterX();
                pipeline.telemetry.update();
                telemetry.addData("EncPos", rightSlide.getCurrentPosition());
                telemetry.update();

                calc();
                align(drive);
                cascadinglift();
                clawControl();
                rotateControl();
                planeControl();
            }
        }
    }
    public static class pixelpipeline1 extends OpenCvPipeline {
        Telemetry telemetry;
        private Scalar lowerBound = new Scalar(0, 0, 200); // Lower bound for white color in HSV
        private Scalar upperBound = new Scalar(180, 30, 255); // Upper bound for white color in HSV
        int midx = 320;
        Scalar lowHSV = new Scalar(20, 70, 80); // lenient lower bound HSV for yellow
        Scalar highHSV = new Scalar(32, 255, 255); // lenient higher bound HSV for yellow
        int potentialAngle = 0;
        //private List<MatOfPoint> hexagons;


        public pixelpipeline1(Telemetry t) {
            telemetry = t;
        }

        @Override
        public Mat processFrame(Mat input) {
            // Convert the input frame to the HSV color space
            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);

            // Threshold the image to find pixels within the specified color range
            Core.inRange(input, lowerBound, upperBound, input);

            // Find contours in the binary image
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(input, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            List<Integer> areas = new ArrayList<>();
            List<Integer> MidX = new ArrayList<>();

            // Iterate through the contours and draw rectangles around them
            for (MatOfPoint contour : contours) {
                MatOfPoint2f approxCurve = new MatOfPoint2f();
                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

                // Approximate the contour with a polygon
                double epsilon = 0.02 * Imgproc.arcLength(contour2f, true);
                Imgproc.approxPolyDP(contour2f, approxCurve, epsilon, true);

                //if (approxCurve.total() == 6) {
                    //hexagons.add(contour);
                    // Convert the polygon to a bounding rectangle
                    Rect rect = Imgproc.boundingRect(new MatOfPoint(approxCurve.toArray()));
                    // Draw a rectangle around the detected region
                    Imgproc.rectangle(input, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new Scalar(255, 0, 0), 2);
                    areas.add(rect.width * rect.height);
                    MidX.add(rect.x + rect.width / 2);
                //}
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
