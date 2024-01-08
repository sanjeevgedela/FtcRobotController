package org.firstinspires.ftc.teamcode.SourceCode.Camera.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.trajectorysequence.TrajectorySequence;
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

@Autonomous(name = "pixeltester")
public class pixelset extends LinearOpMode {

    pixelpipeline pipeline = null;
    OpenCvCamera webcam = null;
    double FrameCenterX = 220;
    double ClosestPixelX = 0;

    //Define motors
    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftBack = null;
    public DcMotorEx rightBack = null;

    double movement;
    double rotation;
    double strafe;

    double y;
    double x;
    int angle;

    //Define servos
    public Servo rightClaw = null;
    public Servo leftClaw = null;
    public Servo rotateClaw = null;

    public void calc(){
        x = Math.copySign(Math.pow(-gamepad1.left_stick_y, 1), -gamepad1.left_stick_y);
        y = Math.copySign(Math.pow(-gamepad1.left_stick_x, 1), -gamepad1.left_stick_x);
    }

    public void clawControl() {
        if (gamepad1.left_trigger > 0) {
            leftClaw.setPosition(1);
        } else {
            leftClaw.setPosition(0);
        }
        if (gamepad1.right_trigger > 0) {
            rightClaw.setPosition(1);
        } else {
            rightClaw.setPosition(0);
        }
    }
    public void driverControl() {
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

    }


    @Override
    public void runOpMode() throws InterruptedException {
        pipeline = new pixelpipeline(telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);

        //Define All servos
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rotateClaw = hardwareMap.get(Servo.class, "rotateClaw");

        //Set Ranges
        rightClaw.scaleRange(0.1, 0.65);
        leftClaw.scaleRange(0, 0.5);
        rotateClaw.scaleRange(0.65, 1);

        //Define all movement motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightRear");

        //Set Zero Power Behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Reverse motors
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

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
        drive.getLocalizer().setPoseEstimate(new Pose2d(0,0,0));
        PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

        TrajectorySequence now = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(10,10,180 + angle))
                .build();

        // Set input bounds for the heading controller
        // Automatically handles overflow
        headingController.setInputBounds(-Math.PI, Math.PI);

        while(opModeInInit()){
            FtcDashboard.getInstance().startCameraStream(webcam, 120);
            ClosestPixelX = pipeline.getCenterX();
            pipeline.telemetry.update();
        }

        drive.setPoseEstimate(new Pose2d(0,0,0));


        waitForStart();

        while(opModeIsActive()){

            clawControl();

            ClosestPixelX = pipeline.getCenterX();
            telemetry.addData("CenterX", ClosestPixelX);
            telemetry.update();
            calc();

            if(ClosestPixelX > 220){
                angle = (int) ((ClosestPixelX - FrameCenterX) / -15);
                headingController.setTargetPosition(drive.getPoseEstimate().getHeading() - Math.toRadians(angle));
            } else if (ClosestPixelX < 220){
                angle = (int) ((FrameCenterX - ClosestPixelX) / 15);
                headingController.setTargetPosition(drive.getPoseEstimate().getHeading() + Math.toRadians(angle));
            }
            if(ClosestPixelX < 225 && ClosestPixelX > 215){
                angle = 0;
            }

            if(gamepad1.a){
                drive.followTrajectorySequence(now);
            }

            Pose2d driveDirection = new Pose2d(x, y, Math.toRadians(angle));

            //drive.setExternalHeading(headingController.getTargetPosition());

            //180 pixels = 15 degrees

//            if (FrameCenterX - ClosestPixelX > 350) {
//                drive.turn(Math.toRadians(5));
//            } else if (FrameCenterX - ClosestPixelX > 100) {
//                drive.turn(Math.toRadians(2));
//            } else if (FrameCenterX - ClosestPixelX > 15) {
//                drive.turn(Math.toRadians(1));
//            } else if (FrameCenterX - ClosestPixelX == 0) {
//                drive.turn(Math.toRadians(0));
//            } else if (FrameCenterX - ClosestPixelX < -15) {
//                drive.turn(Math.toRadians(-1));
//            } else if (FrameCenterX - ClosestPixelX < -100) {
//                drive.turn(Math.toRadians(-2));
//           } else if (FrameCenterX - ClosestPixelX < -350) {
//                drive.turn(Math.toRadians(-5));
//            }

            drive.setWeightedDrivePower(driveDirection);

            // Update the heading controller with our current heading
            headingController.update(drive.getPoseEstimate().getHeading());

            // Update the localizer
            drive.getLocalizer().update();

            // Print pose to telemetry
            telemetry.addData("x", drive.getPoseEstimate().getX());
            telemetry.addData("y", drive.getPoseEstimate().getY());
            telemetry.addData("heading", Math.toDegrees(drive.getPoseEstimate().getHeading()));
            telemetry.addData("CenterX", ClosestPixelX);
            telemetry.addData("angle", angle);
            telemetry.update();
        }

    }

    public static class pixelpipeline extends OpenCvPipeline {
        Telemetry telemetry;
        private Scalar lowerBound = new Scalar(0, 0, 200); // Lower bound for white color in HSV
        private Scalar upperBound = new Scalar(180, 30, 255); // Upper bound for white color in HSV
        int midx = 320;
        Scalar lowHSV = new Scalar(20, 70, 80); // lenient lower bound HSV for yellow
        Scalar highHSV = new Scalar(32, 255, 255); // lenient higher bound HSV for yellow
        int potentialAngle = 0;



        public pixelpipeline(Telemetry t) {
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

                // Convert the polygon to a bounding rectangle
                Rect rect = Imgproc.boundingRect(new MatOfPoint(approxCurve.toArray()));

                // Draw a rectangle around the detected region
                Imgproc.rectangle(input, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new Scalar(255, 0, 0), 2);
                areas.add(rect.width * rect.height);
                MidX.add(rect.x + rect.width / 2);

                Imgproc.line(input, new Point(midx, 235), new Point(midx, 245), new Scalar(0, 0, 255), 5);
                Imgproc.line(input, new Point(280, 235), new Point(280, 245), new Scalar(0, 0, 255), 5);
            }

            for (int i = 0; i < areas.size(); i++) {
                int max = 0;

                if (areas.get(i) > max) {
                    max = areas.get(i);
                    midx = MidX.get(i);

                }
            }

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
