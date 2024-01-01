package org.firstinspires.ftc.teamcode.SourceCode.Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.SourceCode.Camera.BluePipe;
import org.firstinspires.ftc.teamcode.SourceCode.Camera.RedPipe;
import org.firstinspires.ftc.teamcode.commands.Auto.IntakeOuttake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
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

@Autonomous(name = "RedRight")
public class redRight extends LinearOpMode {

    //Define motors
    public DcMotorEx leftSlide;
    public DcMotorEx rightSlide;

    public OpenCvCamera webcam;

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
        slideMovement(1, 0);
        clawControl(0, 0);
        rotateClaw.setPosition(1);
    }

    public void rotateControl(double rotate) {
        rotateClaw.setPosition(rotate);
    }

    public void clawControl(double left, double right) {
        leftClaw.setPosition(left);
        rightClaw.setPosition(right);
    }

    public void scorePositionLow(double rotate) {
        slideMovement(1, 175);
        rotateControl(rotate);
    }

    public void scorePositionMid() {
        slideMovement(1, 400);
        rotateControl(0.75);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        RedPipe0 pipeline = new RedPipe0(telemetry);
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
        rightClaw.scaleRange(0.1, 0.65);
        leftClaw.scaleRange(0, 0.5);
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

        Pose2d startPose = new Pose2d(9.4, -62.4, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d(9.4, -62.4, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    slideMovement(1, 40);
                    rotateControl(0);
                    clawControl(0, 0);
                })
                .lineToLinearHeading(new Pose2d(33.2, -30.6, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(0, 1);
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    reset();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    scorePositionLow(1);
                })
                .lineToLinearHeading(new Pose2d(50.4, -43.2, Math.toRadians(0)))
                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    clawControl(1, 0);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    reset();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    clawControl(1, 0);
                })
                .waitSeconds(0.3)
                .build();

        TrajectorySequence middle = drive.trajectorySequenceBuilder(new Pose2d(9.4, -62.4, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    slideMovement(1, 40);
                    rotateControl(0);
                    clawControl(0, 0);
                })
                .lineToLinearHeading(new Pose2d(12.2, -33.6, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(0, 1);
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    reset();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    scorePositionLow(1);
                })
                .lineToLinearHeading(new Pose2d(50, -35.2, Math.toRadians(0)))
                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    clawControl(1, 0);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    reset();
                })
                .waitSeconds(1)
                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(new Pose2d(10.4, -62.4, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    rotateControl(0);
                    clawControl(0, 0);
                })
                .lineToLinearHeading(new Pose2d(8.4, -34.6, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(0, 1);
                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    scorePositionLow(1);
                })
                .lineToLinearHeading(new Pose2d(51.4, -26.2, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    clawControl(1, 0);
                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    reset();
                })
                //see if grinds against backdrop
                .lineToLinearHeading(new Pose2d(20, -12, Math.toRadians(110)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    slideMovement(1, 20);
                    clawControl(1, 1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    rotateControl(0);
                })
                .lineToLinearHeading(new Pose2d(-53.5, -10, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-64.2, -11, Math.toRadians(180)))

                .UNSTABLE_addTemporalMarkerOffset(.2, () -> {
                    clawControl(0, 0);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    reset();
                })
                //change to -10
                //115
                .lineToLinearHeading(new Pose2d(20, -14, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    rotateControl(0.7);
                    slideMovement(1,300);
                })
                .lineToLinearHeading(new Pose2d(49.9, -34.2, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    clawControl(1, 1);
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    reset();
                })

                //see if grinds against backdrop
                .lineToLinearHeading(new Pose2d(20, -15, Math.toRadians(100)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    slideMovement(1, 12);
                    clawControl(1, 1);
                    //rotateControl(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    rotateControl(0);
                })
                .lineToLinearHeading(new Pose2d(-53.5, -10, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-64.2, -12, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(0, 0);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    reset();
                })
                //change to -10
                //115
                .lineToLinearHeading(new Pose2d(20, -16, Math.toRadians(120)))
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    scorePositionMid();
                })
                .lineToLinearHeading(new Pose2d(48.9, -38.2, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(1, 1);
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                    reset();
                })
                .waitSeconds(2)
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
        RedPipe0.Location0 detectedColor = pipeline.getLocation();

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
        }
    }
    public static class RedPipe0 extends OpenCvPipeline {
        Telemetry telemetry;
        Mat mat = new Mat();

        public enum Location0{
            RIGHT,
            MIDDLE,
            LEFT
        }

        private volatile Location0 location0;
        private static final Rect BMiddle = new Rect(
                new Point(145, 270),
                new Point(345, 200));
        private static final Rect BRight = new Rect(
                new Point(450, 240),
                new Point(580, 370));
        static final double PERCENT_COLOR_THRESHOLD = 0.15;
        public RedPipe0(Telemetry t) {telemetry = t;}
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

    public static class pixelpipeline extends OpenCvPipeline {
        Telemetry telemetry;
        private Scalar lowerBound = new Scalar(0, 0, 200); // Lower bound for white color in HSV
        private Scalar upperBound = new Scalar(180, 30, 255); // Upper bound for white color in HSV
        public static int closestPixelX;

        public pixelpipeline(Telemetry t) {telemetry = t;}
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
            int midx;

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
                MidX.add(rect.x + rect.width/2);
            }

            for (int i = 0; i < areas.size(); i++){
                int max = 0;

                if (areas.get(i) > max){
                    max = areas.get(i);
                    midx = MidX.get(i);

                }
            }
            Imgproc.
            // Return the processed frame
            return input;
        }
    }
        public TfodProcessor tfod;
        public VisionPortal visionPortal;

        public void initTfod(HardwareMap hardwareMap) {

            // Create the TensorFlow processor the easy way.
            tfod = TfodProcessor.easyCreateWithDefaults();

            // Create the vision portal the easy way.
            visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);

        }

        public int closestPixel() {

            List<Recognition> currentRecognitions = tfod.getRecognitions();
            List<Integer> Areas = null;
            Recognition ClosestPixel = null;
            int X = 0;


            // Step through the list of recognitions and display info for each one.
            for (Recognition recognition : currentRecognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                double y = (recognition.getTop() + recognition.getBottom()) / 2;
                double A = x * y;
                Areas.add((int) A);
            }

            for (int i = 0; i < Areas.size(); i++) {
                int min = 0;

                if (Areas.get(i) < min) {
                    min = Areas.get(i);
                    ClosestPixel = currentRecognitions.get(i);
                    X = (int) ClosestPixel.getLeft();
                }
                return X;
            }
            return X;
        }
    }