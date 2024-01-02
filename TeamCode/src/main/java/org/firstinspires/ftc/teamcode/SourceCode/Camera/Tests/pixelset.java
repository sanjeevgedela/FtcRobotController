package org.firstinspires.ftc.teamcode.SourceCode.Camera.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SourceCode.Auton.redRight;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
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
    double FrameCenterX = 320;
    double ClosestPixelX = 0;
    public final double f = 453.46087317;

    @Override
    public void runOpMode() throws InterruptedException {
        pipeline = new pixelpipeline(telemetry);
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
        drive.getLocalizer().setPoseEstimate(new Pose2d(0,0,0));
        PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

        // Set input bounds for the heading controller
        // Automatically handles overflow
        headingController.setInputBounds(-Math.PI, Math.PI);
        double angle = 0;

        while(opModeInInit()){
            FtcDashboard.getInstance().startCameraStream(webcam, 120);
            ClosestPixelX = pipeline.getCenterX();
            pipeline.telemetry.update();
        }

        waitForStart();

        while(opModeIsActive()){

            FtcDashboard.getInstance().startCameraStream(webcam, 120);
            ClosestPixelX = pipeline.getCenterX();
            telemetry.addData("CenterX", ClosestPixelX);
            pipeline.telemetry.update();
            Pose2d driveDirection = new Pose2d(0,0, drive.getLocalizer().getPoseEstimate().getHeading());

            if(ClosestPixelX > 360){
                angle = (ClosestPixelX - 320)/180 * 15;
                driveDirection = new Pose2d(0, 0,drive.getLocalizer().getPoseEstimate().getHeading() - Math.toRadians(angle));
                //headingController.setTargetPosition(drive.getPoseEstimate().getHeading() + angle);
            } else if (ClosestPixelX < 280){
                angle = (320 - ClosestPixelX)/180 * 15;
                driveDirection = new Pose2d(0, 0,drive.getLocalizer().getPoseEstimate().getHeading() + Math.toRadians(angle));
                //headingController.setTargetPosition(drive.getPoseEstimate().getHeading() - angle);
            }



            //180 pixels = 15 degrees


//            if (FrameCenterX - ClosestPixelX + 100 > 350) {
//                drive.turn(Math.toRadians(-5));
//            } else if (FrameCenterX - ClosestPixelX > 100) {
//                drive.turn(Math.toRadians(-2));
//            } else if (FrameCenterX - ClosestPixelX > 5) {
//                drive.turn(Math.toRadians(-1));
//            } else if (FrameCenterX - ClosestPixelX == 0) {
//                drive.turn(Math.toRadians(0));
//            } else if (FrameCenterX - ClosestPixelX < -5) {
//                drive.turn(Math.toRadians(1));
//            } else if (FrameCenterX - ClosestPixelX < -100) {
//                drive.turn(Math.toRadians(2));
//            } else if (FrameCenterX - ClosestPixelX < -350) {
//                drive.turn(Math.toRadians(5));
//            }
            drive.setWeightedDrivePower(driveDirection);

            // Update the heading controller with our current heading
            headingController.update(drive.getPoseEstimate().getHeading());

            // Update he localizer
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
        int midx = 0;
        Scalar lowHSV = new Scalar(20, 70, 80); // lenient lower bound HSV for yellow
        Scalar highHSV = new Scalar(32, 255, 255); // lenient higher bound HSV for yellow


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
            Imgproc.line(input, new Point(midx-5,0), new Point(midx+5,0), new Scalar(255,255,255));
            // Return the processed frame
            return input;
        }
        public Integer getCenterX(){
            return midx;
        }
    }
}
