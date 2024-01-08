package org.firstinspires.ftc.teamcode.SourceCode.Auton.Tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.trajectorysequence.TrajectorySequence;
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

@Autonomous(name = "redRightMovement", group = "Tests")
public class redRightMovement extends LinearOpMode {

    public OpenCvCamera webcam;

    double parkStrafe = 25;

    @Override
    public void runOpMode() throws InterruptedException {

        RedPipe2 pipeline = new RedPipe2();
        Mat next = new Mat();

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


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(6.4, -62.4, Math.toRadians(90));
        Pose2d stage2start = new Pose2d(-41.2, -46.8, Math.toRadians(-90));

        drive.setPoseEstimate(new Pose2d(6.4, -62.4, Math.toRadians(90)));

        TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d(6.4, -62.4, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(46.2, -30.6, Math.toRadians(180)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(50.4, -43.2, Math.toRadians(0)))
                .waitSeconds(1)
                .build();

        TrajectorySequence middle = drive.trajectorySequenceBuilder(new Pose2d(6.4, -62.4, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(12.2, -33.6, Math.toRadians(90)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(50, -35.2, Math.toRadians(0)))
                .waitSeconds(1)
                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(new Pose2d(6.4, -62.4, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(10.4, -30.6, Math.toRadians(180)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(50.4, -29.2, Math.toRadians(0)))
                .waitSeconds(1)
                .build();

        TrajectorySequence stage2 = drive.trajectorySequenceBuilder(stage2start)
                .lineToLinearHeading(new Pose2d(20, -10, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-58.4, -10, Math.toRadians(-180)))
                .waitSeconds(5)
                .build();

        TrajectorySequence stage3 = drive.trajectorySequenceBuilder(new Pose2d(-58.4, -12, Math.toRadians(-180)))
                .lineToLinearHeading(new Pose2d(20, -10, Math.toRadians(0)))
                .splineTo(new Vector2d(50, -35.2), 0)
                .waitSeconds(3)
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(new Pose2d(-50, -35.2, Math.toRadians(0)))
                .strafeRight(parkStrafe)
                .forward(10)
                .build();

        waitForStart();
        String location = pipeline.getLocation();

        //making sure everything else is okay
            if (location == "RIGHT") {
                parkStrafe = 15;
                stage2start = new Pose2d(50.4, -43.2, Math.toRadians(0));
                drive.followTrajectorySequence(right);
            } else if (location == "MIDDLE") {
                parkStrafe = 20;
                stage2start = new Pose2d(50, -35.2, Math.toRadians(0));
                drive.followTrajectorySequence(middle);
            } else if (location == "LEFT")
                parkStrafe = 25;
            stage2start = new Pose2d(50.4, -29.2, Math.toRadians(0));
            drive.followTrajectorySequence(left);
        }


    //Pose Storage for TeleOp

}
class RedPipe2 extends OpenCvPipeline {
    int correctlocation = 3;
    Mat mat = new Mat();
    Mat spare = new Mat();

    public enum Location {
        RIGHT,
        MIDDLE,
        LEFT
    }

    private Location location;
    static final Rect BMiddle = new Rect(
            new Point(145, 270),
            new Point(345, 200));
    static final Rect BRight = new Rect(
            new Point(450, 240),
            new Point(580, 370));
    static final double PERCENT_COLOR_THRESHOLD = 0.15;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(0, 100, 85);
        Scalar highHSV = new Scalar(10, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat middle = mat.submat(BMiddle);
        Mat right = mat.submat(BRight);

        double middleValue = Core.sumElems(middle).val[0] / BMiddle.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / BRight.area() / 255;

        middle.release();
        right.release();


        boolean onRight = rightValue > PERCENT_COLOR_THRESHOLD;
        boolean onMiddle = middleValue > PERCENT_COLOR_THRESHOLD;

        if (onMiddle) {
            correctlocation = 2;
        } else if (onRight) {
            correctlocation = 1;
        } else {
            correctlocation = 3;
        }
        Scalar False = new Scalar(0, 100, 85);
        Scalar True = new Scalar(10, 255, 255);


        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
        Imgproc.rectangle(mat, BRight, location == Location.RIGHT ? True : False);
        Imgproc.rectangle(mat, BMiddle, location == Location.MIDDLE ? True : False);
        return mat;
    }

    public String getLocation() {
        if(correctlocation == 1){
            return "RIGHT";
        } else if (correctlocation == 2) {
            return "Middle";
        } else {
            return "LEFT";
        }
    }
}