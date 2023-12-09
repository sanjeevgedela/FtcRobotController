package org.firstinspires.ftc.teamcode.SourceCode.Auton.Tests;

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
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.SourceCode.Camera.RedPipe;
import org.firstinspires.ftc.teamcode.commands.Auto.IntakeOuttake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
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

import java.util.List;

@Autonomous(name = "redLeftMovement", group = "Tests")
public class redLeftMovement extends LinearOpMode {
    IntakeOuttake equip = new IntakeOuttake();
    RedPipe4 pipeline = new RedPipe4(telemetry);
    double parkStrafe;

    public OpenCvCamera webcam;


    @Override
    public void runOpMode() throws InterruptedException {

        equip.map(hardwareMap);
        equip.initialize();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-16.4,-62.4, Math.toRadians(90));
        Pose2d stage2start = new Pose2d(-41.2, -46.8, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-34.2,-30.6, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-58.4,-10, Math.toRadians(-180)))
                .lineToLinearHeading(new Pose2d(20,-10, Math.toRadians(0)))
                .splineTo(new Vector2d(50, -43.2), 0)
                .build();

        TrajectorySequence middle = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-35.2,-13.1, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(-58.4,-10, Math.toRadians(-180)))
                .lineToLinearHeading(new Pose2d(20,-10, Math.toRadians(0)))
                .splineTo(new Vector2d(50, -35.2), 0)
                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-56.2,-31, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-58.4,-10, Math.toRadians(-180)))
                .lineToLinearHeading(new Pose2d(20,-10, Math.toRadians(0)))
                .splineTo(new Vector2d(50, -35.2), 0)
                .build();

        TrajectorySequence stage2 = drive.trajectorySequenceBuilder(stage2start)
                .lineToLinearHeading(new Pose2d(20,-12, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-58.4,-12, Math.toRadians(-180)))
                .waitSeconds(5)
                .build();

        TrajectorySequence stage3 = drive.trajectorySequenceBuilder(new Pose2d(-58.4,-12, Math.toRadians(-180)))
                .lineToLinearHeading(new Pose2d(20,-12, Math.toRadians(0)))
                .splineTo(new Vector2d(50, -35.2), 0)
                .waitSeconds(3)
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(new Pose2d(-50,-35.2, Math.toRadians(0)))
                .strafeRight(parkStrafe)
                .forward(10)
                .build();

        waitForStart();

        //making sure everything else is okay+
        equip.reset();
        sleep(50);

        //Following the path to determine if its set up correctly
        switch(pipeline.getLocation()){
            case RIGHT:
                parkStrafe = 15;
                stage2start = new Pose2d(50.4,-43.2, Math.toRadians(0));
                drive.followTrajectorySequence(right);
                break;
            case MIDDLE:
                parkStrafe = 20;
                stage2start = new Pose2d(50,-35.2, Math.toRadians(0));
                drive.followTrajectorySequence(middle);
                break;
            case LEFT:
                parkStrafe = 25;
                stage2start = new Pose2d(50.4,-29.2, Math.toRadians(0));
                drive.followTrajectorySequence(left);
                break;
        }
        //Pose Storage for TeleOp
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
class RedPipe4 extends OpenCvPipeline {
    Telemetry telemetry;
    int correctlocation = 3;
    Mat mat = new Mat();

    public enum Location4 {
        RIGHT,
        MIDDLE,
        LEFT
    }

    public Location4 location;
    final Rect BMiddle = new Rect(
            new Point(145, 270),
            new Point(345, 200));
    final Rect BRight = new Rect(
            new Point(450, 240),
            new Point(580, 370));
    static final double PERCENT_COLOR_THRESHOLD = 0.15;
    public RedPipe4(Telemetry t) {telemetry = t;}
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

        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Middle raw value", (int) Core.sumElems(middle).val[0]);
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");
        telemetry.addData("Middle percentage", Math.round(middleValue * 100) + "%");


        boolean onRight = rightValue >PERCENT_COLOR_THRESHOLD;
        boolean onMiddle = middleValue>PERCENT_COLOR_THRESHOLD;

        if (onMiddle){
            correctlocation = 2;
            telemetry.addData("LOCATION!:","MIDDLE");
        }
        else if (onRight){
            correctlocation = 1;
            telemetry.addData("LOCATION!:","RIGHT");
        }
        else{
            correctlocation = 3;
            telemetry.addData("LOCATION!:","LEFT");
        }
        telemetry.update();
        Scalar False = new Scalar(0,100,85
        );
        Scalar True = new Scalar(10,255,255);


        Imgproc.cvtColor(mat,mat,Imgproc.COLOR_GRAY2RGB);
        Imgproc.rectangle(mat ,BRight , location == Location4.RIGHT? True:False);
        Imgproc.rectangle(mat,BMiddle, location == Location4.MIDDLE? True :False);
        return mat;
    }
    public Location4 getLocation(){
        return location;
    }
}