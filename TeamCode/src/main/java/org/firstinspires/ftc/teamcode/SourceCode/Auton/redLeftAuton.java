package org.firstinspires.ftc.teamcode.SourceCode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

public class redLeftAuton extends LinearOpMode {

    OpenCvCamera webcam;
    DcMotorEx spinTake;
    closestPixel Please = new closestPixel();
    int horizRes = 1280;
    int closestPixelX = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        spinTake = hardwareMap.get(DcMotorEx.class, "spin");
        spinTake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(1280, 960, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        TrajectorySequence Left = drive.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(90)))
                .forward(30)
                .turn(Math.toRadians(90))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    spinTake.setPower(0.6);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    spinTake.setPower(0);
                })
                .waitSeconds((1))
                .turn(Math.toRadians(-90))
                .forward(22)
                .turn(Math.toRadians(-90))
                .waitSeconds(0.1)
                .forward(85)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    spinTake.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    spinTake.setPower(0);
                })
                .waitSeconds(3)
                .build();

        TrajectorySequence Right = drive.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(90)))
                .forward(30)
                .turn(Math.toRadians(-90))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    spinTake.setPower(0.6);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    spinTake.setPower(0);
                })
                .waitSeconds((1))
                .turn(Math.toRadians(90))
                .forward(22)
                .waitSeconds(1)
                .turn(Math.toRadians(-90))
                .waitSeconds(0.1)
                .forward(85)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    spinTake.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    spinTake.setPower(0);
                })
                .waitSeconds(3)
                .build();

        TrajectorySequence Forward = drive.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(90)))
                .forward(27)
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    spinTake.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    spinTake.setPower(0);
                })
                .waitSeconds((1))
                .forward(3)
                .forward(22)
                .turn(Math.toRadians(-90))
                .waitSeconds(0.1)
                .forward(85)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    spinTake.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    spinTake.setPower(0);
                })
                .waitSeconds(3)
                .build();

        TrajectorySequence NoDetection = drive.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(90)))
                .forward(27)
                .waitSeconds(1)
                .waitSeconds((1))
                .forward(3)
                .forward(22)
                .turn(Math.toRadians(-90))
                .waitSeconds(0.1)
                .forward(85)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    spinTake.setPower(0.6);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    spinTake.setPower(0);
                })
                .waitSeconds(3)
                .build();

        waitForStart();
        sleep(1000);

        closestPixelX = Please.closestPixel();

        if (closestPixelX < ((1/3) * horizRes)){
            drive.followTrajectorySequence(Left);
            sleep(30000000);

        } else if (closestPixelX < ((2/3) * horizRes)){
            drive.followTrajectorySequence(Forward);
            sleep(30000000);

        } else if (closestPixelX > ((2/3) * horizRes)){
            drive.followTrajectorySequence(Right);
            sleep(30000000);

        } else {
            drive.followTrajectorySequence(NoDetection);
            sleep(30000000);

        }
    }
}
class closestPixel extends LinearOpMode {

    public TfodProcessor tfod;
    public VisionPortal visionPortal;

    public void initTfod() {

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
            double x = (recognition.getRight() - recognition.getLeft());
            double y = (recognition.getTop() - recognition.getBottom());
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

    @Override
    public void runOpMode() throws InterruptedException {
        initTfod();
    }
}