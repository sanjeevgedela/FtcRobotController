package org.firstinspires.ftc.teamcode.SourceCode.Camera.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

public class SetPositionToPixel extends LinearOpMode{

    public OpenCvCamera webcam;
    closestPixel Please;
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    public int ClosestPixelX = Please.closestPixel();
    double FrameCenterX = 640;


    private void angleCorrection() {
        //Determines which way and by how much the robot should rotate
        if (FrameCenterX - ClosestPixelX + 100 > 350) {
            drive.turn(Math.toRadians(-5));
        } else if (FrameCenterX - ClosestPixelX + 100 > 100) {
            drive.turn(Math.toRadians(-2));
        } else if (FrameCenterX - ClosestPixelX + 100 > 1) {
            drive.turn(Math.toRadians(-1));
        } else if (FrameCenterX - ClosestPixelX + 100 == 0) {
            drive.turn(Math.toRadians(0));
        } else if (FrameCenterX - ClosestPixelX + 100 < -1) {
            drive.turn(Math.toRadians(1));
        } else if (FrameCenterX - ClosestPixelX + 100 < -100) {
            drive.turn(Math.toRadians(2));
        } else if (FrameCenterX - ClosestPixelX + 100 < -350) {
            drive.turn(Math.toRadians(5));
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 960, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                angleCorrection();
            }
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

    @Override
    public void runOpMode() throws InterruptedException {
        initTfod();
    }
}