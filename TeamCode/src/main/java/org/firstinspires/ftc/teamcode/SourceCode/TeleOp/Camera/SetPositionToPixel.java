package org.firstinspires.ftc.teamcode.SourceCode.TeleOp.Camera;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

@Autonomous(name = "pixel")
public class SetPositionToPixel extends LinearOpMode{

    public OpenCvCamera webcam;
    double FrameCenterX = 320;
    int ClosestPixelX = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initTfod(hardwareMap);


        waitForStart();


        if (opModeIsActive()) {
            while (opModeIsActive()) {
                ClosestPixelX = closestPixel();
                telemetryTfod();
                telemetry.update();
                sleep(20);
            }
        }
    }
    public TfodProcessor tfod;
    public VisionPortal visionPortal;

    public void initTfod(HardwareMap hardwareMap) {

        // Create the TensorFlow processor the easy way.
        tfod = TfodProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.30f);

        // Disable or re-enable the TFOD processor at any time.
        visionPortal.setProcessorEnabled(tfod, true);
    }


    public int closestPixel() {
            int X = 0;
            if(tfod.getRecognitions().size()!= 0){
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        List<Integer> Areas = null;

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;
            double A = x * y;
            if(A!=0) {
                Areas.add((int) A);
            }
        }
        if(Areas.size() != 0) {
            for (int i = 0; i < Areas.size(); i++) {
                int min = 0;
                Recognition ClosestPixel = null;

                if (Areas.get(i) < min) {
                    min = Areas.get(i);
                    ClosestPixel = currentRecognitions.get(i);
                    X = (int) ClosestPixel.getLeft();
                }
                return X;
            }
        }
        } else {
            X = 0;
        }
                return X;
        }
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop
        telemetry.addData("ClosestPixelX: ", ClosestPixelX);
    }   // end method telemetryTfod()

}   // end class

