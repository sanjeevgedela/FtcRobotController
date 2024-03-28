package org.firstinspires.ftc.teamcode.SourceCode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class apriltag {
    WebcamName webcam1;
    GainControl gainControl;

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    boolean targetFound = false;

    double theta = 0;

    public apriltag(WebcamName a) {
        webcam1 = a;
    }

    public apriltag(){

    }

    public void initAprilTag(VisionPortal visionPortal, HardwareMap hardwareMap) {

        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();


        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 2"), aprilTag);

    }

    public enum DETECT {
        LEFT, MIDDLE, RIGHT
    }

    public enum COLOR {
        BLUE, RED
    }

    public void setType(DETECT type, COLOR color) {
        if (color == COLOR.BLUE) {
            if (type == DETECT.LEFT) {
                DESIRED_TAG_ID = 1;
            }
            if (type == DETECT.MIDDLE) {
                DESIRED_TAG_ID = 2;
            }
            if (type == DETECT.RIGHT) {
                DESIRED_TAG_ID = 3;
            }
        } else if (color == COLOR.RED) {
            if (type == DETECT.LEFT) {
                DESIRED_TAG_ID = 4;
            }
            if (type == DETECT.MIDDLE) {
                DESIRED_TAG_ID = 5;
            }
            if (type == DETECT.RIGHT) {
                DESIRED_TAG_ID = 6;
            }
        }
    }

    public void findTag(Telemetry telemetry) {
        targetFound = false;
        desiredTag = null;
        if(aprilTag.getDetections() != null){
        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }
        }
    }

    public double calculate() {

        double distance = 0;

        if (targetFound) {
            if(desiredTag.ftcPose.x < 0){
                distance = -((desiredTag.ftcPose.x * 2.25) + 2);
            } else if (desiredTag.ftcPose.x > 0) {
                distance = -((desiredTag.ftcPose.x * 2.25) - 2);
            }
            if (distance < 0.4 || distance > -0.4) {
              //  distance = 0;
            }
        }

        return distance;

    }

    public double relocalize(){

        if (targetFound) {
            theta = desiredTag.ftcPose.yaw;
        }

        return theta;
    }
}

