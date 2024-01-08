package org.firstinspires.ftc.teamcode.SourceCode.Auton.OLD;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

//@Autonomous(name = "HELP", group = "FRFR")
public class redLeftAuton extends LinearOpMode {

    OpenCvCamera webcam;
    DcMotorEx spinTake;
    closestPixel Please = new closestPixel();
    int horizRes = 1280;
    int closestPixelX = 0;

    //Define Motors
    public DcMotorEx leftSlide = null;
    public DcMotorEx rightSlide = null;

    //Define servos
    public Servo rightClaw = null;
    public Servo leftClaw = null;
    public Servo rotateClaw = null;

    public void liftControl(double power){
        rightSlide.setPower(power);
        leftSlide.setPower(power);
    }
    @Override
    public void runOpMode() throws InterruptedException {

        //Define all Slide motors
        //leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        //rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        //Define All servos
        //rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        //leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        //rotateClaw = hardwareMap.get(Servo.class, "rotateClaw");

        //Set Zero Power Behavior
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set up encoders
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
                .lineToLinearHeading(new Pose2d(-38,-28,Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    rotateClaw.setPosition(0);
                    leftClaw.setPosition(1);
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    rotateClaw.setPosition(0);
                    leftClaw.setPosition(0);
                })
                .lineToLinearHeading(new Pose2d(-30,-10,Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(30,-10,Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    rotateClaw.setPosition(1);
                    rightSlide.setTargetPosition(1250);
                    leftSlide.setTargetPosition(-1250);
                    rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftControl(1);
                })
                .splineToLinearHeading(new Pose2d(54,-26,Math.toRadians(0)),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    rightClaw.setPosition(1);
                })
                .waitSeconds(1)
                .build();

        TrajectorySequence Part2 = drive.trajectorySequenceBuilder(new Pose2d(54,-26,Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    rotateClaw.setPosition(1);
                    rightSlide.setTargetPosition(0);
                    leftSlide.setTargetPosition(0);
                    rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftControl(1);
                })
                .lineToLinearHeading(new Pose2d(30,-14,Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    rightClaw.setPosition(1);
                    leftClaw.setPosition(1);
                })
                .lineToLinearHeading(new Pose2d(-30,-10,Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    rotateClaw.setPosition(1);
                    rightSlide.setTargetPosition(420);
                    leftSlide.setTargetPosition(-420);
                    rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftControl(1);
                })
                .lineToLinearHeading(new Pose2d(-60,-13,Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    rightClaw.setPosition(0);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    rotateClaw.setPosition(1);
                })
                .waitSeconds(0.5)
                .strafeRight(3.5)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    rotateClaw.setPosition(1);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    leftClaw.setPosition(0);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    rotateClaw.setPosition(1);
                    rightSlide.setTargetPosition(0);
                    leftSlide.setTargetPosition(0);
                    rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftControl(1);
                    rotateClaw.setPosition(0);
                })
                .lineToLinearHeading(new Pose2d(30,-12,Math.toRadians(0)))
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

        //if (closestPixelX < ((1/3) * horizRes)){
        //    drive.followTrajectorySequence(Left);
        //    sleep(30000000);

        //} else if (closestPixelX < ((2/3) * horizRes)){
        //    drive.followTrajectorySequence(Forward);
        //    sleep(30000000);

        //} else if (closestPixelX > ((2/3) * horizRes)){
        //    drive.followTrajectorySequence(Right);
        //    sleep(30000000);

        //} else {
        //    drive.followTrajectorySequence(NoDetection);
        //    sleep(30000000);

        //}
        drive.followTrajectorySequence(Left);
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