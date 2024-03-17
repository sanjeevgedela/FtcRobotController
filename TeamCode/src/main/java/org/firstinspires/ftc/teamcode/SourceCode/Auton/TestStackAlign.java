package org.firstinspires.ftc.teamcode.SourceCode.Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SourceCode.Subsystems.HuskyStackDetection;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.PersonalPID;
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
import java.util.Collections;
import java.util.List;

@Autonomous(name = "TestStackAlign")
public class TestStackAlign extends LinearOpMode {

    //Define motors
    public DcMotorEx leftSlide;
    public DcMotorEx rightSlide;

    public OpenCvCamera webcam;
    public OpenCvCamera webcam2;

    public PersonalPID controller;

    public HuskyStackDetection detect;
    int dist = 0;
    private HuskyLens huskyLens;

    public static double p = 0.007, i = 0, d = 0.0001, f = 0.001;
    int target;

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
        slideMovement(1,0);
        clawControl(0, 0);
        rotateClaw.setPosition(1);
    }

    public void reset2(){
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void rotateControl(double rotate) {
        rotateClaw.setPosition(rotate);
    }

    public void clawControl(double left, double right) {
        leftClaw.setPosition(left);
        rightClaw.setPosition(right);
    }

    public void wristDown(){
        rotateControl(0);
    }

    public void scorePositionLow() {
        rightSlide.setTargetPosition(800);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftControl(1);
        leftSlide.setTargetPosition(800);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftControl(1);
        rotateControl(1);
    }

    public void readyPick() {
        rightSlide.setTargetPosition(400);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftControl(1);
        leftSlide.setTargetPosition(400);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftControl(1);
        rotateControl(1);
    }

    public void slamDown() {
        rightSlide.setTargetPosition(210);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftControl(1);
        leftSlide.setTargetPosition(210);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftControl(1);
    }

    public void scorePositionMid() {
        rightSlide.setTargetPosition(1440);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftControl(1);
        leftSlide.setTargetPosition(1440);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftControl(1);
        rotateControl(1);
    }

    public void alignToStack(int StackCenterX) {

        // TEST 3
        PixelPipeline PixelPipeline = new PixelPipeline(telemetry);
        webcam2.setPipeline(PixelPipeline);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        // 640 = width of camera
        double ALIGNMENT_THRESHOLD = 4;
        int stackCenterX = -1;
        double lateralDistance = 640 / 4 - stackCenterX;
        double lateralVelocity = -lateralDistance * 3.5 / 640;
        if (Math.abs(lateralDistance) < ALIGNMENT_THRESHOLD) {
            drive.setMotorPowers(-1, 0, -1, 0);
        } else {
            drive.setMotorPowers(-lateralVelocity, lateralVelocity, -lateralVelocity, lateralVelocity);
        }

//            double lateralVelocity = -lateralDistance * 1.5 / 640;
//            drive.setMotorPowers(lateralVelocity, lateralVelocity, -lateralVelocity, -lateralVelocity);

        // TEST 2
//        PixelPipeline PixelPipeline = new PixelPipeline(telemetry);
//        webcam2.setPipeline(PixelPipeline);
//
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//
//        double ANGLE_TO_DISTANCE_FACTOR = -2;
//        double STRAFE_POWER = -1;
//
//        double correction = PixelPipeline.getPotentialAngle() * ANGLE_TO_DISTANCE_FACTOR;
//        if (correction > 1) {
//            drive.setDrivePower(new Pose2d(0, STRAFE_POWER, 0));
//        } else if (correction < 1) {
//            drive.setDrivePower(new Pose2d(0, STRAFE_POWER, 0));
//        } else {
//            drive.setDrivePower(new Pose2d(0, STRAFE_POWER, 0));
//        }

        // TEST 1
//        double ANGLE_THRESHOLD = 1.0;
//
//        while (Math.abs(PixelPipeline.getPotentialAngle()) > ANGLE_THRESHOLD) {
//            correction = PixelPipeline.getPotentialAngle() * ANGLE_TO_DISTANCE_FACTOR;
//            drive.setPoseEstimate(drive.getPoseEstimate().plus(new Pose2d(0,correction, 180)));
//            drive.update();
//        }
    }

    public void alignmentStart() {

        PixelPipeline PixelPipeline = new PixelPipeline(telemetry);
        webcam2.setPipeline(PixelPipeline);

        int stackCenterX = PixelPipeline.getStackCenterX();

        if (stackCenterX != -1) {
            alignToStack(stackCenterX);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PersonalPID(p, i, d, f);
        detect = new HuskyStackDetection(telemetry, huskyLens);

        PixelPipeline PixelPipeline = new PixelPipeline(telemetry);
        RedPipe0 pipeline = new RedPipe0(telemetry);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int cameraMonitorViewId2 = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId2", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId2);

        webcam.setPipeline(pipeline);
        webcam2.setPipeline(PixelPipeline);
        detect.init(hardwareMap);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        webcam2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam2.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
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
        leftClaw.scaleRange(0.5, 1);
        rightClaw.scaleRange(0, 0.5);
        rotateClaw.scaleRange(0.65, 1);
        leftClaw.setDirection(Servo.Direction.REVERSE);
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
        DriveConstants.MAX_VEL = 73;
        DriveConstants.MAX_ACCEL = 73;


        Pose2d startPose = new Pose2d(9.4, -62.4, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d(9.4, -62.4, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    rotateControl(0);
                    clawControl(0, 0);
                })
                .lineToLinearHeading(new Pose2d(31.2, -30.6, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(0, 1);
                })
                .waitSeconds(1)
                .back(3)
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    reset();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    scorePositionLow();
                })
                .lineToLinearHeading(new Pose2d(49.4, -38.2, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(1, 0);
                })
                .waitSeconds(0.5)
                .back(5)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    reset();
                })
                .strafeRight(15)
                .forward(5)
                .build();

        TrajectorySequence middle = drive.trajectorySequenceBuilder(new Pose2d(9.4, -62.4, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    slideMovement(1, -40);
                    rotateControl(0);
                    clawControl(0, 0);
                })
                .lineToLinearHeading(new Pose2d(12.2, -33.6, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(0, 1);
                })
                .waitSeconds(1)
                .back(5)
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    reset();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    scorePositionLow();
                })
                .lineToLinearHeading(new Pose2d(46.4, -31.2, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(1, 0);
                })
                .waitSeconds(0.5)
                .back(5)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    reset();
                })
                .strafeRight(23)
                .waitSeconds(1)
                .forward(5)
                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(new Pose2d(14.03, -62.82, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    rotateControl(0);
                    clawControl(0, 0);
                })
                .lineToLinearHeading(new Pose2d(4, -29.6, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(0, 1);
                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    scorePositionLow();
                })
                .lineToLinearHeading(new Pose2d(43.9, -25.2, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    clawControl(1, 0);
                })
                .waitSeconds(0.5)
                .back(7)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    reset();
                    rotateControl(1);
                    clawControl(0,0);
                })

                //Cycle Period

//                .splineToLinearHeading(new Pose2d(-33.57, -8.08, Math.toRadians(180)), Math.toRadians(0))

                .lineToLinearHeading(new Pose2d(28.47, -9, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(-33.57,-8.08))

                .waitSeconds(0.5)

                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    rotateControl(0);
                    clawControl(1,1);
//                    readyPick();
                })

                .splineToConstantHeading(new Vector2d(-53.60,-7), Math.toRadians(180))

//                .lineToConstantHeading(new Vector2d(-53.60,-8))

                .waitSeconds(0.5)

                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
//                    alignToStack();
                    alignmentStart();
                    int stackCenterX = PixelPipeline.getStackCenterX();
                    if (stackCenterX != -2) {
                        alignToStack(stackCenterX);
                    }
//                    alignToStack(-1);
                })

                .waitSeconds(1)

                .forward(13)

//                .lineToConstantHeading(new Vector2d(-64,-4.2))

//                .strafeRight(0.5)
//                .forward(1.5)

//                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
//                    wristDown();
//                    slamDown();
//                })

                .waitSeconds(1)

                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    clawControl(1,0);
                })

                .waitSeconds(1)

                .back(10)

                .waitSeconds(0.5)

                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    reset();
                })

                .lineToLinearHeading(new Pose2d(4.10, -9.47, Math.toRadians(0)))
                .splineTo(new Vector2d(38, -46.87), Math.toRadians(0))

                .waitSeconds(1)

                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    rotateControl(1);
                    scorePositionLow();
                    clawControl(0,0);
                })

                .waitSeconds(1)

                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    clawControl(0,1);
                })

                .waitSeconds(0.5)

                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    scorePositionMid();
                })

                .waitSeconds(1)

                .back(25)

                .waitSeconds(1)

                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    reset();
                })

                .splineToLinearHeading(new Pose2d(50, -64, Math.toRadians(180)), Math.toRadians(0))


                .build();

        TrajectorySequence left2 = drive.trajectorySequenceBuilder(new Pose2d(14.03, -62.8, Math.toRadians(90)))
                // x= 14.03 y= -62.8
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    rotateControl(0);
                    clawControl(0, 0);
                })
                .lineToLinearHeading(new Pose2d(4, -29.6, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    clawControl(0, 1);
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    scorePositionLow();
                })
                .lineToLinearHeading(new Pose2d(43.9, -25.2, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    clawControl(1, 0);
                })
                .waitSeconds(0.1)
                .back(7)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    reset();
                    rotateControl(1);
                    clawControl(0,0);
                })
//
//                Cycle Period
//
//                .splineToLinearHeading(new Pose2d(-33.57, -8.08, Math.toRadians(180)), Math.toRadians(0))
//
                .lineToLinearHeading(new Pose2d(28.47, -9, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-33.57,-8.08), Math.toRadians(180))

                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    rotateControl(0);
                    clawControl(1,1);
                    slideMovement(1,475);
//                    readyPick();
                })

//
                .splineToConstantHeading(new Vector2d(-53.60,-5), Math.toRadians(180))
//                    slideMovement(1,315);

//
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
//                    alignToStack();
//                    alignmentStart();
//                    int stackCenterX = PixelPipeline.getStackCenterX();
//                    if (stackCenterX != -2) {
//                        alignToStack(stackCenterX);
//                    }

//                    DriveConstants.MAX_ACCEL = 30;
//                    DriveConstants.MAX_VEL = 30;

//                    alignmentStart();
//                    int stackCenterX = PixelPipeline.getStackCenterX();
//                    if (stackCenterX != -2) {
//                        alignToStack(stackCenterX);
//                    }
                    dist = detect.method();
                    telemetry.addData("dist", dist);
                    slideMovement(1,450);
                    DriveConstants.MAX_ACCEL = 15;
                    DriveConstants.MAX_VEL = 15;

//                    alignToStack(-1);
                })

                .waitSeconds(.5)
                .strafeRight(dist + .0001)
                .forward(10)

                .waitSeconds(.1)

//
//                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
//                    wristDown();
//                    slamDown();
//                })
//

                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    clawControl(0,0);
                    telemetry.update();
                    DriveConstants.MAX_ACCEL = 73;
                    DriveConstants.MAX_VEL = 73;
                })

                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    rotateControl(1);
                })
                .back(10)

                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    reset();
                })

                .splineToConstantHeading(new Vector2d(4.10, -9.47), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    rotateControl(1);
                    slideMovement(1,1500);
                    clawControl(0,0);
                })
                .lineToLinearHeading(new Pose2d(42, -46.87, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    clawControl(0,1);
                })

                .waitSeconds(0.3)

                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    scorePositionMid();
                })

                .waitSeconds(.3)
                .back(3)

//                .back(25)
//
//                .waitSeconds(1)
//
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    reset();
                })
//
//                .splineToLinearHeading(new Pose2d(50, -64, Math.toRadians(180)), Math.toRadians(0))
                .waitSeconds(.5)

//                .splineToLinearHeading(new Pose2d(-33.57, -8.08, Math.toRadians(180)), Math.toRadians(0))

                .lineToLinearHeading(new Pose2d(28.47, -9, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-33.57,-5), Math.toRadians(180))

                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    rotateControl(0);
                    clawControl(1,1);
                    slideMovement(1,400);

//                    readyPick();
                })

                .splineToConstantHeading(new Vector2d(-53.60,0), Math.toRadians(180))

//                .lineToConstantHeading(new Vector2d(-53.60,-8))

                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
//                    alignToStack();
//                    alignmentStart();
//                    int stackCenterX = PixelPipeline.getStackCenterX();
//                    if (stackCenterX != -2) {
//                        alignToStack(stackCenterX);
//                    }
                    dist = detect.method();
                    telemetry.addData("dist", dist);
                    slideMovement(1,400);
                    DriveConstants.MAX_ACCEL = 15;
                    DriveConstants.MAX_VEL = 15;

//                    alignToStack(-1);
                })

                .waitSeconds(.6)
                .strafeRight(dist + .0001)
                .forward(10)
//                .lineToLinearHeading(new Pose2d(-65.60,-4 + dist, Math.toRadians(180)))

//                .lineToConstantHeading(new Vector2d(-64,-4.2))

//                .strafeRight(0.5)
//                .forward(1.5)

//                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
//                    wristDown();
//                    slamDown();
//                })


                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    clawControl(0,0);
                    telemetry.update();
                    DriveConstants.MAX_ACCEL = 73;
                    DriveConstants.MAX_VEL = 73;
                })

                .waitSeconds(.4)
                .UNSTABLE_addTemporalMarkerOffset(.01, () -> {
                    rotateControl(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    reset();
                })

                .back(10)


                .splineToConstantHeading(new Vector2d(4.10, -3), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    rotateControl(1);
                    scorePositionLow();
                    clawControl(0,0);
                })
                .lineToLinearHeading(new Pose2d(41 , -41.87, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    clawControl(0,1);
                })

                .waitSeconds(0.3)

                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    scorePositionMid();
                })

                .waitSeconds(.5)
                .back(3)

//                .back(25)
//
//                .waitSeconds(1)
//
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    reset();
                })
//
//                .splineToLinearHeading(new Pose2d(50, -64, Math.toRadians(180)), Math.toRadians(0))
                .waitSeconds(2)
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(new Pose2d(-50, -35.2, Math.toRadians(0)))
                .strafeRight(parkStrafe)
                .forward(10)
                .build();

        while (opModeInInit()){

            FtcDashboard.getInstance().startCameraStream(webcam, 120);
            pipeline.telemetry.update();

            FtcDashboard.getInstance().startCameraStream(webcam2, 120);
            PixelPipeline.telemetry.update();
        }

        waitForStart();
        sleep(100);
        RedPipe0.Location0 detectedColor = pipeline.getLocation();

        while (opModeIsActive()) {
            FtcDashboard.getInstance().startCameraStream(webcam, 120);
            FtcDashboard.getInstance().startCameraStream(webcam2, 120);
            pipeline.telemetry.update();
            PixelPipeline.telemetry.update();
            controller.setPIDF(p, i, d, f);
            int armPos = rightSlide.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            rightSlide.setPower(pid);
            leftSlide.setPower(pid);

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
                        drive.followTrajectorySequence(left2);
                        sleep(30000000);
                        break;
                }
            }
            PoseStorage.currentPose = drive.getPoseEstimate();
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
        static final Rect BMiddle = new Rect(
                new Point(145, 160),
                new Point(295, 60));
        static final Rect BRight = new Rect(
                new Point(430, 200),
                new Point(560, 90));

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



    public class PixelPipeline extends OpenCvPipeline {

        Telemetry telemetry;
        private Scalar lowerBound = new Scalar(0, 0, 200);
        private Scalar upperBound = new Scalar(180, 30, 255);

        // TEST 3
        int stackCenterX = -1; // X-coordinate of center of stack


        // TEST 1 && 2
//        int midx = 320;
//        int potentialAngle = 0;

        public PixelPipeline(Telemetry t) {
            telemetry = t;
        }


        @Override
        public Mat processFrame(Mat input) {

            // TEST 3
            Mat hsvImage = new Mat();
            Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_RGB2HSV);

            Mat mask = new Mat();
            Core.inRange(hsvImage, lowerBound, upperBound, mask);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            double maxArea = 0;
            Rect maxRect = new Rect();
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    maxRect = Imgproc.boundingRect(contour);
                }
            }

            stackCenterX = maxRect.x + maxRect.width / 2;

            // Create box
            Imgproc.rectangle(input, new Point(200, 40), new Point(430, 240), new Scalar(255, 0, 0), 2);

            Imgproc.line(input, new Point(stackCenterX, 0), new Point(stackCenterX, input.rows()), new Scalar(0, 255, 0), 2);
            Imgproc.line(input, new Point(stackCenterX - 10, input.rows() / 2), new Point(stackCenterX + 10, input.rows() / 2), new Scalar(0, 255, 0), 2);

            hsvImage.release();
            mask.release();

            return input;


            //TEST 2
//            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
//
//            Core.inRange(input, lowerBound, upperBound, input);
//
//            List<MatOfPoint> contours = new ArrayList<>();
//            Mat hierarchy = new Mat();
//            Imgproc.findContours(input, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
//            List<Integer> areas = new ArrayList<>();
//            List<Integer> MidX = new ArrayList<>();
//
//            for (MatOfPoint contour : contours) {
//                MatOfPoint2f approxCurve = new MatOfPoint2f();
//                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
//
//                double epsilon = 0.02 * Imgproc.arcLength(contour2f, true);
//                Imgproc.approxPolyDP(contour2f, approxCurve, epsilon, true);
//
//                Rect rect = Imgproc.boundingRect(new MatOfPoint(approxCurve.toArray()));
//
//                areas.add(rect.width * rect.height);
//                MidX.add(rect.x + rect.width / 2);
//            }
//
//            if (!areas.isEmpty()) {
//                int BOX_WIDTH = 20;
//                int maxIndex = areas.indexOf(Collections.max(areas));
//                Rect maxRect = Imgproc.boundingRect(new MatOfPoint2f(contours.get(maxIndex).toArray()));
//                Imgproc.rectangle(input, maxRect.tl(), maxRect.br(), new Scalar(255, 0, 0), 2);
//                midx = maxRect.x + maxRect.width - BOX_WIDTH;
//            }
//
//            Imgproc.line(input, new Point(midx, 235), new Point(midx, 245), new Scalar(0, 0, 255), 5);
//
//            potentialAngle = ((midx - 360) / 15);
//            telemetry.addData("angle", potentialAngle);
//
//            return input;
        }

        // TEST 1

//        public Mat processFrame(Mat input) {
//            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
//
//            Core.inRange(input, lowerBound, upperBound, input);
//
//            List<MatOfPoint> contours = new ArrayList<>();
//            Mat hierarchy = new Mat();
//            Imgproc.findContours(input, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
//            List<Integer> areas = new ArrayList<>();
//            List<Integer> MidX = new ArrayList<>();
//
//            for (MatOfPoint contour : contours) {
//                MatOfPoint2f approxCurve = new MatOfPoint2f();
//                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
//
//                double epsilon = 0.02 * Imgproc.arcLength(contour2f, true);
//                Imgproc.approxPolyDP(contour2f, approxCurve, epsilon, true);
//
//                Rect rect = Imgproc.boundingRect(new MatOfPoint(approxCurve.toArray()));
//
//                areas.add(rect.width * rect.height);
//                MidX.add(rect.x + rect.width / 2);
//            }
//
//            Imgproc.line(input, new Point(midx, 235), new Point(midx, 245), new Scalar(0, 0, 255), 5);
//
//            if (!areas.isEmpty()) {
//                int maxIndex = areas.indexOf(Collections.max(areas));
//                Rect maxRect = Imgproc.boundingRect(new MatOfPoint2f(contours.get(maxIndex).toArray()));
//                Imgproc.rectangle(input, maxRect.tl(), maxRect.br(), new Scalar(255, 0, 0), 2);
//                midx = maxRect.x + maxRect.width / 2;
//            }
//
//            potentialAngle = ((midx - 360) / 15);
//            telemetry.addData("angle", potentialAngle);
//
//            return input;
//        }

        // TEST 1&2

//        public Integer getPotentialAngle() {
//            return potentialAngle;
//        }
//
//        public void resetMidx() {
//            midx = 320;
//        }

        // TEST 3
        public int getStackCenterX() {
            return stackCenterX;
        }

    }

}