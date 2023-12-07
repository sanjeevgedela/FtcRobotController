package org.firstinspires.ftc.teamcode.SourceCode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.TeleOp.clawControl;
import org.firstinspires.ftc.teamcode.commands.TeleOp.driverControl;
import org.firstinspires.ftc.teamcode.commands.TeleOp.slideControl;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.advanced.TeleOpAlignWithPoint;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

@TeleOp(name="TeleOpTest3")
public class TeleOpTest3 extends LinearOpMode {

    public static double DRAWING_TARGET_RADIUS = 2;

    private driverControl driver = new driverControl();
    private clawControl claw = new clawControl();
    private slideControl slide = new slideControl();

    // Define 2 states, driver control or alignment control
    enum Mode {
        NORMAL,
        ALIGN
    }

    private Mode currentMode = Mode.NORMAL;

    // Declare a PIDF Controller to regulate heading
    // Use the same gains as SampleMecanumDrive's heading controller
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

    // Declare a target vector you'd like your bot to align with
    // Can be any x/y coordinate of your choosing
    private Vector2d targetPosition = new Vector2d(0, 0);

    //controls lift motors
    //controls lift motors

    private void automatedOuttake() {
        if (gamepad2.dpad_up) {
            claw.rotateClaw.setPosition(1);
            slide.rightSlide.setTargetPosition(660);
            slide.leftSlide.setTargetPosition(660);
            slide.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.liftControl(1);
            sleep(500);
            claw.leftClaw.setPosition(0);
            claw.rightClaw.setPosition(0);
            sleep(250);
            claw.leftClaw.setPosition(1);
            claw.rightClaw.setPosition(1);
            slide.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.liftControl(0.65);
            sleep(150);
        }
    }



    @Override
    public void runOpMode () {
        //Create Mecanum Drive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        driver.driverMap(hardwareMap);
        driver.initialize();

        claw.clawMap(hardwareMap);
        claw.initialize();

        slide.slideMap(hardwareMap);
        slide.initialize();

        //Create turn sequence
        TrajectorySequence turn = drive.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(90)))
                .turn(Math.toRadians(190))
                .waitSeconds(1)
                .build();

        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.getLocalizer().setPoseEstimate(PoseStorage.currentPose);

        // Set input bounds for the heading controller
        // Automatically handles overflow
        headingController.setInputBounds(-Math.PI, Math.PI);



        waitForStart();
        sleep(100);
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                driver.driverControl();
                slide.cascadingLift();
                slide.slideControl();
                claw.clawControl();
                claw.rotateControl();
                automatedOuttake();
                telemetry.update();


                if (gamepad1.y) {
                    drive.followTrajectorySequence(turn);
                }

                if (gamepad1.left_stick_button){
                    drive.update();
                    Pose2d myPose = drive.getPoseEstimate();
                    double heading = Math.toDegrees(myPose.getHeading());

                    double targetHeading = (Math.atan(gamepad1.left_stick_y/gamepad1.left_stick_x));

                    TrajectorySequence findHeading = drive.trajectorySequenceBuilder(new Pose2d(myPose.getX(), myPose.getY(), targetHeading))
                            .turn(Math.toRadians(190))
                            .waitSeconds(1)
                            .build();

                    telemetry.addData("Heading:", heading);

                    drive.followTrajectorySequence(findHeading);
                }


                // Read pose
                Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();

                // Declare a drive direction
                // Pose representing desired x, y, and angular velocity
                Pose2d driveDirection = new Pose2d();

                telemetry.addData("mode", currentMode);

                // Declare telemetry packet for dashboard field drawing
                TelemetryPacket packet = new TelemetryPacket();
                Canvas fieldOverlay = packet.fieldOverlay();

                switch (currentMode) {
                    case NORMAL:
                        // Switch into alignment mode if `a` is pressed
                        if (gamepad1.a) {
                            currentMode = Mode.ALIGN;
                        }
                        driver.driverControl();
                        break;
                    case ALIGN:
                        // Switch back into normal driver control mode if `b` is pressed
                        if (gamepad1.b) {
                            currentMode = Mode.NORMAL;
                        }

                        // Create a vector from the gamepad x/y inputs which is the field relative movement
                        // Then, rotate that vector by the inverse of that heading for field centric control
                        Vector2d fieldFrameInput = new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        );
                        Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());

                        // Difference between the target vector and the bot's position
                        Vector2d difference = targetPosition.minus(poseEstimate.vec());
                        // Obtain the target angle for feedback and derivative for feedforward
                        double theta = difference.angle();

                        // Not technically omega because its power. This is the derivative of atan2
                        double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

                        // Set the target heading for the heading controller to our desired angle
                        headingController.setTargetPosition(theta);

                        // Set desired angular velocity to the heading controller output + angular
                        // velocity feedforward
                        double headingInput = (headingController.update(poseEstimate.getHeading())
                                * DriveConstants.kV + thetaFF)
                                * DriveConstants.TRACK_WIDTH;

                        // Combine the field centric x/y velocity with our derived angular velocity
                        driveDirection = new Pose2d(
                                robotFrameInput,
                                headingInput
                        );

                        // Draw the target on the field
                        fieldOverlay.setStroke("#dd2c00");
                        fieldOverlay.strokeCircle(targetPosition.getX(), targetPosition.getY(), DRAWING_TARGET_RADIUS);

                        // Draw lines to target
                        fieldOverlay.setStroke("#b89eff");
                        fieldOverlay.strokeLine(targetPosition.getX(), targetPosition.getY(), poseEstimate.getX(), poseEstimate.getY());
                        fieldOverlay.setStroke("#ffce7a");
                        fieldOverlay.strokeLine(targetPosition.getX(), targetPosition.getY(), targetPosition.getX(), poseEstimate.getY());
                        fieldOverlay.strokeLine(targetPosition.getX(), poseEstimate.getY(), poseEstimate.getX(), poseEstimate.getY());
                        break;
                }

                // Draw bot on canvas
                fieldOverlay.setStroke("#3F51B5");
                DashboardUtil.drawRobot(fieldOverlay, poseEstimate);

                drive.setWeightedDrivePower(driveDirection);

                // Update the heading controller with our current heading
                headingController.update(poseEstimate.getHeading());

                // Update he localizer
                drive.getLocalizer().update();

                // Send telemetry packet off to dashboard
                FtcDashboard.getInstance().sendTelemetryPacket(packet);

                // Print pose to telemetry
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", poseEstimate.getHeading());
                telemetry.update();
            }
        }
    }
}
