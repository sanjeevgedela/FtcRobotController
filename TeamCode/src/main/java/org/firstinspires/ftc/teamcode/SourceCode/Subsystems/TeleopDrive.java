package org.firstinspires.ftc.teamcode.SourceCode.Subsystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SourceCode.Subsystems.Pipelines.PixelPipeline;
import org.firstinspires.ftc.teamcode.SourceCode.TeleOp.TeleOpTest2;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.trajectorysequence.TrajectorySequence;

public class TeleopDrive {
    //ClosestPixelX = pipeline.getCenterX();

    Equipment equip = new Equipment();

    double FrameCenterX = 180;

    double movement;
    double rotation;
    double strafe;

    enum DriveMode {
        NORMAL_CONTROL,
        ALIGN_TO_POINT,
        ALIGN_TO_PIXEL
    }

    private DriveMode currentMode = DriveMode.NORMAL_CONTROL;
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);
    private Vector2d targetPositionRed = new Vector2d(72, -38.2);
    private Vector2d targetPositionBlue = new Vector2d(72, 38.2);
    private Vector2d targetPosition = new Vector2d();

    double y;
    double x;
    int angle;

    public void calc(Gamepad gamepad1){
        x = Math.copySign(Math.pow(-gamepad1.left_stick_y, 1), -gamepad1.left_stick_y);
        y = Math.copySign(Math.pow(-gamepad1.left_stick_x, 1), -gamepad1.left_stick_x);
    }

    public void align(SampleMecanumDrive drive, Telemetry telemetry, Gamepad gamepad1, double ClosestPixelX){
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
            case NORMAL_CONTROL:
                // Switch into alignment mode if `a` is pressed
                if (gamepad1.a) {
                    currentMode = DriveMode.ALIGN_TO_POINT;
                }
                if (gamepad1.x) {
                    currentMode = DriveMode.ALIGN_TO_PIXEL;
                }

                // Standard teleop control
                // Convert gamepad input into desired pose velocity
                driverControl(drive, gamepad1);
                headingController.update(poseEstimate.getHeading());

                // Update he localizer
                drive.getLocalizer().update();
                break;
            case ALIGN_TO_POINT:
                // Switch back into normal driver control mode if `b` is pressed
                if (gamepad1.b) {
                    currentMode = DriveMode.NORMAL_CONTROL;
                }
                if (gamepad1.x) {
                    currentMode = DriveMode.ALIGN_TO_PIXEL;
                }

                // Create a vector from the gamepad x/y inputs which is the field relative movement
                // Then, rotate that vector by the inverse of that heading for field centric control
                Vector2d fieldFrameInput = new Vector2d(
                        x,
                        y
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
                drive.setWeightedDrivePower(driveDirection);
                headingController.update(poseEstimate.getHeading());

                // Update he localizer
                drive.getLocalizer().update();
                break;

            case ALIGN_TO_PIXEL:
                if (gamepad1.a) {
                    currentMode = DriveMode.ALIGN_TO_POINT;
                }
                if (gamepad1.b) {
                    currentMode = DriveMode.NORMAL_CONTROL;
                }

                TrajectorySequence now = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .turn(Math.toRadians(angle))
                        .forward(14)
                        .build();

                if(ClosestPixelX > FrameCenterX){
                    angle = (int) ((ClosestPixelX - FrameCenterX) / -15);
                    headingController.setTargetPosition(drive.getPoseEstimate().getHeading() - Math.toRadians(angle));
                } else if (ClosestPixelX < FrameCenterX){
                    angle = (int) ((FrameCenterX - ClosestPixelX) / 15);
                    headingController.setTargetPosition(drive.getPoseEstimate().getHeading() + Math.toRadians(angle));
                }
                if((ClosestPixelX < (FrameCenterX + 5)) && (ClosestPixelX > (FrameCenterX - 5))){
                    angle = 0;
                }

                if(gamepad1.y){
                    drive.followTrajectorySequence(now);
                }

                Pose2d driveDirection1 = new Pose2d(x, y, Math.toRadians(angle));
                drive.setWeightedDrivePower(driveDirection1);
                headingController.update(poseEstimate.getHeading());

                // Update the localizer
                drive.getLocalizer().update();
                break;
        }
    }
    public void driverControl(SampleMecanumDrive drive, Gamepad gamepad1) {

        movement = gamepad1.left_stick_y;
        rotation = gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;

        double magnitude = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2));
        double direction = Math.atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        boolean precision = gamepad1.right_bumper;

        //INFO Increasing speed to a maximum of 1
        double lf = magnitude * Math.sin(direction + Math.PI / 4) + rotation;
        double lb = magnitude * Math.cos(direction + Math.PI / 4) + rotation;
        double rf = magnitude * Math.cos(direction + Math.PI / 4) - rotation;
        double rb = magnitude * Math.sin(direction + Math.PI / 4) - rotation;

        double hypot = Math.hypot(movement, strafe);
        double ratio;
        if (movement == 0 && strafe == 0)
            ratio = 1;
        else if (precision)
            ratio = hypot / (Math.max(Math.max(Math.max(Math.abs(lf), Math.abs(lb)), Math.abs(rb)), Math.abs(rf))) / 2;
        else
            ratio = hypot / (Math.max(Math.max(Math.max(Math.abs(lf), Math.abs(lb)), Math.abs(rb)), Math.abs(rf)));

        equip.leftFront.setPower(ratio * lf);
        equip.leftBack.setPower(ratio * lb);
        equip.rightFront.setPower(ratio * rf);
        equip.rightBack.setPower(ratio * rb);

        if (gamepad1.y) {
            drive.turn(Math.toRadians(180));
        }
    }

    public void driveInit(SampleMecanumDrive drive, PixelPipeline pipeline){
        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.getLocalizer().setPoseEstimate(PoseStorage.currentPose);

        if(PoseStorage.currentPose.getY() > 0){
            targetPosition = targetPositionBlue;
        } else {
            targetPosition = targetPositionRed;
        }

        // Set input bounds for the heading controller
        // Automatically handles overflow
        headingController.setInputBounds(-Math.PI, Math.PI);

        drive.setPoseEstimate(PoseStorage.currentPose);

        equip.camera(pipeline);
    }
}
