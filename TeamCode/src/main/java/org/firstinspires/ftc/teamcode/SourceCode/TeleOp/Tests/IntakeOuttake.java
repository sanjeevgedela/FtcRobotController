package org.firstinspires.ftc.teamcode.SourceCode.TeleOp.Tests;

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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@TeleOp(name="IntakeOuttake", group = "tests")
public class IntakeOuttake extends LinearOpMode {

    public DcMotorEx leftSlide = null;
    public DcMotorEx rightSlide = null;

    //Define servos
    public Servo rightClaw = null;
    public Servo leftClaw = null;
    public Servo rotateClaw = null;

    public void clawControl() {

        if (gamepad2.left_trigger > 0) {
            leftClaw.setPosition(1);
        } else {
            leftClaw.setPosition(0);
        }
        if (gamepad2.right_trigger > 0) {
            rightClaw.setPosition(1);
        } else {
            rightClaw.setPosition(0);
        }
    }

    public void rotateControl() {
        if (gamepad2.right_stick_y > 0.2) {
            rotateClaw.setPosition(rotateClaw.getPosition() + 0.1);
        } else if (gamepad2.right_stick_y < -0.2) {
            rotateClaw.setPosition(rotateClaw.getPosition() - 0.1);
        } else if (gamepad2.a) {
            rotateClaw.setPosition(1);
        } else if (gamepad2.b) {
            rotateClaw.setPosition(1);
        } else if (gamepad2.y) {
            rotateClaw.setPosition(1);
        } else if (gamepad2.x) {
            rotateClaw.setPosition(0);
        }
    }

    //controls lift motors
    private void liftControl(double power) {
        rightSlide.setPower(power);
        leftSlide.setPower(power);
    }

    private void resetLift() {
        if (gamepad1.x) {
            rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    //    //Lifts the lift
    private void cascadinglift() {
        int rightSlideTarget = rightSlide.getCurrentPosition() - 50;
        int leftSlideTarget = leftSlide.getCurrentPosition() - 50;

        if(rightSlideTarget > 660){
            rightSlideTarget = 660;
        }
        if (leftSlideTarget > 660){
            leftSlideTarget = 660;
        }

        if (gamepad2.a) {
            rightSlide.setTargetPosition(220);
            leftSlide.setTargetPosition(220);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftControl(1);
        } else if (gamepad2.b) {
            rightSlide.setTargetPosition(440);
            leftSlide.setTargetPosition(440);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftControl(1);
        } else if (gamepad2.y) {
            rightSlide.setTargetPosition(720);
            leftSlide.setTargetPosition(720);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftControl(1);
        } else if (gamepad2.x) {
            rightSlide.setTargetPosition(0);
            leftSlide.setTargetPosition(0);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftControl(1);
        } else if (gamepad2.left_stick_y > 0.2) {
            rightSlide.setTargetPosition(rightSlideTarget);
            leftSlide.setTargetPosition(leftSlideTarget);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftControl(.5);
        } else if (gamepad2.left_stick_y < -0.2) {
            rightSlide.setTargetPosition(rightSlide.getCurrentPosition() + 50);
            leftSlide.setTargetPosition(leftSlide.getCurrentPosition() + 50);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftControl(.5);
        }
    }

    private void automatedOuttake() {
        if (gamepad2.dpad_up) {
            rotateClaw.setPosition(1);
            rightSlide.setTargetPosition(660);
            leftSlide.setTargetPosition(660);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftControl(1);
            sleep(500);
            leftClaw.setPosition(0);
            rightClaw.setPosition(0);
            sleep(250);
            leftClaw.setPosition(1);
            rightClaw.setPosition(1);
            rightSlide.setTargetPosition(0);
            leftSlide.setTargetPosition(0);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftControl(0.65);
            sleep(150);
        }
    }



    @Override
    public void runOpMode () {

        //Define All servos
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rotateClaw = hardwareMap.get(Servo.class, "rotateClaw");

        //Reverse Servos

        //Set Ranges
        rightClaw.scaleRange(0.1,0.4);
        leftClaw.scaleRange(0,0.4);
        rotateClaw.scaleRange(0.65,1);

        waitForStart();
        sleep(100);
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                clawControl();
                rotateControl();
                cascadinglift();
                resetLift();

                automatedOuttake();
            }

        }
    }
}
