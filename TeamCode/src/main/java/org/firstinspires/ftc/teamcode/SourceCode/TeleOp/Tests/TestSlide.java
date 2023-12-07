package org.firstinspires.ftc.teamcode.SourceCode.TeleOp.Tests;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@TeleOp(name="Slide", group = "tests")
public class TestSlide extends LinearOpMode {

    public DcMotorEx leftSlide = null;
    public DcMotorEx rightSlide = null;


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
            rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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


    @Override
    public void runOpMode () {

        //Define all Slide motors
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");


        //Set Zero Power Behavior
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        sleep(100);
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                cascadinglift();
                resetLift();
            }
        }
    }
}
