package org.firstinspires.ftc.teamcode.commands.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class slideControl {
    //Define motors
    public DcMotorEx leftSlide = null;
    public DcMotorEx rightSlide = null;
    //    //Lifts the lift

    public void slideControl() {
        if (gamepad1.x) {
            rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void liftControl(double power) {
        rightSlide.setPower(power);
        leftSlide.setPower(power);
    }

    public void cascadingLift() {
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

    public void slideMap(HardwareMap hardwareMap){
        //Define all Slide motors
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
    }

    public void initialize(){
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Reverse motors
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        //Set up encoders
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
