package org.firstinspires.ftc.teamcode.commands.Auto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.TeleOp.clawControl;
import org.firstinspires.ftc.teamcode.commands.TeleOp.slideControl;

public class IntakeOuttake {

    clawControl claw = new clawControl();
    slideControl slide = new slideControl();

    public void initialize(){
        claw.initialize();
        slide.initialize();
    }

    public void map(HardwareMap hardwareMap){
        claw.clawMap(hardwareMap);
        slide.slideMap(hardwareMap);
    }

    public void slideMovement(double power, int encPos){
        slide.rightSlide.setTargetPosition(encPos);
        slide.leftSlide.setTargetPosition(encPos);
        slide.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.liftControl(power);
    }

    public void reset(){
        slide.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        claw.rightClaw.setPosition(0);
        claw.leftClaw.setPosition(0);
        claw.rotateClaw.setPosition(1);
    }

    public void rotateControl(double rotate){
        claw.rotateClaw.setPosition(rotate);
    }

    public void clawControl(double left, double right){
        claw.leftClaw.setPosition(left);
        claw.rightClaw.setPosition(right);
    }

    public void scorePositionLow(){
        slideMovement(1,250);
        rotateControl(1);
    }

    public void scorePositionMid(){
        slideMovement(1,400);
        rotateControl(1);
    }
}
