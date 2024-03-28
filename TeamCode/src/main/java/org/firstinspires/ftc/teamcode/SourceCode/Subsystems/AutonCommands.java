package org.firstinspires.ftc.teamcode.SourceCode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PersonalPID;

public class AutonCommands {
    Equipment equip;

    public AutonCommands(Equipment e) {equip = e;}

    public void pidUPDATE(){
        equip.controller.setPIDF(equip.p, equip.i, equip.d, equip.f);
        int armPos = equip.rightSlide.getCurrentPosition();
        equip.pid = equip.controller.calculate(armPos, equip.target);
        liftControl(equip.pid);
    }

    public void slideMovement(double power, int encPos) {
        equip.rightSlide.setTargetPosition(encPos);
        equip.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        equip.leftSlide.setTargetPosition(encPos);
        equip.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftControl(power);
    }


    public void liftControl(double power) {
        equip.rightSlide.setPower(power);
        equip.leftSlide.setPower(power);
    }

    public void reset() {
        slideMovement(1,0);
        clawControl(0, 0);
        equip.rotateClaw.setPosition(1);
    }

    public void reset2(){
        equip.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void rotateControl(double rotate) {
        equip.rotateClaw.setPosition(rotate);
    }

    public void clawControl(double left, double right) {
        equip.leftClaw.setPosition(left);
        equip.rightClaw.setPosition(right);
    }

    public void scorePositionLow() {
        slideMovement(1, 600);
        rotateControl(1);
    }

    public void scorePositionMid() {
        slideMovement(1, 900);
        rotateControl(1);
    }

    public void scorePositionHigh() {
        slideMovement(1, 1400);
        rotateControl(1);
    }
}
