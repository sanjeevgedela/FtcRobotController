package org.firstinspires.ftc.teamcode.SourceCode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

public class AutonCommands {
    Equipment equip = new Equipment();

    public void slideMovement(double power, int encPos) {
        equip.rightSlide.setTargetPosition(encPos);
        equip.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftControl(power);
    }


    public void liftControl(double power) {
        equip.rightSlide.setPower(power);
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
        slideMovement(1, 175);
        rotateControl(1);
    }

    public void scorePositionMid() {
        slideMovement(1, 400);
        rotateControl(1);
    }
}
