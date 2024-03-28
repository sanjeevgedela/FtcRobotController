package org.firstinspires.ftc.teamcode.SourceCode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SourceCode.Subsystems.Pipelines.PixelPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class TeleopCommands {

    Equipment equip;

    public TeleopCommands(Equipment e) {equip = e;}


    public void clawControl(Gamepad gamepad2) {
        if (gamepad2.left_trigger > 0) {
            equip.leftClaw.setPosition(1);
        } else {
            equip.leftClaw.setPosition(0);
        }
        if (gamepad2.right_trigger > 0) {
            equip.rightClaw.setPosition(1);
        } else {
            equip.rightClaw.setPosition(0);
        }
    }

    public void rotateControl(Gamepad gamepad2) {
        if (gamepad2.dpad_down){
            equip.rotateClaw.setPosition(0);
        } else if (gamepad2.dpad_up){
            equip.rotateClaw.setPosition(1);
        }
    }

    //controls lift motors
    private void liftControl(double power) {
        equip.rightSlide.setPower(power);
    }

    private void resetLift(Gamepad gamepad1) {
        if (gamepad1.x) {
            equip.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            equip.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    //    //Lifts the lift
    private void cascadinglift(Gamepad gamepad2) {
        int rightSlideTarget = equip.rightSlide.getCurrentPosition() - 50;

        if(rightSlideTarget > 660){
            rightSlideTarget = 660;
        }

        if((equip.rightSlide.getTargetPosition() == equip.rightSlide.getCurrentPosition()) && (equip.rightSlide.getCurrentPosition() == 0)){
            equip.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if (gamepad2.y) {
            equip.rightSlide.setTargetPosition(220);
            equip.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftControl(1);
            equip.rotateClaw.setPosition(1);

        } else if (gamepad2.b) {
            equip.rightSlide.setTargetPosition(440);
            equip.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftControl(1);
            equip.rotateClaw.setPosition(1);

        } else if (gamepad2.a) {
            equip.rightSlide.setTargetPosition(720);
            equip.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftControl(1);
            equip.rotateClaw.setPosition(1);

        } else if (gamepad2.x) {
            equip.rightSlide.setTargetPosition(0);
            equip.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftControl(1);
            equip.rotateClaw.setPosition(1);

        } else if (gamepad2.left_stick_y > 0.2) {
            equip.rightSlide.setTargetPosition(rightSlideTarget);
            equip.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftControl(.5);

        } else if (gamepad2.left_stick_y < -0.2) {
            equip.rightSlide.setTargetPosition(equip.rightSlide.getCurrentPosition() + 50);
            equip.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftControl(.5);

        }
    }

    public void planeControl(Gamepad gamepad2){
        if(gamepad2.right_bumper){
            equip.plane.setPosition(0.3);
        }
    }
    public void hang(Gamepad gamepad2){
        if(gamepad2.dpad_right){
            equip.leftSlide.setTargetPosition(2000);
            equip.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            equip.leftSlide.setPower(1);
        } else if (gamepad2.dpad_left){
            equip.leftSlide.setTargetPosition(0);
            equip.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            equip.leftSlide.setPower(1);
        }
    }

    TeleopDrive drivebase;

    public void setDrivebase(Gamepad gamepad1, SampleMecanumDrive drive, Telemetry t, double x){
        drivebase.calc(gamepad1);
        drivebase.align(drive, t, gamepad1, x);
    }

    public void init(SampleMecanumDrive drive, HardwareMap hardwareMap, Equipment.Mode mode, OpenCvCamera camera){
        drivebase.driveInit(drive);
        equip.initialize(mode, hardwareMap, camera);
        drivebase = new TeleopDrive(equip);
    }

    public void activate(Gamepad gamepad1, Gamepad gamepad2, SampleMecanumDrive drive, Telemetry t, double x){
        //clawControl(gamepad2);
        clawControl(gamepad1);
        rotateControl(gamepad2);
        resetLift(gamepad1);
        cascadinglift(gamepad2);
        planeControl(gamepad2);
        hang(gamepad2);
        setDrivebase(gamepad1, drive, t, x);
    }
}
