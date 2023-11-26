package org.firstinspires.ftc.teamcode.SourceCode.EncoderAuton.Objects;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class EncoderMecanum {

    //Define motors
    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftBack = null;
    public DcMotorEx rightBack = null;


    double wheelCircumference = (3.77953 * Math.PI);
    double ticksToInch = (537.6 / wheelCircumference);


    public void encoderMovementY(int dist, double power) {

        leftFront.setTargetPosition((int) (leftFront.getCurrentPosition() + (dist * ticksToInch)));
        rightFront.setTargetPosition((int) (rightBack.getCurrentPosition() + (dist * ticksToInch)));
        leftBack.setTargetPosition((int) (leftFront.getCurrentPosition() + (dist * ticksToInch)));
        rightBack.setTargetPosition((int) (rightBack.getCurrentPosition() + (dist * ticksToInch)));

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
    }

    public void encoderMovementX(int dist, double power) {

        leftFront.setTargetPosition((int) (leftFront.getCurrentPosition() + (dist * ticksToInch)));
        rightFront.setTargetPosition(-(int) (rightBack.getCurrentPosition() + (dist * ticksToInch)));
        leftBack.setTargetPosition(-(int) (leftFront.getCurrentPosition() + (dist * ticksToInch)));
        rightBack.setTargetPosition((int) (rightBack.getCurrentPosition() + (dist * ticksToInch)));

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
    }


    public void mecanum(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightRear");
    }

    public void initialize() {
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        leftFront.setPower(0);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
