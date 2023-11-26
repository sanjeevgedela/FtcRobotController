package org.firstinspires.ftc.teamcode.SourceCode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TeleOpTest1")
public class TeleOpTest1 extends LinearOpMode {

    //Define motors
    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftBack = null;
    public DcMotorEx  rightBack = null;
    public DcMotorEx spinTake = null;

    public Servo servo = null;
//    public DcMotorEx arm1 = null;
//    public DcMotorEx arm2 = null;

    double movement;
    double rotation;
    double strafe;

    public void driverControl() {
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

        leftFront.setPower(ratio * lf);
        leftBack.setPower(ratio * lb);
        rightFront.setPower(ratio * rf);
        rightBack.setPower(ratio * rb);
    }

    public void spinnerControl() {

        if (gamepad2.left_stick_y > 0.2) {
            spinTake.setPower(gamepad2.left_stick_y);
        } else if (gamepad2.left_stick_y < 0.2) {
            spinTake.setPower(gamepad2.left_stick_y);
        }

    }

    public void servoControl() {
        if (gamepad2.y) {
            servo.setPosition(0);
        } else if (gamepad2.x){
            servo.setPosition(1);
        }

    }

//    private void armControl (double power) {
//        arm1.setPower(power);
//        arm2.setPower(power);
//    }

//    private void resetLift() {
//            arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }

//    private void cascadingArm() {
//        if(Lift1.getCurrentPosition() = 0 && Lift2.getCurrentPosition() = 0.0) {
//        if(gamepad2.x) {
//            resetLift();
//        } else if (gamepad2.y) {
//            arm1.setTargetPosition(1000);
//            arm2.setTargetPosition(-1000);
//            arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            armControl(1);
//        } else if(gamepad2.dpad_up){
//            arm1.setTargetPosition(arm1.getCurrentPosition() + 50);
    //           arm2.setTargetPosition(arm2.getCurrentPosition() - 50);
//            arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            armControl(.5);
//        }
//        else if(gamepad2.dpad_down){
//            arm1.setTargetPosition(arm1.getCurrentPosition() - 50);
    //           arm2.setTargetPosition(arm2.getCurrentPosition() + 50);
//            arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            armControl(.5);
//        }
//    }

    @Override
    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightRear");
        spinTake = hardwareMap.get(DcMotorEx.class, "spin");
        servo = hardwareMap.get(Servo.class, "servo");

//        arm1 = hardwareMap.get(DcMotorEx.class, "arm1");
//        arm2 = hardwareMap.get(DcMotorEx.class, "arm2");


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinTake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        waitForStart();
        sleep(250);
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                driverControl();
                spinnerControl();
                servoControl();

//                cascadingArm();
            }
        }
    }
}
