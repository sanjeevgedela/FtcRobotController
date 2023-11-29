package org.firstinspires.ftc.teamcode.SourceCode.TeleOp;

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

@TeleOp(name="TeleOpTest2")
public class TeleOpTest2 extends LinearOpMode {

    //Define motors
    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftBack = null;
    public DcMotorEx rightBack = null;
    public DcMotorEx leftSlide = null;
    public DcMotorEx rightSlide = null;

    //Define servos
    public Servo rightClaw = null;
    public Servo leftClaw = null;
    public Servo rotateClaw = null;

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

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

        if (gamepad1.left_trigger > 0) {
            leftFront.setPower(ratio * lf * 0.5);
            leftBack.setPower(ratio * lb * 0.5);
            rightFront.setPower(ratio * rf * 0.5);
            rightBack.setPower(ratio * rb * 0.5);
        }

        if (gamepad1.left_stick_button){
        }
    }

    public void clawControl() {

        if (gamepad2.left_trigger > 0) {
            leftClaw.setPosition(0);
        } else if (gamepad2.right_trigger > 0) {
            rightClaw.setPosition(0);
        } else {
            leftClaw.setPosition(1);
            rightClaw.setPosition(1);
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
//        if(Lift1.getCurrentPosition() = 0 && Lift2.getCurrentPosition() = 0.0) {
        if (gamepad2.a) {
            rightSlide.setTargetPosition(1250);
            leftSlide.setTargetPosition(-1250);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftControl(1);
        } else if (gamepad2.b) {
            rightSlide.setTargetPosition(2068);
            leftSlide.setTargetPosition(-2068);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftControl(1);
        } else if (gamepad2.y) {
            rightSlide.setTargetPosition(2852);
            leftSlide.setTargetPosition(-2852);
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
            rightSlide.setTargetPosition(rightSlide.getCurrentPosition() + 50);
            leftSlide.setTargetPosition(leftSlide.getCurrentPosition() - 50);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftControl(.5);
        } else if (gamepad2.left_stick_y < -0.2) {
            rightSlide.setTargetPosition(rightSlide.getCurrentPosition() - 50);
            leftSlide.setTargetPosition(leftSlide.getCurrentPosition() + 50);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftControl(.5);
        }
    }

    private void automatedOuttake() {
        if (gamepad2.dpad_up) {
            rotateClaw.setPosition(1);
            rightSlide.setTargetPosition(1250);
            leftSlide.setTargetPosition(-1250);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftControl(1);
            sleep(1000);
            leftClaw.setPosition(0);
            rightClaw.setPosition(0);
            sleep(1000);
            leftClaw.setPosition(1);
            rightClaw.setPosition(1);
        }
    }

    //Create turn sequence
    TrajectorySequence turn = drive.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(90)))
            .turn(Math.toRadians(190))
            .waitSeconds(1)
            .build();

    private void turn180() {
        if (gamepad1.y) {
            drive.followTrajectorySequence(turn);
        }
    }


    @Override
    public void runOpMode () {

        //Define all movement motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightRear");

        //Define all Slide motors
        //leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        //rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        //Define All servos
        //rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        //leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        //rotateClaw = hardwareMap.get(Servo.class, "rotateClaw");

        //Set Zero Power Behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Reverse motors
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        //Set up encoders
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        sleep(100);
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                driverControl();
                cascadinglift();
                resetLift();
                clawControl();
                rotateControl();
                turn180();
            }
        }
    }
}
