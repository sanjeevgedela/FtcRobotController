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

@TeleOp(name="FieldCentricTest", group = "tests")
public class FieldCentricTest extends LinearOpMode {

    //Define motors
    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftBack = null;
    public DcMotorEx rightBack = null;


    double movement;
    double rotation;
    double strafe;
    double inverseMovement;

    public void driverControl() {
        movement = gamepad1.left_stick_y;
        rotation = gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;
        inverseMovement = -movement;

        double magnitude = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2));
        double direction = Math.atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        boolean precision = gamepad1.right_bumper;

        //INFO Increasing speed to a maximum of 1
        double lf = magnitude * Math.sin(direction + Math.PI / 4) + rotation;
        double lb = magnitude * Math.cos(direction + Math.PI / 4) + rotation;
        double rf = magnitude * Math.cos(direction + Math.PI / 4) - rotation;
        double rb = magnitude * Math.sin(direction + Math.PI / 4) - rotation;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(inverseMovement) + Math.abs(rotation) + Math.abs(strafe), 1);
        double frontLeftPower = (inverseMovement + rotation + strafe) / denominator;
        double backLeftPower = (inverseMovement - rotation + strafe) / denominator;
        double frontRightPower = (inverseMovement - rotation - strafe) / denominator;
        double backRightPower = (inverseMovement + rotation - strafe) / denominator;


        double hypot = Math.hypot(movement, strafe);
        double ratio;
        if (movement == 0 && strafe == 0)
            ratio = 1;
        else if (precision)
            ratio = hypot / (Math.max(Math.max(Math.max(Math.abs(lf), Math.abs(lb)), Math.abs(rb)), Math.abs(rf))) / 2;
        else
            ratio = hypot / (Math.max(Math.max(Math.max(Math.abs(lf), Math.abs(lb)), Math.abs(rb)), Math.abs(rf)));

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);


        telemetry.addData("lf:", lf);
        telemetry.addData("lb:", lb);
        telemetry.addData("rf:", rf);
        telemetry.addData("rb:", rb);
        telemetry.addData("frontLeftPower:", frontLeftPower);
        telemetry.addData("backLeftPower:", backLeftPower);
        telemetry.addData("frontRightPower:", frontRightPower);
        telemetry.addData("backRightPower:", backRightPower);

    }

    @Override
    public void runOpMode () {
        //Create Mecanum Drive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        //Define all movement motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightRear");

        //Set Zero Power Behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Reverse motors
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);


        //Set up encoders
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();
        sleep(100);
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                driverControl();

                if (gamepad1.left_stick_button){
                    drive.update();
                    Pose2d myPose = drive.getPoseEstimate();
                    double heading = Math.toDegrees(myPose.getHeading());

                    double targetHeading = (Math.atan(gamepad1.left_stick_y/gamepad1.left_stick_x));

                    TrajectorySequence findHeading = drive.trajectorySequenceBuilder(new Pose2d(myPose.getX(), myPose.getY(), targetHeading))
                            .turn(Math.toRadians(190))
                            .waitSeconds(1)
                            .build();

                    telemetry.addData("Heading:", heading);

                    drive.followTrajectorySequence(findHeading);

                    telemetry.update();
                }
            }
        }
    }
}
