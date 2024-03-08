package org.firstinspires.ftc.teamcode.SourceCode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SourceCode.Auton.apriltag;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "AprilTAGS")
public class aprilTagTest extends LinearOpMode {
    //Define motors
    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftBack = null;
    public DcMotorEx rightBack = null;


    //Define servos
    public Servo rightClaw = null;
    public Servo leftClaw = null;
    public Servo rotateClaw = null;
    public Servo plane = null;

    double movement;
    double rotation;
    double strafe;
    int target;

    WebcamName webcam1;
    apriltag tag;
    double dist;

    public void driverControl(SampleMecanumDrive drive) {

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

        if (gamepad1.y) {
            drive.turn(Math.toRadians(180));
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        //Define all movement motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightRear");

        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 2");
        tag = new apriltag(webcam1);
        if(opModeInInit()) {
            while(opModeInInit()){
                tag.initAprilTag();
            }
        }

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        while (opModeIsActive()){
            driverControl(drive);
            tag.setType(apriltag.DETECT.LEFT, apriltag.COLOR.BLUE);
            tag.findTag(telemetry);
            telemetry.addData("distance", tag.calculate());
        }
    }
}
