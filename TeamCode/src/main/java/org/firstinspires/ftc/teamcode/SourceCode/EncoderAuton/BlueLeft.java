package org.firstinspires.ftc.teamcode.SourceCode.EncoderAuton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.SourceCode.EncoderAuton.Objects.EncoderMecanum;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;

@Autonomous(name = "BlueLeftParkENC", group = "Encoder")
public class BlueLeft extends LinearOpMode {

    EncoderMecanum drive = new EncoderMecanum();

    Servo rotateClaw;
    Servo rightClaw;
    Servo leftClaw;

    public void rotateControl(double rotate){
        rotateClaw.setPosition(rotate);
    }

    public void clawControl(double left, double right){
        leftClaw.setPosition(left);
        rightClaw.setPosition(right);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);

        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rotateClaw = hardwareMap.get(Servo.class, "rotateClaw");

        //Set Ranges
        rightClaw.scaleRange(0.1, 0.4);
        leftClaw.scaleRange(0, 0.4);
        rotateClaw.scaleRange(0.65, 1);

        drive.mecanum(hardwareMap);
        drive.initialize();

        waitForStart();

        clawControl(0,0);
        rotateControl(1);
        drive.encoderMovementX(4, 0.5);
        sleep(10000);
        drive.encoderMovementY(40,0.4);
        sleep(10000);
        clawControl(1,1);
        sleep(1000);


    }
}
