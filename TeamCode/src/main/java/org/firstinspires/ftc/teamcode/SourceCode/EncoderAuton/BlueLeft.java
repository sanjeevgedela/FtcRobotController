package org.firstinspires.ftc.teamcode.SourceCode.EncoderAuton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SourceCode.EncoderAuton.Objects.EncoderMecanum;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;

@Autonomous(name = "BlueLeftParkENC", group = "Encoder")
public class BlueLeft extends LinearOpMode {

    EncoderMecanum drive = new EncoderMecanum();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);

        drive.mecanum(hardwareMap);
        drive.initialize();

        waitForStart();

        drive.encoderMovementX(4, 0.5);
        sleep(10000);
        drive.encoderMovementY(-40,0.4);
        sleep(10000);

        PoseStorage.currentPose = robot.getPoseEstimate();
    }
}
