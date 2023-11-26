package org.firstinspires.ftc.teamcode.SourceCode.EncoderAuton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SourceCode.EncoderAuton.Objects.EncoderMecanum;

@Autonomous(name = "BlueRightParkENC", group = "Encoder")
public class BlueRight extends LinearOpMode {

    EncoderMecanum drive = new EncoderMecanum();

    @Override
    public void runOpMode() throws InterruptedException {
        drive.mecanum(hardwareMap);
        drive.initialize();

        waitForStart();
        drive.encoderMovementX(48, 0.5);
        sleep(10000);
        drive.encoderMovementY(60,0.4);
        sleep(10000);
    }
}
