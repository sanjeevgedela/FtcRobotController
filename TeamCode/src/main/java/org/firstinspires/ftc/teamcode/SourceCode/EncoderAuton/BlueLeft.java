package org.firstinspires.ftc.teamcode.SourceCode.EncoderAuton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SourceCode.EncoderAuton.Objects.EncoderMecanum;

@Autonomous(name = "BlueLeft")
public class BlueLeft extends LinearOpMode {

    EncoderMecanum drive = new EncoderMecanum();

    @Override
    public void runOpMode() throws InterruptedException {
        drive.mecanum(hardwareMap);
        drive.initialize();

        waitForStart();
        drive.encoderMovementY(4, 0.5);
        sleep(10000);
        drive.encoderMovementY(-40,0.4);
        sleep(10000);
    }
}
