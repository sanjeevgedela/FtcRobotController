package org.firstinspires.ftc.teamcode.SourceCode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SourceCode.Subsystems.Equipment;
import org.firstinspires.ftc.teamcode.SourceCode.Subsystems.TeleopCommands;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "POTENTIAL")
public class TeleOpTest4 extends LinearOpMode {
    Equipment equip = new Equipment();
    TeleopCommands commands = new TeleopCommands();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        commands.init(drive, hardwareMap);

        waitForStart();
        while(opModeIsActive()){
            commands.activate(gamepad1, gamepad2, drive, telemetry);
        }
    }
}