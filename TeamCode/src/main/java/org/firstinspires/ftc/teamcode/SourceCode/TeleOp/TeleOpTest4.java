package org.firstinspires.ftc.teamcode.SourceCode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SourceCode.Subsystems.Equipment;
import org.firstinspires.ftc.teamcode.SourceCode.Subsystems.Pipelines.PixelPipeline;
import org.firstinspires.ftc.teamcode.SourceCode.Subsystems.TeleopCommands;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "POTENTIAL")
public class TeleOpTest4 extends LinearOpMode {
    Equipment equip = new Equipment();

    @Override
    public void runOpMode() throws InterruptedException {
        //equip.CamInit(hardwareMap);
        PixelPipeline pipe = new PixelPipeline(telemetry);
        TeleopCommands commands = new TeleopCommands(equip);
        //commands.init(new SampleMecanumDrive(hardwareMap), hardwareMap, Equipment.Mode.TELEOP);

        while(opModeInInit()){
            //FtcDashboard.getInstance().startCameraStream(equip.webcam, 120);
            //equip.camera(pipe);
        }

        waitForStart();
        while(opModeIsActive()){
            commands.activate(gamepad1, gamepad2, new SampleMecanumDrive(hardwareMap), telemetry, pipe.getCenterX());
        }
    }
}