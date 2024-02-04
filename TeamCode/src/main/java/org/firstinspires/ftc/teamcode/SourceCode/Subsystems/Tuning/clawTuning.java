package org.firstinspires.ftc.teamcode.SourceCode.Subsystems.Tuning;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SourceCode.Subsystems.Equipment;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class clawTuning extends LinearOpMode {

    public static double DISTANCE = 50;
    Equipment equip = new Equipment();

    @Override
    public void runOpMode() throws InterruptedException {
            equip.initialize(Equipment.Mode.TELEOP, hardwareMap);

            waitForStart();

            while (opModeIsActive() && !isStopRequested()) {
                equip.rightClaw.setPosition(1);
                equip.leftClaw.setPosition(1);
                sleep(1000);
                equip.rightClaw.setPosition(0);
                equip.leftClaw.setPosition(0);
            }
    }
}
