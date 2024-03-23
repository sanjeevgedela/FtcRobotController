package org.firstinspires.ftc.teamcode.SourceCode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SourceCode.Subsystems.HuskyStackDetection;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.trajectorysequence.TrajectorySequence;

@Autonomous(name = "husky")
public class HuskyALIGN extends LinearOpMode {

    SampleMecanumDrive drive;
    public HuskyStackDetection detect;
    public HuskyLens husky;
    int dist = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        detect = new HuskyStackDetection(telemetry, husky);
        detect.init(hardwareMap);



        waitForStart();

        sleep(500);

        dist = detect.method();
        telemetry.addData("dist", dist);

        sleep(1000);
    }
}
