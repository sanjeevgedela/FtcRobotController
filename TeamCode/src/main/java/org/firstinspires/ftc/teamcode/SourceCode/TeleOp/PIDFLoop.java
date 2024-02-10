package org.firstinspires.ftc.teamcode.SourceCode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.PersonalPID;

@Config
@TeleOp(name = "PID Loop")
public class PIDFLoop extends LinearOpMode {

    private PersonalPID controller;

    public static double p = 0.025, i = 0, d = 0.0001, f = 0.001;

    public static int target = 0;

    private DcMotorEx leftSlide;
    private DcMotorEx rightSlide;
    double ff = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PersonalPID(p, i, d, f);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){
            controller.setPIDF(p, i, d, f);
            int armPos = rightSlide.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            rightSlide.setPower(pid);
            leftSlide.setPower(pid);

            telemetry.addData("armPos", armPos);
            telemetry.addData("target", target);
            telemetry.addData("pid", pid);
            telemetry.update();
        }
    }
}
