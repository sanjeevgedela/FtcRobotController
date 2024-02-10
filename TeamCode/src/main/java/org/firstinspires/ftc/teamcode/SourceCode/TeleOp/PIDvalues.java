package org.firstinspires.ftc.teamcode.SourceCode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PersonalPID;

public class PIDvalues {
    public static double p = 0.0073, i = 0, d = 0.0001, f = 0.0013;

    public void setController(PersonalPID controller) {
        controller = new PersonalPID(p, i, d, f);
    }

    public void slideController(DcMotorEx rightSlide, DcMotorEx leftSlide, PersonalPID controller, double target, Telemetry telemetry) {
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
