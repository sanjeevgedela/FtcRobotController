package org.firstinspires.ftc.teamcode.SourceCode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.Auto.IntakeOuttake;

@TeleOp(name = "AutoCommandsTest", group = "tests")
public class AutonCommandsTest extends LinearOpMode {

    IntakeOuttake equip = new IntakeOuttake();

    private void reset(){
        if(gamepad1.a) {
            equip.reset();
        }
    }

    private void scoreLow(){
        if(gamepad1.b){
            equip.scorePositionLow();
        }
    }

    private void scoreMid(){
        if(gamepad1.x){
            equip.scorePositionMid();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        equip.map(hardwareMap);
        equip.initialize();

        waitForStart();

        while(opModeIsActive()){
            reset();

            scoreLow();

            scoreMid();
        }
    }
}
