package org.firstinspires.ftc.teamcode.commands.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class clawControl {

    //Define servos
        public Servo rightClaw = null;
        public Servo leftClaw = null;
        public Servo rotateClaw = null;



        public void clawMap(HardwareMap hardwareMap) {
        //Define All servos
        rightClaw =hardwareMap.get(Servo.class,"rightClaw");
        leftClaw =hardwareMap.get(Servo.class,"leftClaw");
        rotateClaw =hardwareMap.get(Servo.class,"rotateClaw");
    }

    public void initialize() {
        //Set Ranges
        rightClaw.scaleRange(0.1, 0.4);
        leftClaw.scaleRange(0, 0.4);
        rotateClaw.scaleRange(0.65, 1);
    }

}
