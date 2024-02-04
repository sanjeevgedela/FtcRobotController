package org.firstinspires.ftc.teamcode.SourceCode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SourceCode.TeleOp.TeleOpTest2;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class Equipment {

    //Define motors
    public DcMotorEx leftSlide = null;
    public DcMotorEx rightSlide = null;
    //    //Lifts the lift
    //Define motors
    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftBack = null;
    public DcMotorEx rightBack = null;

    //Define servos
    public Servo rightClaw = null;
    public Servo leftClaw = null;
    public Servo rotateClaw = null;
    public Servo plane = null;

    //Define camera
    public OpenCvCamera webcam = null;

    public enum Mode{
        TELEOP,
        AUTON
    }

    public void CamInit(HardwareMap hardwareMap){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    public void camera(OpenCvPipeline pipeline){
        webcam.setPipeline(pipeline);
    }

    public void clawMap(HardwareMap hardwareMap) {
        //Define All servos
        rightClaw =hardwareMap.get(Servo.class,"rightClaw");
        leftClaw =hardwareMap.get(Servo.class,"leftClaw");
        rotateClaw =hardwareMap.get(Servo.class,"rotateClaw");
    }

    public void clawInit() {
        //Set Ranges
        rightClaw.scaleRange(0.1, 0.4);
        leftClaw.scaleRange(0, 0.4);
        rotateClaw.scaleRange(0.65, 1);
    }

    public void planeMap(HardwareMap hardwareMap) {
        //Define All servos
        plane = hardwareMap.get(Servo.class,"plane");
    }

    public void planeInit() {
        //Set Ranges
        plane.scaleRange(0, 0.4);
        plane.setDirection(Servo.Direction.FORWARD);
    }


    public void driveMap(HardwareMap hardwareMap) {
        rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
        leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
        rightBack = hardwareMap.get(DcMotorEx.class,"rightRear");
        leftBack = hardwareMap.get(DcMotorEx.class,"leftRear");

        //xRail = hardwareMap.get(DcMotorEx.class, "xRail");
    }

    //initialize for TeleOp
    public void driveInit() {
        double reset = 0;
        rightFront.setPower(reset);
        //frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setPower(reset);
        leftBack.setPower(reset);
        rightBack.setPower(reset);

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void slideMap(HardwareMap hardwareMap){
        //Define all Slide motors
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
    }

    public void slideInit(){
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Reverse motors
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        //Set up encoders
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void initialize(Mode type, HardwareMap hardwareMap){
        switch(type){
            case AUTON:
                slideMap(hardwareMap);
                slideInit();
                clawMap(hardwareMap);
                clawInit();
                CamInit(hardwareMap);
                break;

            case TELEOP:
                slideMap(hardwareMap);
                slideInit();
                clawMap(hardwareMap);
                clawInit();
                driveMap(hardwareMap);
                driveInit();
                CamInit(hardwareMap);
                planeMap(hardwareMap);
                planeInit();
                break;
        }
    }
}
