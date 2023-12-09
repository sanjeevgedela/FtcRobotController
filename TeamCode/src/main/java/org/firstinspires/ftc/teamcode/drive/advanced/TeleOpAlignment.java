package org.firstinspires.ftc.teamcode.drive.advanced;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commands.TeleOp.driverControl;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "AlignBlue", group = "tests")
public class TeleOpAlignment extends LinearOpMode {

    Gamepad x = new Gamepad();

    //Should setup driving

    //Setting up all points that will be used
    Pose2d blueBackDrop = new Pose2d(62,36,Math.toRadians(0));
    Pose2d redBackDrop = new Pose2d(62,-36,Math.toRadians(0));
    Pose2d current = PoseStorage.currentPose;

    //Declare a PIDF Controller to regulate heading
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

    double distance;
    double direction;

    //Setting up enum for determining Alliance Color
    enum Side{
        RED,
        BLUE
    }

    //Determining whether the robot is to align to the backdrop or normal driver control
    enum driveType{
        NORMAL,
        ALIGN
    }

    //Start settings as placeholder for the enum
    private Side side = Side.RED;
    private driveType mode = driveType.NORMAL;
    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftBack = null;
    public DcMotorEx rightBack = null;

    double movement;
    double rotation;
    double strafe;

    public void driverControl() {
        movement = gamepad1.left_stick_y;
        rotation = gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;

        double magnitude = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2));
        double direction = Math.atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        boolean precision = gamepad1.right_bumper;

        //INFO Increasing speed to a maximum of 1
        double lf = magnitude * Math.sin(direction + Math.PI / 4) + rotation;
        double lb = magnitude * Math.cos(direction + Math.PI / 4) + rotation;
        double rf = magnitude * Math.cos(direction + Math.PI / 4) - rotation;
        double rb = magnitude * Math.sin(direction + Math.PI / 4) - rotation;

        double hypot = Math.hypot(movement, strafe);
        double ratio;
        if (movement == 0 && strafe == 0)
            ratio = 1;
        else if (precision)
            ratio = hypot / (Math.max(Math.max(Math.max(Math.abs(lf), Math.abs(lb)), Math.abs(rb)), Math.abs(rf))) / 2;
        else
            ratio = hypot / (Math.max(Math.max(Math.max(Math.abs(lf), Math.abs(lb)), Math.abs(rb)), Math.abs(rf)));

        leftFront.setPower(ratio * lf);
        leftBack.setPower(ratio * lb);
        rightFront.setPower(ratio * rf);
        rightBack.setPower(ratio * rb);
    }

    //Calculates distance and direction from the blue scoring backdrop
    public void BlueAlignOut() {
            distance = Math.hypot((blueBackDrop.getX() - current.getX()), blueBackDrop.getY() - current.getY());
            direction = Math.atan2((blueBackDrop.getY() - current.getY()), (blueBackDrop.getX() - current.getX()));
    }

    //Calculates distance and direction from the red scoring backdrop
    public void RedAlignOut() {
            distance = Math.hypot((redBackDrop.getX() - current.getX()), redBackDrop.getY() - current.getY());
            direction = Math.atan2((redBackDrop.getY() - current.getY()), (redBackDrop.getX() - current.getX()));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //Declare drive
        SampleMecanumDrive drive= new SampleMecanumDrive(hardwareMap);

        //Set up mecanum

        rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
        leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
        rightBack = hardwareMap.get(DcMotorEx.class,"rightRear");
        leftBack = hardwareMap.get(DcMotorEx.class,"leftRear");

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

        //Retrieve our pose from the PoseStorage.currentPose static field
        drive.getLocalizer().setPoseEstimate(PoseStorage.currentPose);

        //Set input bounds for the heading controller
        headingController.setInputBounds(-Math.PI, Math.PI);

        waitForStart();

        //Find out alliance side
        if (current.getY() > 0) {
            side = Side.BLUE;
        } else if (current.getY() < 0) {
            side = Side.RED;
        }

        while (opModeIsActive()){

            //Determine whether to use red or blue backboard
            switch(side){
                case RED:
                    RedAlignOut();
                    break;
                case BLUE:
                    BlueAlignOut();
                    break;
            }

            //Set up swirching from Align to normal
            switch(mode){
                case NORMAL:
                    if (gamepad1.a){
                        mode = driveType.ALIGN;
                    }
                    driverControl();
                    break;
                case ALIGN:
                    if (gamepad1.b){
                        mode = driveType.NORMAL;
                    }
                    driverControl();
                    headingController.setTargetPosition(direction);
                    break;
            }

            //Telemetry for static pose storage
            telemetry.addData("Pose Storage x:", current);
            telemetry.addData("Pose Storage y:", current);
            telemetry.addData("Pose Storage heading:", current);

            //Telemetry for localizer based pose estimate
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("Localizer x: ", poseEstimate.getX());
            telemetry.addData("Localizer y: ", poseEstimate.getY());
            telemetry.addData("Localizer heading: ", poseEstimate.getHeading());

            //Telemetry for making sure calculations are correct
            telemetry.addData("Direction: ", Math.toDegrees(direction));
            telemetry.addData("Distance: ", distance);

            //Telemetry for the mode
            telemetry.addData("Mode:", mode);

            telemetry.update();
        }
    }
}
