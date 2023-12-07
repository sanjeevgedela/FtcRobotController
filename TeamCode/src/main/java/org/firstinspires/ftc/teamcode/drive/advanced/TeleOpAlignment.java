package org.firstinspires.ftc.teamcode.drive.advanced;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.TeleOp.driverControl;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "AlignBlue", group = "tests")
public class TeleOpAlignment extends LinearOpMode {

    //Should setup driving
    private driverControl driver = new driverControl();

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
        driver.driverMap(hardwareMap);
        driver.initialize();

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
                    driver.driverControl();
                    break;
                case ALIGN:
                    if (gamepad1.b){
                        mode = driveType.NORMAL;
                    }
                    driver.driverControl();
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
