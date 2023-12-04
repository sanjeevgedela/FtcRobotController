package org.firstinspires.ftc.teamcode.SourceCode.Camera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp (name = "RedColorTest", group = "CameraTests")
public class RedColorTest extends LinearOpMode {

    public OpenCvCamera webcam;


    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        RedPipeline scanner = new RedPipeline(telemetry);
        webcam.setPipeline(scanner);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        waitForStart();
        while (opModeIsActive()) {
            FtcDashboard.getInstance().startCameraStream(webcam, 120);
        }
    }
}
class RedPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    int correctlocation = 3;
    Mat mat = new Mat();
    public enum Location{
        RIGHT,
        MIDDLE,
        LEFT
    }
    private Location location;
    static final Rect BMiddle = new Rect(
            new Point(120, 170),
            new Point(450, 220));
    static final Rect BRight = new Rect(
            new Point(480, 180),
            new Point(640, 260));
    static final double PERCENT_COLOR_THRESHOLD = 0.05;
    public RedPipeline(Telemetry t) {telemetry = t;}

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input,mat,Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(0,100,85);
        Scalar highHSV = new Scalar(10,255,255);

        Core.inRange(mat,lowHSV,highHSV,mat);

        Mat middle = mat.submat(BMiddle);
        Mat right = mat.submat(BRight);

        double middleValue = Core.sumElems(middle).val[0] / BMiddle.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / BRight.area() / 255;

        middle.release();
        right.release();

        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Middle raw value", (int) Core.sumElems(middle).val[0]);
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");
        telemetry.addData("Middle percentage", Math.round(middleValue * 100) + "%");


        boolean onRight = rightValue >PERCENT_COLOR_THRESHOLD;
        boolean onMiddle = middleValue>PERCENT_COLOR_THRESHOLD;

        if (onMiddle){
            correctlocation = 2;
            telemetry.addData("LOCATION!:","MIDDLE");
        }
        else if (onRight){
            correctlocation = 1;
            telemetry.addData("LOCATION!:","RIGHT");
        }
        else{
            correctlocation = 3;
            telemetry.addData("LOCATION!:","LEFT");
        }
        telemetry.update();
        Scalar False = new Scalar(0,100,85
        );
        Scalar True = new Scalar(10,255,255);


        Imgproc.cvtColor(mat,mat,Imgproc.COLOR_GRAY2RGB);
        Imgproc.rectangle(mat,BRight , location == Location.RIGHT? True:False);
        Imgproc.rectangle(mat,BMiddle, location == Location.MIDDLE? True :False);
        return mat;
    }
    public Location getLocation(){
        return location;
    }
}