package org.firstinspires.ftc.teamcode.SourceCode.TeleOp.Camera;

import com.qualcomm.robotcore.hardware.HardwareMap;

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

public class BluePipe extends OpenCvPipeline {
    Telemetry telemetry;
    int correctlocation = 3;
    Mat mat = new Mat();

    public OpenCvCamera webcam;

    public enum Location{
        RIGHT,
        MIDDLE,
        LEFT
    }

    public void map(HardwareMap hardwareMap){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
    }

    public void initialize(){
        BluePipe scanner = new BluePipe(telemetry);
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
    }

    private Location location;
    static final Rect BMiddle = new Rect(
            new Point(145, 270),
            new Point(345, 200));
    static final Rect BRight = new Rect(
            new Point(450, 240),
            new Point(580, 370));
    static final double PERCENT_COLOR_THRESHOLD = 0.15;
    public BluePipe(Telemetry t) {telemetry = t;}

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input,mat,Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(208, 91, 60);
        Scalar highHSV = new Scalar(254, 100, 100);

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
        Scalar False = new Scalar(208, 91, 60);
        Scalar True = new Scalar(254, 100, 100);


        Imgproc.cvtColor(mat,mat,Imgproc.COLOR_GRAY2RGB);
        Imgproc.rectangle(mat,BRight , location == Location.RIGHT? True:False);
        Imgproc.rectangle(mat,BMiddle, location == Location.MIDDLE? True :False);
        return mat;
    }
    public Location getLocation(){
        return location;
    }
}