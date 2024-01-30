package org.firstinspires.ftc.teamcode.SourceCode.Subsystems.Pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BluePipeline extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();

    public enum Location{
        RIGHT,
        MIDDLE,
        LEFT
    }

    private volatile Location location;
    static final Rect BMiddle = new Rect(
            new Point(145, 160),
            new Point(295, 60));
    static final Rect BRight = new Rect(
            new Point(430, 200),
            new Point(560, 90));
    static final double PERCENT_COLOR_THRESHOLD = 0.15;
    public BluePipeline(Telemetry t) {telemetry = t;}

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

        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Middle raw value", (int) Core.sumElems(middle).val[0]);
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");
        telemetry.addData("Middle percentage", Math.round(middleValue * 100) + "%");


        boolean onRight = rightValue > PERCENT_COLOR_THRESHOLD;
        boolean onMiddle = middleValue > PERCENT_COLOR_THRESHOLD;

        if (onMiddle){
            telemetry.addData("LOCATION!:","MIDDLE");
            location = Location.MIDDLE;
        }
        else if (onRight){
            telemetry.addData("LOCATION!:","RIGHT");
            location = Location.RIGHT;
        }
        else{
            telemetry.addData("LOCATION!:","LEFT");
            location = Location.LEFT;
        }
        telemetry.update();
        Scalar False = new Scalar(0,100,85);
        Scalar True = new Scalar(10,255,255);


        Imgproc.cvtColor(mat,mat,Imgproc.COLOR_GRAY2RGB);
        Imgproc.rectangle(mat,BRight , location == Location.RIGHT? True:False);
        Imgproc.rectangle(mat,BMiddle, location == Location.MIDDLE? True :False);

        middle.release();
        right.release();
        return mat;
    }
    public Location getLocation(){
        return location;
    }
}
