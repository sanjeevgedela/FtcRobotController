package org.firstinspires.ftc.teamcode.SourceCode.Subsystems.Pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class PixelPipeline extends OpenCvPipeline{
    Telemetry telemetry;
    private Scalar lowerBound = new Scalar(0, 0, 200); // Lower bound for white color in HSV
    private Scalar upperBound = new Scalar(180, 30, 255); // Upper bound for white color in HSV
    int midx = 320;
    Scalar lowHSV = new Scalar(20, 70, 80); // lenient lower bound HSV for yellow
    Scalar highHSV = new Scalar(32, 255, 255); // lenient higher bound HSV for yellow
    int potentialAngle = 0;


    public PixelPipeline(Telemetry t) {
            telemetry = t;
        }
        @Override
        public Mat processFrame(Mat input) {
            // Convert the input frame to the HSV color space
            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);

            // Threshold the image to find pixels within the specified color range
            Core.inRange(input, lowerBound, upperBound, input);

            // Find contours in the binary image
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(input, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            List<Integer> areas = new ArrayList<>();
            List<Integer> MidX = new ArrayList<>();

            // Iterate through the contours and draw rectangles around them
            for (MatOfPoint contour : contours) {
                MatOfPoint2f approxCurve = new MatOfPoint2f();
                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

                // Approximate the contour with a polygon
                double epsilon = 0.02 * Imgproc.arcLength(contour2f, true);
                Imgproc.approxPolyDP(contour2f, approxCurve, epsilon, true);

                // Convert the polygon to a bounding rectangle
                Rect rect = Imgproc.boundingRect(new MatOfPoint(approxCurve.toArray()));

                // Draw a rectangle around the detected region
                Imgproc.rectangle(input, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new Scalar(255, 0, 0), 2);
                areas.add(rect.width * rect.height);
                MidX.add(rect.x + rect.width / 2);

                Imgproc.line(input, new Point(midx, 235), new Point(midx, 245), new Scalar(0, 0, 255), 5);
                Imgproc.line(input, new Point(280, 235), new Point(280, 245), new Scalar(0, 0, 255), 5);
            }

            for (int i = 0; i < areas.size(); i++) {
                int max = 0;

                if (areas.get(i) > max) {
                    max = areas.get(i);
                    midx = MidX.get(i);

                }
            }

            // Return the processed frame
            potentialAngle = ((midx - 360) / 15);
            telemetry.addData("angle", potentialAngle);

            return input;
        }

        public Integer getCenterX() {
            return midx;
        }
}
