package org.firstinspires.ftc.teamcode.SourceCode.Subsystems;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

public class HuskyStackDetection {
    private Telemetry telemetry;
    private final int READ_PERIOD = 1;

    private HuskyLens huskyLens;

    double area = 0;
    double max = 0;

    int dist;

    Deadline rateLimit;

    public HuskyStackDetection(Telemetry t, HuskyLens hl) {telemetry = t; huskyLens = hl;}

    public void init(HardwareMap hardwareMap) {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);

        telemetry.update();
    }

    public int method(){
//        if (!rateLimit.hasExpired()) {
//            continue;
//        }
//        rateLimit.reset();

        /*
         * All algorithms, except for LINE_TRACKING, return a list of Blocks where a
         * Block represents the outline of a recognized object along with its ID number.
         * ID numbers allow you to identify what the device saw.  See the HuskyLens documentation
         * referenced in the header comment above for more information on IDs and how to
         * assign them to objects.
         *
         * Returns an empty array if no objects are seen.
         */
        HuskyLens.Block[] blocks = huskyLens.blocks();

        if (blocks.length > 0) {
            telemetry.addData("Block count", blocks.length);

            for (int i = 0; i < blocks.length; i++) {

                if (blocks[i].id == 1) {
                    telemetry.addData("Stack detected?", "Yes");
                    telemetry.addData("X", blocks[i].x);
                    telemetry.addData("Y", blocks[i].y);
                    area = blocks[i].x * blocks[i].y;
                    if(area > max){
                        max = area;
                    }
                    dist = (blocks[i].x - 220)/ 60;

                } else {
                    telemetry.addData("Stack detected?", "No");
                }
            }

        }
        telemetry.addData("distance in x", dist);
//
        telemetry.update();
        return dist;
    }

    }
