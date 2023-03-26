package org.firstinspires.ftc.teamcode.Core.toolkit.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ConeAlignmentBlue extends OpenCvPipeline
{
    Telemetry telemetry;
    Mat mat = new Mat();

    public double leftValue;


    static final Rect LEFT_ROI = new Rect(
            new Point(160, 50),
            new Point(220, 130));


    public ConeAlignmentBlue(Telemetry t) {
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input)
    {
        System.out.println("Thread2: " + Thread.currentThread().getName());
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar blueLowHSV = new Scalar(90, 80, 60);
        Scalar blueHighHSV = new Scalar(120, 255, 255);

        Core.inRange(mat, blueLowHSV, blueHighHSV, mat);

        Mat left = mat.submat(LEFT_ROI);

        leftValue = Core.sumElems(left).val[0] / 255;


        Imgproc.rectangle(input, LEFT_ROI, new Scalar(0, 255, 0), 4);

        telemetry.addData("Left Raw Value", (leftValue / 255) * 6.4);

        telemetry.update();


        return input;

    }
}
