package org.firstinspires.ftc.teamcode.Core.toolkit.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ConeAlignmentRed extends OpenCvPipeline
{
    Telemetry telemetry;
    Mat mat = new Mat();

    public double leftValue;
    public double rightValue;

    static final Rect LEFT_ROI = new Rect(
            new Point(120, 50),
            new Point(200, 130));
    static final Rect RIGHT_ROI = new Rect(
            new Point(200, 50),
            new Point(280, 130));

    public ConeAlignmentRed(Telemetry t) {
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input)
    {
        System.out.println("Thread2: " + Thread.currentThread().getName());
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar redLowHSV = new Scalar(5, 80, 60);
        Scalar redHighHSV = new Scalar(25, 255, 255);

        Core.inRange(mat, redLowHSV, redHighHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        leftValue = Core.sumElems(left).val[0] / 255;
        rightValue = Core.sumElems(right).val[0] / 255;

        Imgproc.rectangle(input, LEFT_ROI, new Scalar(0, 255, 0), 4);
        Imgproc.rectangle(input, RIGHT_ROI, new Scalar(0, 255, 0), 4);

        telemetry.addData("Left Raw Value", leftValue);
        telemetry.addData("Right Raw Value", rightValue);

        telemetry.update();


        return input;

    }
}
