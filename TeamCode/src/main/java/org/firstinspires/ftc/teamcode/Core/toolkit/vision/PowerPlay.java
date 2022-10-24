package org.firstinspires.ftc.teamcode.Core.toolkit.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class  PowerPlay extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public int location = -1;
    static final Rect ROI = new Rect(
            new Point(90, 60),
            new Point(160, 120));


    public PowerPlay(Telemetry t) {
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input) {
        System.out.println("Thread2: " + Thread.currentThread().getName());
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Mat box = mat.submat(ROI);

        Scalar purplelowHSV = new Scalar(90, 50, 70);
        Scalar purplehighHSV = new Scalar(128, 255, 255);

        Scalar greenlowHSV = new Scalar(90, 50, 70);
        Scalar greenhighHSV = new Scalar(128, 255, 255);

        Scalar orangelowHSV = new Scalar(10, 100, 20);
        Scalar orangehighHSV = new Scalar(25, 255, 255);

        Mat purpleMat = new Mat();
        Mat greenMat = new Mat();
        Mat orangeMat = new Mat();

        Core.inRange(box, orangelowHSV, orangehighHSV, orangeMat);
        Core.inRange(box, greenlowHSV, greenhighHSV, greenMat);
        Core.inRange(box, purplelowHSV, purplehighHSV, purpleMat);





        //rectangle(image, rect, Scalar(0,255,0), 1, 8, 0);

        double OrangeValue = Core.sumElems(box).val[0] / ROI.area() / 255;
        double PurpleValue = Core.sumElems(box).val[0] / ROI.area() / 255;
        double GreenValue = Core.sumElems(box).val[0] / ROI.area() / 255;

        box.release();


        telemetry.addData("Orange Raw Value", (int) Core.sumElems(box).val[0]);

        telemetry.addData("Orange Percentage", Math.round(OrangeValue * 100) + "%");

        telemetry.addData("Purple Raw Value", (int) Core.sumElems(box).val[0]);

        telemetry.addData("Purple Percentage", Math.round(PurpleValue * 100) + "%");

        telemetry.addData("Green Raw Value", (int) Core.sumElems(box).val[0]);

        telemetry.addData("Green Percentage", Math.round(GreenValue * 100) + "%");

        Imgproc.rectangle(input, ROI, new Scalar(0, 255, 0), 4);

//        telemetry.addData("box", OrangeValue);
        telemetry.update();


        if (PurpleValue > OrangeValue && PurpleValue > GreenValue) {
            location = 2;
        } else if (GreenValue > PurpleValue && GreenValue > OrangeValue) {
            location = 1;
        } else if (OrangeValue > PurpleValue && OrangeValue > GreenValue) {
            location = 3;
        }
        telemetry.addData("Location:", +location);
        return input;
    }
}


