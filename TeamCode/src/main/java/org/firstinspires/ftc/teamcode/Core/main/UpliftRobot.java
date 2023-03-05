package org.firstinspires.ftc.teamcode.Core.main;


import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.Core.Threads.DriveThread;
//import org.firstinspires.ftc.teamcode.Core.Threads.OperatorThread;
import org.firstinspires.ftc.teamcode.Core.toolkit.vision.ConeAlignmentBlue;
import org.firstinspires.ftc.teamcode.Core.toolkit.vision.ConeAlignmentRed;
import org.firstinspires.ftc.teamcode.Core.toolkit.vision.PowerPlay;
import org.firstinspires.ftc.teamcode.Core.toolkit.vision.PowerPlay2;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class UpliftRobot
{
    DcMotor leftFront, rightFront, leftBack, rightBack, slide1, slide2;
    Servo grabber, arm1, arm2, fourBar1, fourBar2, twister;
    DistanceSensor coneDetector;
    TouchSensor magnet;
    OpenCvCamera webcam;
    DcMotor odoRight;
    public BNO055IMU imu;

//    AdafruitBNO055IMU gyro;



    double arm1HighPos = .95;
    double arm2HighPos = 0.05;

    double arm1LowPos = 0.33 ;
    double arm2LowPos = 0.67;



//    double bar1FrontPos = .14;
//    double bar2FrontPos = .86;
//
//    double bar1BackPos = .8;
//    double bar2BackPos = .2;

    double bar1FrontPos = .77;
    double bar2FrontPos = .23;

    double bar1BackPos = .18;
    double bar2BackPos = .82;

    double grabberOpenPos = .033;
    double grabberClosePos = 0.24;

    double twisterUpPos = .84;
    double twisterDownPos = .16;

    double arm1StackPos5 = .57;
    double arm2StackPos5 = .38;

    double arm1StackPos4 = .56;
    double arm2StackPos4 = .44;

    double arm1StackPos3 = .51;
    double arm2StackPos3 = .48;

    double arm1StackPos2 = .41;
    double arm2StackPos2 = .52;

    double arm1StackPos1 = .35;
    double arm2StackPos1 = .65;


//    double arm1HighPos = .87;
//    double arm2HighPos = .15;
//
//    double arm1LowPos = 0.24 ;
//    double arm2LowPos = 0.74;

//    double arm1StackPos5 = .39;
//    double arm2StackPos5 = .56;
//
//    double arm1StackPos4 = .35;
//    double arm2StackPos4 = .60;
//
//    double arm1StackPos3 = .30;
//    double arm2StackPos3 = .65;
//
//    double arm1StackPos2 = .26;
//    double arm2StackPos2 = .69;
//
//    double arm1StackPos1 = .14;
//    double arm2StackPos1 = .81;

    public PowerPlay pipeline1;
    public PowerPlay2 pipeline4;
    public ConeAlignmentBlue pipeline2;
    public ConeAlignmentRed pipeline3;
    public LinearOpMode opMode;
    public HardwareMap hardwareMap;

    public UpliftRobot(LinearOpMode opMode) {
        this.opMode = opMode;
        getHardware();
    }

    public void getHardware() {

        hardwareMap = opMode.hardwareMap;

        initializeCamera();


        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");

        slide1 = hardwareMap.get(DcMotor.class, "slide1");
        slide2 = hardwareMap.get(DcMotor.class, "slide2");

        grabber = hardwareMap.get(Servo.class, "grabber");

        fourBar1 = hardwareMap.get(Servo.class,"fourBar1");
        fourBar2= hardwareMap.get(Servo.class,"fourBar2");

        twister = hardwareMap.get(Servo.class,"twister");

        coneDetector = hardwareMap.get(DistanceSensor.class, "coneDetector");


        magnet = hardwareMap.get(TouchSensor.class, "magnet");



        odoRight = hardwareMap.get(DcMotor.class, "odoRight");

//        gyro = hardwareMap.get(AdafruitBNO055IMU.class, "gyro");
//        AdafruitBNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        gyro.initialize(parameters);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm2.setDirection(Servo.Direction.REVERSE);
        arm1.setDirection(Servo.Direction.REVERSE);

    }

    public void initializeCamera()
    {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline1 = new PowerPlay(opMode.telemetry);
        pipeline2 = new ConeAlignmentBlue(opMode.telemetry);
        pipeline3 = new ConeAlignmentRed(opMode.telemetry);
        pipeline4 = new PowerPlay2(opMode.telemetry);

        webcam.setPipeline(pipeline4);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {

                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//
//                pipeline3 = new ConeAlignmentRed(opMode.telemetry);
//                webcam.setPipeline(pipeline3);
//                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }

    public OpenCvCamera getWebcam()
    {
        return webcam;
    }

    public Servo getFourBar1() {
        return fourBar1;
    }

    public Servo getFourBar2() {
        return fourBar2;
    }

    public Servo getTwister()
    {
        return twister;
    }

    public Servo getGrabber()
    {
        return grabber;
    }

    public Servo getArm1()
    {
        return arm1;
    }

    public Servo getArm2()
    {
        return arm2;
    }

    public DcMotor getLeftFront() {
        return leftFront;
    }

    public DcMotor getLeftBack() {
        return leftBack;
    }

    public DcMotor getRightBack() {
        return rightBack;
    }

    public DcMotor getRightFront() {
        return rightFront;
    }

    public DcMotor getSlide1()
    {
        return slide1;
    }

    public DcMotor getSlide2()
    {
        return slide2;
    }

    public DistanceSensor getConeDetector()
    {
        return coneDetector;
    }



    public TouchSensor getMagnet()
    {
        return magnet;
    }


    public DcMotor getOdoRight()
    {
        return odoRight;
    }


    public double getTwisterUpPos()
    {
        return twisterUpPos;
    }

    public double getTwisterDownPos()
    {
        return twisterDownPos;
    }

    public double getGrabberOpenPos()
    {
        return grabberOpenPos;
    }

    public double getGrabberClosePos()
    {
        return grabberClosePos;
    }

    public double getArm1HighPos()
    {
        return arm1HighPos;
    }

    public double getArm2HighPos()
    {
        return arm2HighPos;
    }

    public double getArm1LowPos()
    {
        return arm1LowPos;
    }

    public double getArm2LowPos()
    {
        return arm2LowPos;
    }

    public double getBar1FrontPos()
    {
        return bar1FrontPos;
    }

    public double getBar2FrontPos()
    {
        return bar2FrontPos;
    }

    public double getBar1BackPos()
    {
        return bar1BackPos;
    }

    public double getBar2BackPos()
    {
        return bar2BackPos;
    }

    public double getArm1StackPos5()
    {
        return  arm1StackPos5;
    }

    public double getArm2StackPos5()
    {
        return arm2StackPos5;
    }

    public double getArm1StackPos4() {
        return arm1StackPos4;
    }

    public double getArm2StackPos4() {
        return arm2StackPos4;
    }

    public double getArm1StackPos3() {
        return arm1StackPos3;
    }

    public double getArm2StackPos3() {
        return arm2StackPos3;
    }

    public double getArm1StackPos2()
    {
        return arm1StackPos2;
    }

    public double getArm2StackPos2()
    {
        return arm2StackPos2;
    }

    public double getArm1StackPos1()
    {
        return arm1StackPos1;
    }

    public double getArm2StackPos1()
    {
        return arm2StackPos1;
    }

}
