package org.firstinspires.ftc.teamcode.Core.main;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Core.Threads.DriveThread;
import org.firstinspires.ftc.teamcode.Core.Threads.OperatorThread;
import org.firstinspires.ftc.teamcode.Core.toolkit.vision.PowerPlay;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class UpliftRobot {
    DcMotor leftFront, rightFront, leftBack, rightBack, slide1, slide2;
    BNO055IMU imu;
    Servo grabber, arm1, arm2;
    TouchSensor magneticSensor;
    OpenCvCamera webcam;



//    double grabberOpenPos = 0.23;
    double grabberOpenPos = 1;
    double grabberClosePos = 0.10;


    public PowerPlay pipeline;
    public LinearOpMode opMode;
    public HardwareMap hardwareMap;

    public UpliftRobot(LinearOpMode opMode) {
        this.opMode = opMode;
        getHardware();
//        PowerPlay pipeline = new PowerPlay(opMode.telemetry);
    }

    public void getHardware() {

        hardwareMap = opMode.hardwareMap;

        initializeCamera();

        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        slide1 = hardwareMap.get(DcMotor.class, "slide1");
        slide2 = hardwareMap.get(DcMotor.class, "slide2");

        grabber = hardwareMap.get(Servo.class, "grabber");

        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);

        magneticSensor = hardwareMap.get(TouchSensor.class, "magnetic_sensor");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    }

        public void initializeCamera()
        {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                pipeline = new PowerPlay(opMode.telemetry);
                webcam.setPipeline(pipeline);
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
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

    public TouchSensor getMagneticSensor()
    {
        return magneticSensor;
    }



    public double getGrabberOpenPos()
    {
        return grabberOpenPos;
    }

    public double getGrabberClosePos()
    {
        return grabberClosePos;
    }




}
