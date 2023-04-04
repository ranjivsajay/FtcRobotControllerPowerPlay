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
    Servo grabber1, grabber2, armLeft, armRight, fourBar, twister, extensionLeft, extensionRight, odoMid, gRotation, gPosition;
    DistanceSensor coneDetector, poleDetector;
    TouchSensor magnet;
    OpenCvCamera webcam;
    DcMotor odoRight;
    public BNO055IMU imu;

//    AdafruitBNO055IMU gyro;



    double armLeftHighPos = .95;

    double armRightHighPos = 0.05;

    double arm1LowPos = 0.33 ;
    double arm2LowPos = 0.67;

    double barFrontPos = .77;
    double barBackPos = .18;

    double grabber1OpenPos = .033;
    double grabber1ClosePos = 0.24;

    double grabber2OpenPos = 0.5;
    double grabber2ClosePos = 0.5;

    double twisterUpPos = .84;
    double twisterDownPos = .16;

    double arm1PickUpPos = .6;
    double arm2PickUpPos = .4;

    double arm1StackPos4 = arm1PickUpPos - 0.03;
    double arm2StackPos4 = arm2PickUpPos - 0.03;

    double arm1StackPos3 = arm1StackPos4 - 0.03;
    double arm2StackPos3 = arm2StackPos4 - 0.03;

    double arm1StackPos2 = arm1StackPos3 - 0.03;
    double arm2StackPos2 = arm2StackPos3 - 0.03;

    double arm1StackPos1 = arm1StackPos2 - 0.03;
    double arm2StackPos1 = arm2StackPos2 - 0.03;


    double rotationStack1 = 0;
    double rotationStack2 = rotationStack1 + 0.1;
    double rotationStack3 = rotationStack2 + 0.1;
    double rotationStack4 = rotationStack3 + 0.1;
    double rotationStack5 = rotationStack4 + 0.1;

    double positionStack1 = .98;
    double positionStack2 = positionStack1 - 0.06;
    double positionStack3 = positionStack2 - 0.06;
    double positionStack4 = positionStack3 - 0.06;
    double positionStack5 = positionStack4 - 0.06;
    double gPositionStore = 0.57;



    double extensionLeftIn = 1;
    double extensionRightIn = 0;

    double extensionLeftOut = 0.5;
    double extensionRightOut = 0.5;

    double extensionLeftTransfer = 0.5;
    double extensionRightTransfer = 0.5;

    double extensionLeftCyclePos = 0.5;
    double extensionRightCyclePos = 0.5;

    double odoMidUp = 0.35;
    double odoMidDown = 1;

    double gPositionTransfer = 0.5;
    double gRotationTransfer = 0.5;








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

        armLeft = hardwareMap.get(Servo.class, "armLeft");
        armRight = hardwareMap.get(Servo.class, "armRight");

        slide1 = hardwareMap.get(DcMotor.class, "slide1");
        slide2 = hardwareMap.get(DcMotor.class, "slide2");

        grabber1 = hardwareMap.get(Servo.class, "grabber1");
        grabber2 = hardwareMap.get(Servo.class, "grabber2");

        fourBar = hardwareMap.get(Servo.class,"fourBar");

        twister = hardwareMap.get(Servo.class,"twister");

        extensionLeft = hardwareMap.get(Servo.class, "extensionLeft");
        extensionRight = hardwareMap.get(Servo.class, "extensionRight");

        odoMid = hardwareMap.get(Servo.class, "odoMid");

        gPosition = hardwareMap.get(Servo.class, "gPosition");

        gRotation = hardwareMap.get(Servo.class, "gRotation");


        coneDetector = hardwareMap.get(DistanceSensor.class, "coneDetector");

        poleDetector = hardwareMap.get(DistanceSensor.class, "poleDetector");



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

        armRight.setDirection(Servo.Direction.REVERSE);
        armLeft.setDirection(Servo.Direction.REVERSE);

    }

    public void initializeCamera()
    {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline1 = new PowerPlay(opMode.telemetry);
        pipeline2 = new ConeAlignmentBlue(opMode.telemetry);
        pipeline3 = new ConeAlignmentRed(opMode.telemetry);
        pipeline4 = new PowerPlay2(opMode.telemetry);

        webcam.setPipeline(pipeline2);

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

    public Servo getGrabber1()
    {
        return grabber1;
    }

    public Servo getGrabber2()
    {
        return grabber2;
    }

    public Servo getArmLeft()
    {
        return armLeft;
    }

    public Servo getArmRight()
    {
        return armRight;
    }

    public Servo getFourBar() {
        return fourBar;
    }

    public Servo getTwister()
    {
        return twister;
    }



    public Servo getExtensionRight()
    {
        return extensionRight;
    }

    public Servo getExtensionLeft()
    {
            return extensionLeft;
    }

    public Servo getOdoMid()
    {
        return odoMid;
    }

    public Servo getgPosition()
    {
        return gPosition;
    }

    public Servo getgRotation()
    {
        return gRotation;
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

    public DistanceSensor getPoleDetector() {
        return poleDetector;
    }

    public TouchSensor getMagnet()
    {
        return magnet;
    }


    public double getTwisterUpPos()
    {
        return twisterUpPos;
    }

    public double getTwisterDownPos()
    {
        return twisterDownPos;
    }

    public double getGrabber1OpenPos()
    {
        return grabber1OpenPos;
    }

    public double getGrabber1ClosePos()
    {
        return grabber1ClosePos;
    }

    public double getGrabber2OpenPos()
    {
        return grabber2OpenPos;
    }

    public double getGrabber2ClosePos()
    {
        return grabber2ClosePos;
    }

    public double getArmLeftHighPos()
    {
        return armLeftHighPos;
    }

    public double getArmRightHighPos()
    {
        return armRightHighPos;
    }

    public double getArm1LowPos()
    {
        return arm1LowPos;
    }

    public double getArm2LowPos()
    {
        return arm2LowPos;
    }

    public double getBarFrontPos()
    {
        return barFrontPos;
    }

    public double getBarBackPos()
    {
        return barBackPos;
    }

    public double getArm1StackPos5()
    {
        return  arm1PickUpPos;
    }

    public double getArm2StackPos5()
    {
        return arm2PickUpPos;
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

    public double getExtensionLeftIn()
    {
        return extensionLeftIn;
    }

    public double getExtensionRightIn()
    {
        return extensionRightIn;
    }

    public double getExtensionLeftOut()
    {
        return extensionLeftOut;
    }

    public double getExtensionRightOut()
    {
        return extensionRightOut;
    }

    public double getExtensionLeftTransfer()
    {
        return extensionLeftTransfer;
    }

    public double getExtensionRightTransfer()
    {
        return extensionRightTransfer;
    }

    public double getExtensionLeftCyclePos()
    {
        return extensionLeftCyclePos;
    }

    public double getExtensionRightCyclePos()
    {
        return extensionRightCyclePos;
    }

    public double getOdoMidUp()
    {
        return odoMidUp;
    }

    public double getOdoMidDown()
    {
        return odoMidDown;
    }

    public double getgPositionTransfer()
    {
        return gPositionTransfer;
    }

    public double getgRotationTransfer()
    {
        return gRotationTransfer;
    }

    public double getPositionStack1()
    {
            return positionStack1;
    }
    public double getPositionStack2()
    {
        return positionStack2;
    }
    public double getPositionStack3()
    {
        return positionStack3;
    }
    public double getPositionStack4()
    {
        return positionStack4;
    }
    public double getPositionStack5()
    {
        return positionStack5;
    }
    public double getgPositionStore()
    {
        return gPositionStore;
    }

    public double getRotationStack1()
    {
        return rotationStack1;
    }
    public double getRotationStack2()
    {
        return rotationStack2;
    }
    public double getRotationStack3()
    {
        return rotationStack3;
    }
    public double getRotationStack4()
    {
        return rotationStack4;
    }
    public double getRotationStack5()
    {
        return rotationStack5;
    }


    }
