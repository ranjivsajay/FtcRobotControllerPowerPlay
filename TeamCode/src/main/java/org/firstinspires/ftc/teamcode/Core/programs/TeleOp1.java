package org.firstinspires.ftc.teamcode.Core.programs;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.message.redux.StopOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.Threads.DriveThread;
import org.firstinspires.ftc.teamcode.Core.Threads.OperatorThread;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.main.UpliftTele;
import org.firstinspires.ftc.teamcode.Core.toolkit.UpliftMath;

@TeleOp (name = "teleOp", group = "Opmodes")
public class TeleOp1 extends UpliftTele {

    UpliftRobot robot;

    DriveThread driverThread;
    OperatorThread operatorThread;
//    boolean grabberState = true;
//    boolean blockGrabberInput = false;
//    boolean threadOn = false;
//

    double arm1HighPos = .4;
    double arm2HighPos = .0;

    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);

        createDriveThread(robot);
        createOperatorThread(robot);

    }

    @Override
    public void initAction() {





        robot.getFourBar().setPosition(.75);
        robot.getArmLeft().setPosition(robot.getArm1LowPos());
        robot.getArmRight().setPosition(robot.getArm2LowPos());

        robot.getGrabber1().setPosition(robot.getGrabber1ClosePos());
        robot.getTwister().setPosition(robot.getTwisterDownPos());

        //testing grabber 2 pos
//        robot.getgPosition().setPosition(robot.getPostitionStack1());
//        sleep(2000);
//        robot.getgPosition().setPosition(robot.getPostitionStack2());
//        sleep(2000);
//        robot.getgPosition().setPosition(robot.getPostitionStack3());
//        sleep(2000);
//        robot.getgPosition().setPosition(robot.getPostitionStack4());
//        sleep(2000);
//        robot.getgPosition().setPosition(robot.getPostitionStack5());
//        sleep(2000);
//        robot.getgPosition().setPosition(robot.getgPositionStore());

        //testing extension
//          robot.getExtensionRight().setPosition(robot.getExtensionRightIn());
//          robot.getExtensionLeft().setPosition(robot.getExtensionLeftIn());
//          sleep(2000);
//          robot.getExtensionRight().setPosition(.3);
//          robot.getExtensionLeft().setPosition(.7);

//        robot.getOdoMid().setPosition(robot.getOdoMidDown());
//        sleep(2000);
//        robot.getOdoMid().setPosition(robot.getOdoMidUp());



        driverThread.start();
        operatorThread.start();

    }

    @Override

    public void bodyLoop() throws InterruptedException {

        telemetry.addData("dist ", robot.getConeDetector().getDistance(DistanceUnit.METER));
       // telemetry.addData("magnet:" , robot.getMagnet().getValue());
        telemetry.update();

//        test();


//        sleep(1000);
//        sleep(1000);


//        double leftY = (.7 * Range.clip(-gamepad1.left_stick_y, -1, 1));
//        double rightX = (.7 * Range.clip(gamepad1.right_stick_x, -1, 1));
//        double leftX = (.7 * Range.clip(gamepad1.left_stick_x, -1, 1));
//
//
//        double angle = 90 - Math.toDegrees(UpliftMath.atan2UL(leftY, leftX));
//        double magnitude = 0.8 * Range.clip(sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2)), -1, 1);

//        teleDrive(angle, magnitude, rightX, gamepad1.right_trigger, robot);
//
//        robot.getSlide1().setPower(Range.clip(gamepad2.right_stick_y, -1, 1));
//        robot.getSlide2().setPower(Range.clip(-gamepad2.right_stick_y, -1, 1));


//        grab();

//        holdSlidePos();

//        armHigh();

//        if(gamepad2.dpad_up)
//        {
//            thread1.start();
//            threadOn = true;
//        }
//        else
//        {
//            threadOn = false;
//
//        }


//        double angle = 90 - Math.toDegrees(UpliftMath.atan2UL(leftY, leftX));
//        double magnitude = 0.8 * Range.clip(sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2)), -1, 1);

//        teleDrive(angle, magnitude, rightX, gamepad1.right_trigger, robot);
//
//        robot.getSlide1().setPower(Range.clip(gamepad2.right_stick_y, -1, 1));
//        robot.getSlide2().setPower(Range.clip(-gamepad2.right_stick_y, -1, 1));


//        grab();

//        holdSlidePos();
//
//        armHigh();


    }

    @Override
    public void exit()
    {
        operatorThread.end();
        driverThread.end();

    }

    public void createDriveThread(UpliftRobot robot1)
    {

        driverThread = new DriveThread(robot1);
        telemetry.addData("Driver Thread started", driverThread.toString());

    }

    public void createOperatorThread(UpliftRobot robot1) {

        operatorThread = new OperatorThread(robot1);
        telemetry.addData("Operator Thread started", operatorThread.toString());

    }


//    public static void teleDrive(double joystickAngle, double speedVal,
//                                 double turnVal, float slowModeInput, UpliftRobot robot) {
//
//        double lfPow = sin(toRadians(joystickAngle) + (0.25 * PI)) * speedVal + turnVal;
//        double rfPow = sin(toRadians(joystickAngle) - (0.25 * PI)) * speedVal - turnVal;
//        double lbPow = sin(toRadians(joystickAngle) - (0.25 * PI)) * speedVal + turnVal;
//        double rbPow = sin(toRadians(joystickAngle) + (0.25 * PI)) * speedVal - turnVal;
//
//        // find max total input out of the 4 motors
//        double maxVal = abs(lfPow);
//        if (abs(rfPow) > maxVal) {
//            maxVal = abs(rfPow);
//        }
//        if (abs(lbPow) > maxVal) {
//            maxVal = abs(lbPow);
//        }
//        if (abs(rbPow) > maxVal) {
//            maxVal = abs(rbPow);
//        }
//
//        if (maxVal < (1 / sqrt(2))) {
//            maxVal = 1 / sqrt(2);
//        }
//
//        // set the scaled powers
//        float speedFactor = 1.0f;
//        if (slowModeInput > 0.1f)
//            speedFactor = 0.5f;
//
//        robot.getLeftFront().setPower(speedFactor * (lfPow / maxVal));
//        robot.getLeftBack().setPower(speedFactor * (lbPow / maxVal));
//        robot.getRightBack().setPower(speedFactor * (rbPow / maxVal));
//        robot.getRightFront().setPower(speedFactor * (rfPow / maxVal));

//    }

//    public void slidesPower(double power) {
//        robot.getSlide1().setPower(-power);
//        robot.getSlide2().setPower(power);
//    }


//    public void slidesDist(double power, int dist)
//    {
//        robot.getSlide1().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.getSlide2().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        robot.getSlide1().setTargetPosition(-dist);
//        robot.getSlide2().setTargetPosition(dist);
//
//        slidesPower(power);
//
//        robot.getSlide1().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.getSlide2().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        while(robot.getSlide1().isBusy() && robot.getSlide2().isBusy())
//        {
//            telemetry.addData("status", "arm moving");
//            telemetry.update();
//        }
//
//        robot.getSlide1().setPower(0);
//        robot.getSlide2().setPower(0);
//
//
//    }
}

//    public void grab() throws InterruptedException
//    {
//    public void grab() throws InterruptedException {
//        if(gamepad2.right_trigger > robot.getGrabberClosePos() && !blockGrabberInput)
//        {
//            robot.getGrabber().setPosition(grabberState ? robot.getGrabberClosePos() : robot.getGrabberOpenPos());
//            grabberState = !grabberState;
//            blockGrabberInput = true;
//        }
//        else if (gamepad2.right_trigger < robot.getGrabberOpenPos() && blockGrabberInput)
//        {
//            blockGrabberInput = false;
//        }
//    }

//    public void armHigh()
//    {
//        if(gamepad2.dpad_up)
//        {
//
//            robot.getArm2().setPosition(arm2HighPos);
//            robot.getArm1().setPosition(arm1HighPos);
//
//            slidesDist(0.5, 953);
//
//
//        }
//    }
//
//    public void holdSlidePos()
//    {
//        if(gamepad2.left_trigger > 0)
//        {
//            robot.getSlide1().setPower(-0.15);
//            robot.getSlide2().setPower(0.15);
//        }
//    }

//    Thread thread1 = new Thread()
//    {
//        public void run()
//        {
////            armHigh();
//        }
//    };
//}



