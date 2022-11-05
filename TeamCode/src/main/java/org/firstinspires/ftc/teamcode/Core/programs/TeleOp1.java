package org.firstinspires.ftc.teamcode.Core.programs;


import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Core.core.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.core.UpliftTele;
import org.firstinspires.ftc.teamcode.Core.toolkit.UpliftMath;

@TeleOp (name = "teleOp", group = "Opmodes")
public class TeleOp1 extends UpliftTele {

    UpliftRobot robot;

    @Override
    public void initHardware()
    {
        robot = new UpliftRobot(this);
    }

    @Override
    public void initAction()
    {
        robot.getGrabber().setPosition(0.2);

    }

    @Override
    public void bodyLoop() throws InterruptedException {

        telemetry.addData("magnetic sensor", robot.getMagneticSensor().isPressed());
        telemetry.update();

        double leftY =(.6 * Range.clip(-gamepad1.left_stick_y, -1, 1));
        double rightX = (.6 * Range.clip(gamepad1.right_stick_x, -1, 1));
        double leftX = ( .6 * Range.clip(gamepad1.left_stick_x, -1, 1));



        double angle = 90 - Math.toDegrees(UpliftMath.atan2UL(leftY, leftX));
        double magnitude = 0.8 * Range.clip(sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2)), -1, 1);

        teleDrive(angle, magnitude, rightX, gamepad1.right_trigger, robot);

        robot.getSlide1().setPower(0.3 * Range.clip(gamepad2.right_stick_y, -.5, 1));
        robot.getSlide2().setPower(0.3 * Range.clip(-gamepad2.right_stick_y, -.5, 1));


        grab();
        low();
        medium();
        high();
        open();
        cap();



//
//        if(gamepad1.dpad_down)
//        {
//            robot.getLeftFront().setPower(-0.4);
//            robot.getLeftBack().setPower(-0.4);
//            robot.getRightBack().setPower(-0.4);
//            robot.getRightFront().setPower(-0.4);
//        }
//
//        if(gamepad1.dpad_up)
//        {
//            robot.getLeftFront().setPower(0.4);
//            robot.getLeftBack().setPower(0.4);
//            robot.getRightBack().setPower(0.4);
//            robot.getRightFront().setPower(0.4);
//        }
//
//        if(gamepad1.dpad_right)
//        {
//            robot.getLeftFront().setPower(0.4);
//            robot.getLeftBack().setPower(-0.4);
//            robot.getRightBack().setPower(0.4);
//            robot.getRightFront().setPower(-0.4);
//        }
//
//        if(gamepad1.dpad_left)
//        {
//            robot.getLeftFront().setPower(-0.4);
//            robot.getLeftBack().setPower(0.4);
//            robot.getRightBack().setPower(-0.4);
//            robot.getRightFront().setPower(0.4);
//        }
//
    }

    @Override
    public void exit() {

    }

    public static void teleDrive ( double joystickAngle, double speedVal,
                                   double turnVal, float slowModeInput, UpliftRobot robot) {
        double lfPow = sin(toRadians(joystickAngle) + (0.25 * PI)) * speedVal + turnVal;
        double rfPow = sin(toRadians(joystickAngle) - (0.25 * PI)) * speedVal - turnVal;
        double lbPow = sin(toRadians(joystickAngle) - (0.25 * PI)) * speedVal + turnVal;
        double rbPow = sin(toRadians(joystickAngle) + (0.25 * PI)) * speedVal - turnVal;
//
        // find max total input out of the 4 motors
        double maxVal = abs(lfPow);
        if (abs(rfPow) > maxVal) {
            maxVal = abs(rfPow);
        }
        if (abs(lbPow) > maxVal) {
            maxVal = abs(lbPow);
        }
        if (abs(rbPow) > maxVal) {
            maxVal = abs(rbPow);
        }

        if (maxVal < (1 / sqrt(2))) {
            maxVal = 1 / sqrt(2);
        }

        // set the scaled powers
        float speedFactor = 1.0f;
        if (slowModeInput > 0.1f)
            speedFactor = 0.5f;

        robot.getLeftFront().setPower(speedFactor * (lfPow / maxVal));
        robot.getLeftBack().setPower(speedFactor * (lbPow / maxVal));
        robot.getRightBack().setPower(speedFactor * (rbPow / maxVal));
        robot.getRightFront().setPower(speedFactor * (rfPow / maxVal));


//        // set the scaled powers
//        robot.getLeftFront().setPower(lfPow / maxVal);
//        robot.getLeftBack().setPower(lbPow / maxVal);
//        robot.getRightBack().setPower(rbPow / maxVal);
//        robot.getRightFront().setPower(rfPow / maxVal);
    }
//    public void slowMode() throws InterruptedException
//    {
//        if(gamepad1.right_trigger > 0)
//        {
//
//
//
//        }
//    }

    public void slides(double power, double dist) {
        double initialPos1 = robot.getSlide2().getCurrentPosition();

        while (robot.getSlide2().getCurrentPosition() < initialPos1 + dist)
        {
            robot.getSlide1().setPower(-power);
            robot.getSlide2().setPower(power);
        }
        robot.getSlide1().setPower(0);
        robot.getSlide2().setPower(0);


    }

    public void grab() throws InterruptedException {
        if(gamepad2.right_trigger > 0)
        {
            robot.getGrabber().setPosition(0.06);

        }
    }
    public void cap() throws InterruptedException {
        if(gamepad2.left_trigger > 0)
        {
            robot.getGrabber().setPosition(.14);

        }
    }
    public void low() throws InterruptedException{
        if(gamepad2.b){
            slides(0.25,1800);
        }

    }
    public void medium() throws InterruptedException{
        if(gamepad2.y){
            slides(0.25,2700 );
        }

    }
    public void high() throws InterruptedException{
        if(gamepad2.x)
        {
            slides(0.25,3300);
        }
    }

    public void open()
    {
        if(gamepad2.a) {
            robot.getGrabber().setPosition(0.2);
        }
//            if(robot.getMagneticSensor().isPressed()) {
//                robot.getSlide1().setPower(0);
//                robot.getSlide2().setPower(0);
//            }

//            while(robot.getMagneticSensor().isPressed())
//            {
//
//
//            }



        }
    }
