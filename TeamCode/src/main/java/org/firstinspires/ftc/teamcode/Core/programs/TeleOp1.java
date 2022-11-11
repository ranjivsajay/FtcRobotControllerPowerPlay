package org.firstinspires.ftc.teamcode.Core.programs;


import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Core.core.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.core.UpliftTele;
import org.firstinspires.ftc.teamcode.Core.toolkit.UpliftMath;

@TeleOp (name = "teleOp", group = "Opmodes")
public class TeleOp1 extends UpliftTele {

    UpliftRobot robot;
    boolean grabberState = true;
    boolean blockGrabberInput = false;
    private double grabberOpenPosition = 0.25;

    @Override
    public void initHardware()
    {
        robot = new UpliftRobot(this);
    }

    @Override
    public void initAction()
    {
        robot.getGrabber().setPosition(grabberOpenPosition);
        robot.getSlide1().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.getSlide2().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

        robot.getSlide1().setPower(
                (1.0 - 0.8 * gamepad2.left_trigger) * Range.clip(gamepad2.right_stick_y, -1, 1));
        robot.getSlide2().setPower(
                (1.0 - 0.8 * gamepad2.left_trigger) * Range.clip(-gamepad2.right_stick_y, -1, 1));




        grab();
    //`     low();
//        medium();
//        high();
        cap();

        if(gamepad2.dpad_left)
        {
            robot.getSlide1().setPower(.1);
        }

        if(gamepad2.dpad_up)
        {
            robot.getSlide1().setPower(-.1);
        }
        if(gamepad2.dpad_down)
        {
            robot.getSlide2().setPower(-.1);
        }

        if(gamepad2.dpad_right)
        {
            robot.getSlide2().setPower(.1);
        }

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
    }



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
        if(gamepad2.right_trigger > 0.1 && !blockGrabberInput)   
        {
            robot.getGrabber().setPosition(grabberState ? .1 : grabberOpenPosition);
            grabberState = !grabberState;
            blockGrabberInput = true;
        }
        else if (gamepad2.right_trigger < 0.1 && blockGrabberInput)
        {
            blockGrabberInput = false;
        }
    }
    public void cap() throws InterruptedException {
        if(gamepad2.left_trigger > 0)
        {
            robot.getGrabber().setPosition(.06);

        }
    }

    }
