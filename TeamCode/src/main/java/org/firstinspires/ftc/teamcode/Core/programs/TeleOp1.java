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
    public void initHardware() {
        robot = new UpliftRobot(this);
    }

    @Override
    public void initAction(){
//        robot.getGrabber().setPosition(0);
//        robot.initializeCamera();


    }

    @Override
    public void bodyLoop() throws InterruptedException {

        double leftY = Range.clip(-gamepad2.left_stick_y, -1, 1);
        double rightX = Range.clip(gamepad2.right_stick_x, -1, 1);
        double leftX = Range.clip(gamepad2.left_stick_x, -1, 1);


//
        double angle = 90 - Math.toDegrees(UpliftMath.atan2UL(leftY, leftX));
        double magnitude = 0.8 * Range.clip(sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2)), -1, 1);

        teleDrive(angle, magnitude, rightX, robot);

//        slides(0.5, 200);
        robot.getSlide1().setPower(Range.clip(gamepad2.right_stick_y, -1, 1));
        robot.getSlide2().setPower(Range.clip(-gamepad2.right_stick_y, -1, 1));

        grab();

    }

    @Override
    public void exit() {

    }

    public static void teleDrive ( double joystickAngle, double speedVal,
                                   double turnVal, UpliftRobot robot) {
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
        robot.getLeftFront().setPower(lfPow / maxVal);
        robot.getLeftBack().setPower(lbPow / maxVal);
        robot.getRightBack().setPower(rbPow / maxVal);
        robot.getRightFront().setPower(rfPow / maxVal);
    }

    public void slides(double power, double dist) {
//        double initialPos = robot.getSlides().getCurrentPosition();
//
//        while (robot.getSlides().getCurrentPosition() < initialPos + dist) {
//            robot.getSlides().setPower(power);
//        }
//        robot.getSlides().setPower(0);

    }

    public void grab()
    {
        if(gamepad2.a)
        {
            robot.getGrabber().setPower(.2);
            
        }
    }



}
