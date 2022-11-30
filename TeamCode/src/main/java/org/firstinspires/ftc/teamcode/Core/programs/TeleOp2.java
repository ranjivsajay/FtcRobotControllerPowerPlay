//package org.firstinspires.ftc.teamcode.Core.programs;
//
//import static java.lang.Math.PI;
//import static java.lang.Math.abs;
//import static java.lang.Math.min;
//import static java.lang.Math.sin;
//import static java.lang.Math.sqrt;
//import static java.lang.Math.toRadians;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
//import org.firstinspires.ftc.teamcode.Core.main.UpliftTele;
//import org.firstinspires.ftc.teamcode.Core.toolkit.UpliftMath;
//
//@TeleOp (name = "teleOp2", group = "Opmodes")
//public class TeleOp2 extends UpliftTele {
//
//    UpliftRobot robot;
//
//    double arm1HighPos = .4;
//    double arm2HighPos = .0;
//
//    @Override
//    public void initHardware() {
//        robot = new UpliftRobot(this);
//    }
//
//    @Override
//    public void initAction()
//    {
//        robot.getArm1().setPosition(arm1HighPos);
//    }
//
//    @Override
//    public void bodyLoop() throws InterruptedException {
//        double leftY = (.7 * Range.clip(-gamepad1.left_stick_y, -1, 1));
//        double rightX = (.7 * Range.clip(gamepad1.right_stick_x, -1, 1));
//        double leftX = (.7 * Range.clip(gamepad1.left_stick_x, -1, 1));
//        double angle = 90 - Math.toDegrees(UpliftMath.atan2UL(leftY, leftX));
//        double magnitude = 1.0  * Range.clip(sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2)), -1, 1);
//
//        double magnitude6bar = (.7 * Range.clip(-gamepad2.right_stick_y, -1, 1));
//        double Ltrigger = (gamepad1.left_trigger);
//
//        teleDrive(angle, magnitude, rightX, gamepad1.right_trigger, robot);
//        bottom6bar(magnitude6bar);
////        barhold(Ltrigger);
//        raiseTop6bar();
//        test();
//
//        telemetry.addData("servo1 pos" , robot.getArm1().getPosition());
//        telemetry.addData("slide1 pos" , robot.getSlide1().getCurrentPosition());
//        telemetry.addData("servo2 pos" , robot.getArm2().getPosition());
//        telemetry.addData("slide2 pos" , robot.getSlide2().getCurrentPosition());
//        telemetry.update();
//
//
//    }
//
//
//    @Override
//    public void exit() {
//
//    }
//
//
//
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
//
//    }
//
//    public void bottom6bar(double power)
//    {
//        robot.getSlide1().setPower(-power);
//        robot.getSlide2().setPower(power);
//
//    }
//
//    public void top6bar ()
//    {
//        robot.getArm1().setPosition(arm1HighPos);
//        robot.getArm2().setPosition(arm2HighPos);
//    }
////
////    public void  barhold(double power)
////    {
////        if(power > 0)
////        {
////            robot.getSlide1().setPower(-.8);
////            robot.getSlide2().setPower(.8);
////        }
//    public void raiseTop6bar()
//    {
//        if (gamepad2.y)
//        {
//            robot.getArm1().setPosition(arm1HighPos);
//            robot.getArm2().setPosition(arm2HighPos);
//        }
//    }
//
//    public void test()
//    {
//        if (gamepad2.a)
//        {
//            robot.getGrabber().setPosition(1);
//        }
//    }
//
//
//    }
//
//
//
//
//
//
//
