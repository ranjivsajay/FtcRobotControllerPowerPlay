//package org.firstinspires.ftc.teamcode.Core.programs.autos;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
//
//@Autonomous(name = "sahilTest", group = "Opmodes")
//public class sahilTest extends UpliftAutoImpl
//{
//    @Override
//    public void initAction()
//    {
//
//        robot.getTwister().setPosition(robot.getTwisterDownPos());
//        robot.getGrabber().setPosition(robot.getGrabberClosePos());
//
//        robot.getFourBar1().setPosition(.28);
//        robot.getFourBar2().setPosition(.72);
//
//
//    }
//
//    @Override
//    public void body() throws InterruptedException
//    {
//        int parkLocation = robot.pipeline1.location;
//        robot.getWebcam().setPipeline(robot.pipeline2);
//
//        moveRight(0.4, 230);
//        moveForward(.75, 2150);
//        Thread.sleep(200);
//
//        turnPID(150);
//
////        robot aligns itself with the pole
//        while(robot.getPoleDetector().getDistance(DistanceUnit.CM) > 37)
//        {
//            robot.getRightFront().setPower(-0.2);
//            robot.getRightBack().setPower(-0.2);
//            robot.getLeftFront().setPower(0.2);
//            robot.getLeftBack().setPower(0.2);
//        }
//        stopMotors();
//
//
//
////        moveBackward(0.3, 600);
////
////        stopMotors();
////        Thread.sleep(50);
////
////        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
////        Thread.sleep(1000);
////
////        moveForward(0.35, 350);
////        turnToPID(92);
////
////        robot.getSlide1().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////        robot.getSlide2().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////
////        robot.getSlide1().setPower(0.4);
////        robot.getSlide2().setPower(-0.4);
////        Thread.sleep(750);
////
////        //robot aligns itself with the stack of cones
////        if(robot.pipeline2.leftValue > robot.pipeline2.rightValue)
////        {
////            while(robot.pipeline2.leftValue > robot.pipeline2.rightValue)
////            {
////                moveLeft(0.3);
////            }
////            if(robot.pipeline2.rightValue > robot.pipeline2.leftValue)
////            {
////                while(robot.pipeline2.rightValue > robot.pipeline2.leftValue)
////                {
////                    moveRight(0.3);
////                }
////
////            }
////
////        }
////        else if(robot.pipeline2.rightValue > robot.pipeline2.leftValue)
////        {
////            while(robot.pipeline2.rightValue > robot.pipeline2.leftValue)
////            {
////                moveRight(0.25);
////            }
////            if(robot.pipeline2.leftValue > robot.pipeline2.rightValue)
////            {
////                while(robot.pipeline2.leftValue > robot.pipeline2.rightValue)
////                {
////                    moveLeft(0.3);
////                }
////                if(robot.pipeline2.rightValue > robot.pipeline2.leftValue + 20)
////                {
////                    while(robot.pipeline2.rightValue > robot.pipeline2.leftValue + 20)
////                    {
////                        moveRight(0.3);
////                    }
////
////                }
////
////            }
////
////        }
////        stopMotors();
////        robot.getWebcam().setPipeline(robot.pipeline1);
////
////        robot.getArm1().setPosition(robot.getArm1StackPos5());
////        robot.getArm2().setPosition(robot.getArm2StackPos5());
////
////        fourBarFront();
////        robot.getTwister().setPosition(robot.getTwisterDownPos());
////
////        Thread.sleep(500);
////        while(robot.getConeDetector().getDistance(DistanceUnit.CM) > 6)
////        {
////            moveForward(0.1);
////        }
////        stopMotors();
////
////
////        robot.getGrabber().setPosition(robot.getGrabberClosePos());
////        Thread.sleep(500);
//
////here
////        moveBackwardHigh(0.5,0.5, 700, 1250);
////
////        while(robot.getPoleDetector().getDistance(DistanceUnit.CM) > 45)
////        {
////            robot.getRightFront().setPower(0.2);
////            robot.getRightBack().setPower(0.2);
////            robot.getLeftFront().setPower(-0.2);
////            robot.getLeftBack().setPower(-0.2);
////        }
////
////        stopMotors();
//
////to here
//
//
////        moveBackward(0.35, 500);
////        stopMotors();
////        Thread.sleep(100);
////
////        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
////        Thread.sleep(100);
////
////        moveForward(0.35, 490);
////        turnToPID(92);
////
////        robot.getSlide1().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////        robot.getSlide2().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////
////        robot.getSlide1().setPower(0.4);
////        robot.getSlide2().setPower(-0.4);
////        Thread.sleep(750);
////        stopMotors();
////
////        //moves forward till it senses line 2nd time
////        robot.getLineDetector().enableLed(true);
////        while(robot.getLineDetector().blue() <= 55)
////        {
////            moveForward(0.5);
////        }
////
////        stopMotors();
////
////        robot.getArm1().setPosition(robot.getArm1StackPos5());
////        robot.getArm2().setPosition(robot.getArm2StackPos5());
////
////        fourBarFront();
////        robot.getTwister().setPosition(robot.getTwisterDownPos());
////
////
////
////        moveForward(0.2, 230);
////        Thread.sleep(500);
////
////
////        robot.getGrabber().setPosition(robot.getGrabberClosePos());
////        Thread.sleep(500);
////
////
////        moveBackwardHigh(0.5,0.5, 700, 1250);
////
////        while(robot.getPoleDetector().getDistance(DistanceUnit.CM) > 45)
////        {
////            robot.getRightFront().setPower(0.2);
////            robot.getRightBack().setPower(0.2);
////            robot.getLeftFront().setPower(-0.2);
////            robot.getLeftBack().setPower(-0.2);
////        }
////
////        stopMotors();
////
////
////        moveBackward(0.35, 500);
////        stopMotors();
////        Thread.sleep(100);
////
////        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
////        Thread.sleep(100);
////
////        moveForward(0.35, 490);
////        turnToPID(92);
////
////        robot.getSlide1().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////        robot.getSlide2().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////
////        robot.getSlide1().setPower(0.4);
////        robot.getSlide2().setPower(-0.4);
////        Thread.sleep(750);
////
////        //moves forward till it senses line 3rd time
////        robot.getLineDetector().enableLed(true);
////        while(robot.getLineDetector().blue() <= 55)
////        {
////            moveForward(0.5);
////        }
////
////        stopMotors();
////
////        robot.getArm1().setPosition(robot.getArm1StackPos5());
////        robot.getArm2().setPosition(robot.getArm2StackPos5());
////
////        fourBarFront();
////        robot.getTwister().setPosition(robot.getTwisterDownPos());
////
////
////
////        moveForward(0.2, 230);
////        Thread.sleep(500);
////
////
////        robot.getGrabber().setPosition(robot.getGrabberClosePos());
////        Thread.sleep(500);
////
////
////        moveBackwardHigh(0.5,0.5, 700, 1250);
////
////        while(robot.getPoleDetector().getDistance(DistanceUnit.CM) > 45)
////        {
////            robot.getRightFront().setPower(0.2);
////            robot.getRightBack().setPower(0.2);
////            robot.getLeftFront().setPower(-0.2);
////            robot.getLeftBack().setPower(-0.2);
////        }
////
////        stopMotors();
////
////
////        moveBackward(0.35, 500);
////        stopMotors();
////        Thread.sleep(100);
////
////        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
////        Thread.sleep(100);
////
////        moveForward(0.35, 490);
////        turnToPID(92);
////
////        robot.getSlide1().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////        robot.getSlide2().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////
////        robot.getSlide1().setPower(0.4);
////        robot.getSlide2().setPower(-0.4);
////        Thread.sleep(750);
////
////
////
////
////        if(parkLocation == 1)
////        {
////            moveForward(1, 1050);
////
////        }
////
////        else if(parkLocation == 2)
////        {
////
////        }
////
////        else if(parkLocation == 3)
////        {
////            moveBackward(.95, 650);
////
////
////        }
//
//    }
//
//}
