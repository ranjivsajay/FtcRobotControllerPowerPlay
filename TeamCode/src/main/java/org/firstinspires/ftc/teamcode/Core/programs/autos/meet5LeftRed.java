package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;

@Autonomous(name = "meet5LeftRed", group = "Opmodes")
public class meet5LeftRed extends UpliftAutoImpl
{
    @Override
    public void initAction()
    {

        robot.getTwister().setPosition(robot.getTwisterDownPos());
        robot.getGrabber().setPosition(robot.getGrabberClosePos());

        robot.getFourBar1().setPosition(0.72);
        robot.getFourBar2().setPosition(0.28);


    }

    @Override
    public void body() throws InterruptedException
    {
        int parkLocation = robot.pipeline1.location;
        robot.getWebcam().setPipeline(robot.pipeline3);

        moveRight(0.4, 200);
        moveForwardUp(0.75, 0.4, 2650, 1000);
        Thread.sleep(200);

        moveBackward(0.5, 400);
        Thread.sleep(200);

        turnPID(152);

//        robot aligns itself with the pole
        while(robot.getPoleDetector().getDistance(DistanceUnit.CM) > 37)
        {
            robot.getRightFront().setPower(-0.2);
            robot.getRightBack().setPower(-0.2);
            robot.getLeftFront().setPower(0.2);
            robot.getLeftBack().setPower(0.2);
        }
        stopMotors();



        moveBackward(0.3, 500);

        stopMotors();
        Thread.sleep(50);

        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
        Thread.sleep(1000);

        moveForward(0.35, 350);
        turnToPID(93);

        robot.getSlide1().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getSlide2().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.getSlide1().setPower(0.4);
        robot.getSlide2().setPower(-0.4);
        Thread.sleep(750);

        robot.getWebcam().setPipeline(robot.pipeline1);

//
//        //robot aligns itself with the stack of cones
//        if(robot.pipeline3.leftValue > robot.pipeline3.rightValue)
//        {
//            while(robot.pipeline3.leftValue > robot.pipeline3.rightValue)
//            {
//                moveLeft(0.3);
//            }
//            if(robot.pipeline3.rightValue > robot.pipeline3.leftValue)
//            {
//                while(robot.pipeline3.rightValue > robot.pipeline3.leftValue)
//                {
//                    moveRight(0.3);
//                }
//
//            }
//
//        }
//        else if(robot.pipeline3.rightValue > robot.pipeline3.leftValue)
//        {
//            while(robot.pipeline3.rightValue > robot.pipeline3.leftValue)
//            {
//                moveRight(0.25);
//            }
//            if(robot.pipeline3.leftValue > robot.pipeline3.rightValue)
//            {
//                while(robot.pipeline3.leftValue > robot.pipeline3.rightValue)
//                {
//                    moveLeft(0.3);
//                }
//                if(robot.pipeline3.rightValue > robot.pipeline3.leftValue + 20)
//                {
//                    while(robot.pipeline3.rightValue > robot.pipeline3.leftValue + 20)
//                    {
//                        moveRight(0.3);
//                    }
//
//                }
//
//            }
//
//        }
//        turnToPID(92);
//        stopMotors();
//        robot.getWebcam().setPipeline(robot.pipeline1);
//
//        robot.getArm1().setPosition(robot.getArm1StackPos5());
//        robot.getArm2().setPosition(robot.getArm2StackPos5());
//
//        fourBarFront();
//        robot.getTwister().setPosition(robot.getTwisterDownPos());
//        turnToPID(92);
//        Thread.sleep(500);
//        moveForward(.5, 300);
//        while(robot.getConeDetector().getDistance(DistanceUnit.CM) > 8)
//        {
//            moveForward(0.3);
//        }
//        stopMotors();
//
//
//        robot.getGrabber().setPosition(robot.getGrabberClosePos());
//        Thread.sleep(500);
//        robot.getFourBar1().setPosition( 0.4);
//        robot.getFourBar2().setPosition( 0.6);
//        Thread.sleep(500);
//
//        moveBackwardHigh(0.5,0.5, 600, 1200);
//        turnToPID(135);
//
////        robot aligns itself with the pole
//        while(robot.getPoleDetector().getDistance(DistanceUnit.CM) > 37)
//        {
//            robot.getRightFront().setPower(0.2);
//            robot.getRightBack().setPower(0.2);
//            robot.getLeftFront().setPower(-0.2);
//            robot.getLeftBack().setPower(-0.2);
//        }
//        stopMotors();
//
//
//        moveBackward(0.3, 520);
//        Thread.sleep(200);
//
//        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
//        Thread.sleep(200);
//
//        moveForward(0.35, 450);
//        turnToPID(92);
//
//        robot.getSlide1().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.getSlide2().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        robot.getSlide1().setPower(0.4);
//        robot.getSlide2().setPower(-0.4);
//        Thread.sleep(750);
//        stopMotors();
//
//        turnToPID(92);
//        stopMotors();
//        robot.getWebcam().setPipeline(robot.pipeline1);
//
//        robot.getArm1().setPosition(robot.getArm1StackPos4());
//        robot.getArm2().setPosition(robot.getArm2StackPos4());
//
//        fourBarFront();
//        robot.getTwister().setPosition(robot.getTwisterDownPos());
////        turnToPID(93);
//        Thread.sleep(500);
//
//        moveForward(.5, 300);
//        while(robot.getConeDetector().getDistance(DistanceUnit.CM) > 8)
//        {
//            moveForward(0.3);
//        }
//        stopMotors();
//
//        robot.getGrabber().setPosition(robot.getGrabberClosePos());
//        Thread.sleep(500);
//        robot.getFourBar1().setPosition( 0.4);
//        robot.getFourBar2().setPosition( 0.6);
//        Thread.sleep(500);
//
//        moveBackwardHigh(0.5,0.5, 630, 1200);
//        turnToPID(128.5);
//
////        robot aligns itself with the pole
//        while(robot.getPoleDetector().getDistance(DistanceUnit.CM) > 37)
//        {
//            robot.getRightFront().setPower(0.2);
//            robot.getRightBack().setPower(0.2);
//            robot.getLeftFront().setPower(-0.2);
//            robot.getLeftBack().setPower(-0.2);
//        }
//        stopMotors();
//
//
//        moveBackward(0.3, 520);
//        Thread.sleep(200);
//
//        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
//        Thread.sleep(200);
//
//        moveForward(0.35, 440);
//        turnToPID(92);
//
//        robot.getSlide1().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.getSlide2().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        robot.getSlide1().setPower(0.4);
//        robot.getSlide2().setPower(-0.4);
//        Thread.sleep(750);
//        stopMotors();
//
//        turnToPID(92);
//        stopMotors();
//        robot.getWebcam().setPipeline(robot.pipeline1);
//
//        robot.getArm1().setPosition(robot.getArm1StackPos3());
//        robot.getArm2().setPosition(robot.getArm2StackPos3());
//
//        fourBarFront();
//        robot.getTwister().setPosition(robot.getTwisterDownPos());
////        turnToPID(93);
//        Thread.sleep(500);
//
//        moveForward(.5, 300);
//        while(robot.getConeDetector().getDistance(DistanceUnit.CM) > 8)
//        {
//            moveForward(0.3);
//        }
//        stopMotors();
//
//        robot.getGrabber().setPosition(robot.getGrabberClosePos());
//        Thread.sleep(500);
//        robot.getFourBar1().setPosition( 0.7);
//        robot.getFourBar2().setPosition( 0.3);
//        Thread.sleep(500);






//
//        moveBackwardHigh(0.5,0.5, 700, 1200);
//        turnToPID(120);
//
////        robot aligns itself with the pole
//        while(robot.getPoleDetector().getDistance(DistanceUnit.CM) > 37)
//        {
//            robot.getRightFront().setPower(0.2);
//            robot.getRightBack().setPower(0.2);
//            robot.getLeftFront().setPower(-0.2);
//            robot.getLeftBack().setPower(-0.2);
//        }
//        stopMotors();
//
//
//        moveBackward(0.3, 500);
//        Thread.sleep(200);
//
//        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
//        Thread.sleep(200);
//
//        moveForward(0.35, 440);
//        turnToPID(92);
//
//        robot.getSlide1().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.getSlide2().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        robot.getSlide1().setPower(0.4);
//        robot.getSlide2().setPower(-0.4);
//        Thread.sleep(750);
//        stopMotors();

        if(parkLocation == 1)
        {
            moveForward(0.5, 1100);
        }

        else if(parkLocation == 2)
        {
            moveForward(0.5, 200);
        }

        else if(parkLocation == 3)
        {
            moveBackward(0.5, 900);


        }

    }

}

