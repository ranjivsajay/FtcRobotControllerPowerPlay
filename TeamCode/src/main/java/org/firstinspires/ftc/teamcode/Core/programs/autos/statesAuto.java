package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;

@Autonomous(name = "statesAuto", group = "Opmodes")
public class statesAuto extends UpliftAutoImpl
{
    @Override
    public void initAction()
    {

        robot.getTwister().setPosition(robot.getTwisterDownPos());
        robot.getGrabber().setPosition(robot.getGrabberClosePos());

        robot.getFourBar1().setPosition(0.25);
        robot.getFourBar2().setPosition(0.75);


    }

    @Override
    public void body() throws InterruptedException
    {
        int parkLocation = robot.pipeline1.location;
        robot.getWebcam().setPipeline(robot.pipeline2);

        moveRight(.6, 230);
        moveBackward(.6,150);
        Thread.sleep(200);
        moveForwardUp(.8, 0.4, 2250, 950);
        Thread.sleep(200);

//        turnPID(148.5);
        turnPID(150);

        moveForward(.4, 300);

        moveBackward(.4, 950);

        stopMotors();
        Thread.sleep(300);
//        moveForward(.4 , 40);
        robot.getFourBar1().setPosition(.1);
        robot.getFourBar2().setPosition(.9);

        Thread.sleep(150);
//        moveBackward(.4, 100);
        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
        Thread.sleep(500);

        moveForward(0.35, 350);
        robot.getGrabber().setPosition(robot.getGrabberClosePos());

        turnToPID(91);

        robot.getSlide1().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getSlide2().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.getSlide1().setPower(0.3);
        robot.getSlide2().setPower(-0.3);
        Thread.sleep(600);
        moveLeft(.6,50);
//
//        //robot aligns itself with the stack of cones
//        if(robot.pipeline2.leftValue > robot.pipeline2.rightValue)
//        {
//            while(robot.pipeline2.leftValue > robot.pipeline2.rightValue)
//            {
//                moveLeft(0.3);
//            }
//            if(robot.pipeline2.rightValue > robot.pipeline2.leftValue)
//            {
//                while(robot.pipeline2.rightValue > robot.pipeline2.leftValue)
//                {
//                    moveRight(0.3);
//                }
//
//            }
//
//        }
//        else if(robot.pipeline2.rightValue > robot.pipeline2.leftValue)
//        {
//            while(robot.pipeline2.rightValue > robot.pipeline2.leftValue)
//            {
//                moveRight(0.25);
//            }
//            if(robot.pipeline2.leftValue > robot.pipeline2.rightValue)
//            {
//                while(robot.pipeline2.leftValue > robot.pipeline2.rightValue)
//                {
//                    moveLeft(0.3);
//                }
//                if(robot.pipeline2.rightValue > robot.pipeline2.leftValue + 20)
//                {
//                    while(robot.pipeline2.rightValue > robot.pipeline2.leftValue + 20)
//                    {
//                        moveRight(0.3);
//                    }
//
//                }
//
//            }
//
//        }
        turnToPID(91);
        stopMotors();
        robot.getWebcam().setPipeline(robot.pipeline1);

        robot.getArm1().setPosition(robot.getArm1StackPos5());
        robot.getArm2().setPosition(robot.getArm2StackPos5());

        robot.getTwister().setPosition(robot.getTwisterDownPos());
        fourBarFront();

        turnToPID(91);
        Thread.sleep(500);
        moveForward(.5, 500);
        while(robot.getConeDetector().getDistance(DistanceUnit.CM) > 8)
        {
            moveForward(0.3);
        }
        stopMotors();
//
//
        robot.getGrabber().setPosition(robot.getGrabberClosePos());
        Thread.sleep(500);



        robot.getFourBar1().setPosition( 0.4);
        robot.getFourBar2().setPosition( 0.6);
        Thread.sleep(500);

        moveBackwardHigh(0.5,0.5, 800, 1200);
        turnToPID(135);


        moveBackward(0.7, 830);
        Thread.sleep(300);

        robot.getFourBar1().setPosition(.1);
        robot.getFourBar2().setPosition(.9);
        Thread.sleep(300);

        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
        Thread.sleep(200);


        moveForward(0.35, 350);
        turnToPID(93);

        robot.getSlide1().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getSlide2().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.getSlide1().setPower(0.3);
        robot.getSlide2().setPower(-0.3);
        Thread.sleep(600);
//


        turnToPID(92);
        stopMotors();
        robot.getWebcam().setPipeline(robot.pipeline1);

        robot.getArm1().setPosition(robot.getArm1StackPos4());
        robot.getArm2().setPosition(robot.getArm2StackPos4());

        fourBarFront();
        robot.getTwister().setPosition(robot.getTwisterDownPos());

        moveLeft(.6,200);
        turnToPID(91);
        Thread.sleep(500);
        moveForward(.5, 500);
        while(robot.getConeDetector().getDistance(DistanceUnit.CM) > 8)
        {
            moveForward(0.3);
        }
        stopMotors();
//
//
        robot.getGrabber().setPosition(robot.getGrabberClosePos());
        Thread.sleep(500);



        robot.getFourBar1().setPosition( 0.4);
        robot.getFourBar2().setPosition( 0.6);
        Thread.sleep(500);

        moveBackwardHigh(0.5,0.5, 900, 1200);
        turnToPID(135);


        moveBackward(0.7, 830);
        Thread.sleep(300);

        robot.getFourBar1().setPosition(.1);
        robot.getFourBar2().setPosition(.9);
        Thread.sleep(300);

        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
        Thread.sleep(200);


//        if(parkLocation == 1)
//        {
//            moveForward(0.2, 100);
//        }
//
//        else if(parkLocation == 2)
//        {
//            moveBackward(.95, 650);
//
//        }
//
//        else if(parkLocation == 3)
//        {
//            moveBackward(1, 1545);
//
//
//        }

    }

}