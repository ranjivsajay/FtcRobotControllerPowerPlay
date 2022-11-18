package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.core.UpliftAutoImpl;

@Autonomous(name = "VisionDropCone", group = "Opmodes")
public class VisionDropCone extends UpliftAutoImpl
{


    @Override
    public void initAction()
    {
        robot.getGrabber().setPosition(robot.getGrabberClosePos());
        robot.getSlide1().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.getSlide2().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    }

    @Override
    public void body() throws InterruptedException
    {
        int location = robot.pipeline.location;

        moveBackward(0.5, 100);

        moveLeft(0.6, 3400);

        high();

        moveForward(0.3, 165);
        Thread.sleep(500);

        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
        Thread.sleep(250);

        moveBackward(0.3, 175);

        slides(-0.6, -3450);

        moveRight(0.6,500);
        turnRight(.6,167);
        moveForward(.6,950);
        slides(.6,400);
        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
        moveForward(.6,200);
        Thread.sleep(500);
        robot.getGrabber().setPosition(robot.getGrabberClosePos());
        Thread.sleep(500);
        slides(0.6, 500);
        Thread.sleep(500);
        moveBackward(.6,950);
        turnRight(.5,171);
        moveLeft(0.7,570);
        slides(0.6, 2600);

        moveForward(0.4, 150);
        Thread.sleep(500);

        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
        Thread.sleep(250);

        moveBackward(0.4, 170);

        slides(-0.6, -3450);

//        moveRight(0.6,400);
//        turnRight(.6,166);
//        moveForward(.6,805);
//        slides(.6,400);
//        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
//        moveForward(.4,200);
//        Thread.sleep(500);
//        robot.getGrabber().setPosition(robot.getGrabberClosePos());
//        Thread.sleep(500);
//        slides(0.6, 500);
//        Thread.sleep(500);
//        moveBackward(.6,450);



        if(location == 1)
        {
//            moveRight(0.5, 450);
            moveBackward(0.5, 1);

        }

        else if(location == 2)
        {

//            moveRight(.6,500);
            moveBackward(.5,600);
        }

        else if(location == 3)
        {
//            moveRight(0.5, 450);
            moveBackward(0.5, 1500);

          }

    }
}
