package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;

@Autonomous(name = "VisionDropCone", group = "Opmodes")
public class VisionDropCone extends UpliftAutoImpl
{


    @Override
    public void initAction()
    {
        robot.getGrabber().setPosition(robot.getGrabberClosePos());




    }

    @Override
    public void body() throws InterruptedException
    {
        int location = robot.pipeline.location;

        moveBackward(0.5, 100);

        moveLeft(0.6, 3500);

        high();

        moveForward(0.3, 240);
        Thread.sleep(500);

        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
        Thread.sleep(250);

        moveBackward(0.3, 175);

        slides(-0.6, -3450);

        moveRight(0.6,600);

        turnRight(.6,162);

        moveForward(.6,980);

        slides(.6,550);

        robot.getGrabber().setPosition(robot.getGrabberOpenPos());

        moveForward(.4,200);
        Thread.sleep(500);

        robot.getGrabber().setPosition(robot.getGrabberClosePos());
        Thread.sleep(500);

        slides(0.6, 500);

        Thread.sleep(500);

        moveBackward(.6,950);

        turnRight(.5,173);

        moveLeft(0.5,600);

        slides(0.6, 2600);

        moveForward(0.3, 150);
        Thread.sleep(500);

        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
        Thread.sleep(250);

        moveBackward(0.3, 170);

        slides(-0.6, -3450);

        moveRight(0.6,500);

        if(location == 1)
        {
            moveBackward(0.5, 950);

        }

        else if(location == 2)
        {

        }

        else if(location == 3)
        {
            moveForward(0.5, 1100);

        }
    }
}
