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
        robot.getGrabber().setPosition(0.1);
        robot.getSlide1().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.getSlide2().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    }

    @Override
    public void body() throws InterruptedException
    {
        int location = robot.pipeline.location;

        moveBackward(0.3, 50);

        moveLeft(0.5, 2460);

        high();

        moveForward(0.2, 165);
        Thread.sleep(500);

        robot.getGrabber().setPosition(0.25);
        Thread.sleep(250);

        moveBackward(0.2, 175);

        slides(-0.5, -3450);

        moveRight(0.6,310);
        turnRight(.6,161);
        moveForward(.6,505);
        slides(.5,500);
        robot.getGrabber().setPosition(.25);
        moveForward(.4,200);
        Thread.sleep(500);
        robot.getGrabber().setPosition(.08);
        Thread.sleep(500);
        slides(0.5, 500);
        Thread.sleep(500);
        moveBackward(.6,500);
        turnRight(.5,165);
        moveLeft(0.6,400);
        slides(0.5, 2500);

        moveForward(0.2, 170);
        Thread.sleep(500);

        robot.getGrabber().setPosition(0.25);
        Thread.sleep(250);

        moveBackward(0.2, 170);

        slides(-0.5, -3450);



        if(location == 1)
        {
            moveRight(0.5, 450);
            moveBackward(0.5, 500);

        }

        else if(location == 2)
        {

            moveRight(.6,500);

        }

        else if(location == 3)
        {
            moveRight(0.5, 450);
            moveForward(0.5, 800);

        }

    }
}
