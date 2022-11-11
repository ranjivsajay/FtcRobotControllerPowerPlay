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

        moveLeft(0.5, 2560);

        high();

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
            moveRight(0.5,450);
        }

        else if(location == 3)
        {
            moveRight(0.5, 450);
            moveForward(0.5, 700);

        }





//        medium();
//
//        moveBackward(0.2, 100);
//        moveLeft(0.25, 100);
//
//        turnLeft(0.2, 90);
//
//        moveBackward(0.25);
//        Thread.sleep(1500);
//
//        moveForward(0.25, 1500);
//
//        moveBackward(0.25, 105);
//
//        turnRight(0.2, 85);
//
//        moveForward(0.15);
//        Thread.sleep(500);
//        stopMotors();
//        Thread.sleep(100);
//
//
//        robot.getGrabber().setPosition(0.2);
//        Thread.sleep(1000);
//
//        moveBackward(0.35, 200);
//
//        moveRight(0.5, 450);

//        if(location == 1)
//        {
//            moveBackward(0.2, 700);
//
//        }
//        else if(location == 2)
//        {
//            moveForward(.2,100);
//        }
//        else if(location == 3)
//        {
//            moveForward(0.35, 800);
//
//        }

    }
}
