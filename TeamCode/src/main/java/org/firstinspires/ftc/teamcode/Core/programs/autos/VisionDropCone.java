package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.core.UpliftAutoImpl;

@Autonomous(name = "VisionDropCone", group = "Opmodes")
public class VisionDropCone extends UpliftAutoImpl
{


    @Override
    public void initAction()
    {
        robot.getGrabber().setPosition(0.1);

    }

    @Override
    public void body() throws InterruptedException
    {
        int location = robot.pipeline.location;

        medium();

        moveBackward(0.2, 100);
        moveLeft(0.25, 100);

        turnLeft(0.2, 90);

        moveBackward(0.2);
        Thread.sleep(1500);

        moveForward(0.35, 1500);

        moveBackward(0.35, 105);

        turnRight(0.2, 90);

        moveForward(0.15);
        Thread.sleep(1000);
        stopMotors();
        Thread.sleep(100);


        robot.getGrabber().setPosition(0.2);
        Thread.sleep(1000);

        moveBackward(0.35, 200);

        moveRight(0.5, 450);

        if(location == 1)
        {
            moveBackward(0.2, 700);

        }
        else if(location == 2)
        {

        }
        else if(location == 3)
        {
            moveForward(0.35, 800);

        }

    }
}
