package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.core.UpliftAutoImpl;

@Autonomous(name = "VisionDropCone", group = "Opmodes")
public class VisionDropCone extends UpliftAutoImpl
{

    @Override
    public void initAction()
    {
        robot.getGrabber().setPosition(0.11);

    }

    public void body() throws InterruptedException
    {
        int location = robot.pipeline.location;

        if(location == 1)
        {
            robot.getWebcam().stopStreaming();

            moveForward(0.5, 500);
            moveLeft(0.5, 500);

            turnLeft(0.2, 20);
            high();

            turnRight(0.2, 20);
            moveBackward(0.5, 1000);

        }
        else if(location == 2)
        {
            robot.getWebcam().stopStreaming();

            moveForward(0.5, 500);
            moveLeft(0.5, 500);

            turnLeft(0.2, 20);
            high();

            turnRight(0.2, 20);
            moveBackward(0.5, 600);
        }
        else if(location == 3)
        {
            robot.getWebcam().stopStreaming();

            moveForward(0.5, 500);
            moveLeft(0.5, 500);

            turnLeft(0.2, 20);
            high();

            turnRight(0.2, 20);

        }

    }
}
