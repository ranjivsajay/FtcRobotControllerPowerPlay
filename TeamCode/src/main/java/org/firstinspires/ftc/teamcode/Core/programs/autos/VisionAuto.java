package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.core.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.core.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.toolkit.vision.PowerPlay;

@Autonomous(name = "VisionAuto", group = "Opmodes")
public class VisionAuto extends UpliftAutoImpl {

    int location;

    @Override
    public void initAction(){
        robot.getGrabber().setPosition(0.11);
        location = robot.pipeline.location;
        robot.getWebcam().closeCameraDevice();


    }
    public void body() throws InterruptedException
    {
        robot.getWebcam().closeCameraDevice();



        if(location == 1)
        {

            moveBackward(0.5, 750);
            moveLeft(0.5, 1200);
        }
        else if(location == 2)
        {
            moveLeft(0.5, 1200);
        }
        else if(location == 3)
        {

            moveForward(0.5, 750);
            moveLeft(0.5, 1200);

        }


    }

}
