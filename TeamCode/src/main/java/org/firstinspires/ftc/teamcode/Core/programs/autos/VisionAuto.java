package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.core.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.core.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.toolkit.vision.PowerPlay;

@Autonomous(name = "VisionAuto", group = "Opmodes")
public class VisionAuto extends UpliftAutoImpl {



    @Override
    public void initAction(){
        robot.getGrabber().setPosition(0.11);
//        robot.getWebcam().closeCameraDevice();


    }
    @Override
    public void body() throws InterruptedException
    {
        int location = robot.pipeline.location;




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
