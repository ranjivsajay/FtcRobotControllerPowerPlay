package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;

@Autonomous(name = "VisionAuto", group = "Opmodes")
public class VisionAuto extends UpliftAutoImpl {



    @Override
    public void initAction(){
        robot.getGrabber1().setPosition(robot.getGrabber1ClosePos());
//        robot.getWebcam().closeCameraDevice();
    }

    @Override
    public void body() throws InterruptedException
    {
        int location = robot.pipeline1.location;


        if(location == 1)
        {
            moveLeft(0.5, 1200);
            moveForward(0.5, 1500);
        }
        else if(location == 2)
        {
            moveForward(0.5, 1500);
        }
        else if(location == 3)
        {

            moveRight(0.5, 1200);
            moveForward(0.5, 1500);



        }


    }

}
