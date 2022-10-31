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

    }
    public void body() throws InterruptedException
    {
        int location = robot.pipeline.location;


        if(location == 1)
        {
            robot.getWebcam().stopRecordingPipeline();
            robot.getWebcam().stopStreaming();
            moveBackward(0.5, 750);
            moveLeft(0.5, 1200);
        }
        else if(location == 2)
        {
            robot.getWebcam().stopStreaming();

            robot.getWebcam().stopRecordingPipeline();
            moveLeft(0.5, 1200);
        }
        else if(location == 3)
        {
            robot.getWebcam().stopStreaming();

            robot.getWebcam().stopRecordingPipeline();
            moveForward(0.5, 750);
            moveLeft(0.5, 1200);

        }


    }

}
