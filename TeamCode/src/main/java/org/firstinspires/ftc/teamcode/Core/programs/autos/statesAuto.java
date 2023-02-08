package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;

@Autonomous(name = "statesAuto", group = "Opmodes")
public class statesAuto extends UpliftAutoImpl
{
    @Override
    public void initAction()
    {

        robot.getTwister().setPosition(robot.getTwisterDownPos());
        robot.getGrabber().setPosition(robot.getGrabberClosePos());

        robot.getFourBar1().setPosition(0.4);
        robot.getFourBar2().setPosition(0.6);


    }

    @Override
    public void body() throws InterruptedException
    {
        int parkLocation = robot.pipeline1.location;
        robot.getWebcam().setPipeline(robot.pipeline2);

        moveRight(.6, 230);
        moveForwardUp(.85, 0.4, 2150, 1000);
        Thread.sleep(200);

        turnPID(148.5);


        moveBackward(.6, 500);
        stopMotors();
        Thread.sleep(50);

        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
        Thread.sleep(1000);



//        if(parkLocation == 1)
//        {
//            moveForward(0.2, 100);
//        }
//
//        else if(parkLocation == 2)
//        {
//            moveBackward(.95, 650);
//
//        }
//
//        else if(parkLocation == 3)
//        {
//            moveBackward(1, 1545);
//
//
//        }

    }

}