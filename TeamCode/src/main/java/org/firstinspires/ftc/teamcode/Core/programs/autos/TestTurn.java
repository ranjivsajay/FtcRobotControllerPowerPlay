package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;

@Autonomous(name = "TestFieldCentric", group = "Opmodes")
public class TestTurn extends UpliftAutoImpl {

    @Override
    public void initAction()
    {

        robot.getArm1().setPosition(robot.getArm1StackPos3());
        robot.getArm2().setPosition(robot.getArm2StackPos3());
        robot.getGrabber().setPosition(robot.getGrabberOpenPos());

        robot.getTwister().setPosition(robot.getTwisterDownPos());
        fourBarFront();


    }
    @Override
    public void body() throws InterruptedException {





    }
}



