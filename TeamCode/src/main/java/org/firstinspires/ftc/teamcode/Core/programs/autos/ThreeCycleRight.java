package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;

@Autonomous(name = "3CycleRight", group = "Opmodes")
public class ThreeCycleRight extends UpliftAutoImpl {

    @Override
    public void initAction() {


        robot.getGrabber().setPosition(robot.getGrabberClosePos());
        robot.getFourBar1().setPosition(.28);
        robot.getFourBar2().setPosition(.72);

    }

    @Override
    public void body() throws InterruptedException
    {
        moveForwardHigh(0.5, 0.5, 2000, 953);

    }



}
