package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.Threads.DriveAuto;
import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;

@Autonomous(name = "3CycleRight", group = "Opmodes")
public class ThreeCycleRight extends UpliftAutoImpl {

    DriveAuto driveAuto = new DriveAuto(robot);
    @Override
    public void initAction() {
        robot.getGrabber().setPosition(robot.getGrabberClosePos());
        robot.getFourBar1().setPosition(.28);
        robot.getFourBar2().setPosition(.72);

    }

    @Override
    public void body() throws InterruptedException
    {



//        moveRight(0.5, 200);

        driveAuto.start();



        driveAuto.end();
//        moveForward(0.5, 2500);

    }
}
