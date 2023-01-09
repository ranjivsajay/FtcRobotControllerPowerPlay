package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;

@Autonomous(name = "3CycleRight", group = "Opmodes")
public class ThreeCycleRight extends UpliftAutoImpl {

    @Override
    public void initAction()
    {

//        robot.getLeftFront().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getRightFront().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getLeftBack().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.getRightBack().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        robot.getSlide1().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.getSlide2().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.getGrabber().setPosition(robot.getGrabberClosePos());
        robot.getFourBar1().setPosition(.28);
        robot.getFourBar2().setPosition(.72);

    }

    @Override
    public void body() throws InterruptedException
    {

//        moveForwardHigh(0.7, 0.8, 2300, 953);
//
//        robot.getSlide1().setPower(-0.4);
//        robot.getSlide2().setPower(0.4);
//        Thread.sleep(500);
//
//        robot.getRightFront().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.getLeftBack().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        turnRight(0.5, 93);
//        Thread.sleep(400);

//        moveBackwardHigh(0.5, 10000);

//        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
//        Thread.sleep(500);


    }
}
