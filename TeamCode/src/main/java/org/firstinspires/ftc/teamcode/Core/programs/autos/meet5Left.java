package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;

@Autonomous(name = "meet5Left", group = "Opmodes")
public class meet5Left extends UpliftAutoImpl
{
    @Override
    public void initAction()
    {
        robot.getTwister().setPosition(robot.getTwisterDownPos());
        robot.getGrabber().setPosition(robot.getGrabberClosePos());
        robot.getFourBar1().setPosition(.28);
        robot.getFourBar2().setPosition(.72);
    }

    @Override
    public void body() throws InterruptedException
    {
        moveRight(0.5, 150);
        moveForwardUp(0.8, 0.5, 2100, 953);
        Thread.sleep(200);

//        turnPID(140);

        //robot aligns itself with the pole
        while(robot.getPoleDetector().getDistance(DistanceUnit.CM) > 5)
        {
            robot.getRightFront().setPower(0.3);
            robot.getRightBack().setPower(0.3);
            robot.getLeftFront().setPower(-0.3);
            robot.getLeftBack().setPower(-0.3);
        }
        stopMotors();

        while(robot.getPoleDetector().getDistance(DistanceUnit.CM) > 2)
        {
            moveBackward(0.3);
        }
        stopMotors();
        Thread.sleep(50);

//        moveBackward(0.4, 200);
//        Thread.sleep(50);
        robot.getGrabber().setPosition(robot.getGrabberOpenPos());

        moveForward(0.4, 200);
        turnToPID(90);

        robot.getLineDetector().enableLed(true);
        while(robot.getLineDetector().blue() < 20)
        {
            moveForward(0.3);
        }

        moveForwardDown(0.5, 0.3, 700, -900, 0.49, 0.51);

//        moveForward(0.25, 400);
//        robot.getGrabber().setPosition(robot.getGrabberClosePos());

        while(robot.getConeDetector().getDistance(DistanceUnit.CM) > 0.2)
        {
            moveForward(0.3);
        }
        robot.getGrabber().setPosition(robot.getGrabberClosePos());





    }

}
