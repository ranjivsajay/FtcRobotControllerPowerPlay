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
//        turnToPID(90);


        moveRight(0.4, 230);
        moveForwardUp(0.75, 0.4, 2150, 980);
        Thread.sleep(200);

        turnPID(120);

//        robot aligns itself with the pole
        while(robot.getPoleDetector().getDistance(DistanceUnit.INCH) >= 25)
        {
            robot.getRightFront().setPower(0.37);
            robot.getRightBack().setPower(0.37);
            robot.getLeftFront().setPower(-0.37);
            robot.getLeftBack().setPower(-0.37);
        }
        stopMotors();

        while(robot.getPoleDetector().getDistance(DistanceUnit.CM) >= 8)
        {
            moveBackward(0.2);
        }
        stopMotors();
        Thread.sleep(50);

        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
        Thread.sleep(50);

        moveForward(0.4, 270);
        turnToPID(92);

        robot.getLineDetector().enableLed(true);
        while(robot.getLineDetector().blue() <= 80)
        {
            moveForward(0.3);
        }
        stopMotors();

        slides(-0.2,-900);





//
//        moveForwardDown(0.5, 0.3, 700, -900, 0.49, 0.51);

//        moveForward(0.25, 400);
//        robot.getGrabber().setPosition(robot.getGrabberClosePos());

//        while(robot.getConeDetector().getDistance(DistanceUnit.CM) > 0.2)
//        {
//            moveForward(0.3);
//        }
//        robot.getGrabber().setPosition(robot.getGrabberClosePos());





    }

}
