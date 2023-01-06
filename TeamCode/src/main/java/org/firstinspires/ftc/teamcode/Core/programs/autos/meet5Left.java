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

        moveRight(0.4, 230);
        moveForwardUp(0.75, 0.4, 2150, 1000);
        Thread.sleep(200);

        turnPID(120);

//        robot aligns itself with the pole
        while(robot.getPoleDetector().getDistance(DistanceUnit.INCH) > 25)
        {
            robot.getRightFront().setPower(0.38);
            robot.getRightBack().setPower(0.38);
            robot.getLeftFront().setPower(-0.38);
            robot.getLeftBack().setPower(-0.38);
        }
        stopMotors();

        while(robot.getPoleDetector().getDistance(DistanceUnit.CM) > 8)
        {
//            if(robot.getPoleDetector().getDistance(DistanceUnit.CM))
//
            moveBackward(0.2);
        }

        stopMotors();
        Thread.sleep(50);

        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
        Thread.sleep(100);

        moveForward(0.35, 230 );
        turnToPID(95);
        
        robot.getSlide1().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getSlide2().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.getSlide1().setPower(0.4);
        robot.getSlide2().setPower(-0.4);
        Thread.sleep(750);

//        robot.getLineDetector().enableLed(true);
//        while(robot.getLineDetector().blue() <= 80)
//        {
//            moveForward(0.3);
//        }
//        stopMotors();



        robot.getArm1().setPosition(robot.getArm1StackPos5());
        robot.getArm2().setPosition(robot.getArm2StackPos5());

        fourBarFront();
        robot.getTwister().setPosition(robot.getTwisterDownPos());

        moveForward(0.5, 700);







//        moveForwardDown(0.5, 0.3, 700, -900, robot.getArm1StackPos5(), robot.getArm2StackPos5());

//        moveForward(0.25, 400);
//        robot.getGrabber().setPosition(robot.getGrabberClosePos());

//        while(robot.getConeDetector().getDistance(DistanceUnit.CM) > 0.2)
//        {
//            moveForward(0.3);
//        }
//        robot.getGrabber().setPosition(robot.getGrabberClosePos());





    }

}
