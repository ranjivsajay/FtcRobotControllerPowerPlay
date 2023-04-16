package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;

@Autonomous(name = "test", group = "Opmodes")
public class test extends UpliftAutoImpl
{
    @Override
    public void initAction()
    {

//        robot.getTwister().setPosition(robot.getTwisterDownPos());
//        robot.getGrabber().setPosition(robot.getGrabberClosePos());
//
////        robot.getFourBar1().setPosition(0.25);
////        robot.getFourBar2().setPosition(0.075);
//
//        robot.getArm1().setPosition(robot.getArm1StackPos5());
//        robot.getArm2().setPosition(robot.getArm2StackPos5());
//
//        fourBarFront();
        robot.getFourBar().setPosition(robot.getBarFrontPos());
        robot.getArmLeft().setPosition(robot.getArm1StackPos5());
        robot.getArmRight().setPosition(robot.getArm2StackPos5());
        robot.getGrabber1().setPosition(robot.getGrabber1OpenPos());
        robot.getTwister().setPosition(robot.getTwisterDownPos());
        robot.getWebcam().setPipeline(robot.pipeline2);

    }

    @Override
    public void body() throws InterruptedException
    {
        telemetry.addData("power", robot.getLeftFront().getPower());
        telemetry.update();

        while(opModeIsActive() && robot.pipeline2.getError() != 0)
        {
            double var = robot.pipeline2.getError() * 0.0042;
            moveRight(-var);
            while(robot.getLeftFront().isBusy())
            {
                if (Math.abs(robot.getLeftFront().getPower() ) < .2)
                    stopMotors();
            }

        }
        stopMotors();


        if(robot.pipeline2.getError() == 0)
        {
            while (robot.getConeDetector().getDistance(DistanceUnit.CM) > 8)
            {
                moveForward(0.25);
            }

            stopMotors();
        }
        else
        {
            while(opModeIsActive() && robot.pipeline2.getError() != 0)
            {
                double var = robot.pipeline2.getError() * 0.0042;
                moveRight(-var);
                while(robot.getLeftFront().isBusy())
                {
                    if (Math.abs(robot.getLeftFront().getPower() ) < .2)
                        stopMotors();
                }
            }
            stopMotors();
        }



    }

}
