package org.firstinspires.ftc.teamcode.Core.Threads;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;

public class OperatorThread extends Thread
{


    private UpliftRobot robot;

    private static final String OPERATOR_NAME = "OperatorThreadName";

    private boolean shutDown = false;

    private boolean grabberState;
    private boolean blockGrabberInput;
    private boolean twisterState;
    private boolean blockTwisterInput;



    public OperatorThread(UpliftRobot robot)
    {
        this.robot = robot;

        this.grabberState = true;
        this.blockGrabberInput = false;

    }


    @Override
    public void run()
    {
        while(!shutDown)
        {
            try
            {

                servoTest();

                grab();

                if(robot.opMode.gamepad2.dpad_up)
                {
                    robot.getFourBar1().setPosition(robot.getBar1FrontPos());
                    robot.getFourBar2().setPosition(robot.getBar2FrontPos());

                }

//                sixBarHigh();
//
//                sixBarMedium();
//
//                sixBarLow();
//
//                sixBarDown();

                holdSlidePos();


                robot.getSlide1().setPower(0.75 * Range.clip(robot.opMode.gamepad2.right_stick_y, -1, 1));
                robot.getSlide2().setPower(0.75 * Range.clip(-robot.opMode.gamepad2.right_stick_y, -1, 1));

//                telemetry.addData("4bar1" , robot.getFourBar1().getPosition());
//                telemetry.addData("4bar2" , robot.getFourBar2().getPosition());


                // todo: validate user responsiveness and set sleep
                sleep(50);
            } catch (Exception e) {
                e.printStackTrace();

//                StringWriter sw = new StringWriter();
//                PrintWriter pw = new PrintWriter(sw);
//                e.printStackTrace(pw);
//
//                robot.opMode.telemetry.addData("Operator error ", e.getMessage());
//                robot.opMode.telemetry.addData("Operator error stack", sw.toString());
            }
        }
    }

    public void end()
    {
        shutDown = true;

        robot.opMode.telemetry.addData("Operator Thread stopped ", shutDown);

        robot.opMode.telemetry.update();
    }


    public void slides(double power, int dist)
    {

        double initialPos2 = robot.getSlide2().getCurrentPosition();

        while (Math.abs(robot.getSlide2().getCurrentPosition() - (initialPos2 + dist)) > 50)
        {
            robot.getSlide1().setPower(-0.9 * power);
            robot.getSlide2().setPower(power);
        }
        robot.getSlide1().setPower(0);
        robot.getSlide2().setPower(0);
    }


    public void grab() throws InterruptedException
    {
        if(robot.opMode.gamepad2.right_trigger > robot.getGrabberClosePos() && !blockGrabberInput)
        {
            robot.getGrabber().setPosition(grabberState ? robot.getGrabberClosePos() : robot.getGrabberOpenPos());
            grabberState = !grabberState;
            blockGrabberInput = true;
        }
        else if (robot.opMode.gamepad2.right_trigger < robot.getGrabberOpenPos() && blockGrabberInput)
        {
            blockGrabberInput = false;
        }

    }



    public void servoArmsHigh()
    {
        robot.getArm1().setPosition(robot.getArm1HighPos());
        robot.getArm2().setPosition(robot.getArm2HighPos());
    }

    public void servoArmsDown()
    {
        robot.getArm1().setPosition(robot.getArm1LowPos());
        robot.getArm2().setPosition(robot.getArm2LowPos());
    }


    public void sixBarHigh()
    {
        if(robot.opMode.gamepad2.dpad_up)
        {

            fourBarBack();

            servoArmsHigh();

            slides(0.5, 953);

            robot.getTwister().setPosition(robot.getTwisterDownPos());

        }
    }

    public void sixBarMedium()
    {

    }

    public void sixBarLow()
    {

    }

    public void servoTest()
    {
        if(robot.opMode.gamepad2.dpad_left)
        {
            robot.getFourBar1().setPosition(robot.getBar1BackPos());
            robot.getFourBar2().setPosition(robot.getBar2BackPos());

        }
    }



    public void sixBarDown()
    {
        if(robot.opMode.gamepad2.dpad_down)
        {

            robot.getTwister().setPosition(robot.getTwisterUpPos());

            fourBarFront();

            slides(-0.5, -953);

            servoArmsDown();



        }
    }


    public void fourBarFront()
    {
        robot.getFourBar1().setPosition(robot.getBar1FrontPos());
        robot.getFourBar2().setPosition(robot.getBar2FrontPos());
    }

    public void fourBarBack()
    {
        robot.getFourBar1().setPosition(robot.getBar1BackPos());
        robot.getFourBar2().setPosition(robot.getBar2BackPos());
    }

//    public void twister()
//    {
//        if(robot.opMode.gamepad2.left_trigger > robot.getTwisterUpPos() && !blockTwisterInput)
//        {
//            robot.getTwister().setPosition(twisterState ? robot.getTwisterUpPos() : robot.getTwisterDownPos());
//           twisterState = !twisterState;
//            blockTwisterInput = true;
//
//        }else if (robot.opMode.gamepad2.left_trigger < robot.getTwisterUpPos() && blockTwisterInput)
//        {
//            blockTwisterInput = false;
//        }
//
//    }


    public void holdSlidePos()
    {
        if(robot.opMode.gamepad2.left_trigger > 0)
        {
            robot.getSlide1().setPower(-0.2);
            robot.getSlide2().setPower(0.2);
        }
    }

    @Override
    public String toString()
    {
        return "OperatorThread{" +
                "name=" + OPERATOR_NAME +
                '}';
    }
}
