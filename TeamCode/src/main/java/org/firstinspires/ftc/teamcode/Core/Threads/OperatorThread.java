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
                grab();

                reset();

                servoArmsHigh();

                highBackwards();

                highForwards();

                mediumBackwards();

                mediumForwards();

                stackHeight5();

                stackHeight4();

                stackHeight3();

                stackHeight2();

                openGrabber();

                holdSlidePos();

                if (robot.opMode.gamepad2.x) // bring 4bar up to drive
                {
                    robot.getFourBar1().setPosition(.38);
                    robot.getFourBar2().setPosition(.62);
                }


               robot.opMode.telemetry.addData("slide1" , robot.getSlide1().getCurrentPosition());
               robot.opMode.telemetry.addData("slide2" , robot.getSlide2().getCurrentPosition());
               robot.opMode.telemetry.update();




                robot.getSlide1().setPower(0.75 * Range.clip(robot.opMode.gamepad2.right_stick_y, -1, 1));
                robot.getSlide2().setPower(0.75 * Range.clip(-robot.opMode.gamepad2.right_stick_y, -1, 1));


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

    public void reset()
    {
        if (robot.opMode.gamepad2.a) // reset grabber pos
        {
            //move 6 bar motors down
            slides(-0.4, -950);
            robot.getArm1().setPosition(robot.getArm1LowPos());
            robot.getArm2().setPosition(robot.getArm2LowPos());

            robot.getFourBar1().setPosition(robot.getBar1FrontPos());
            robot.getFourBar2().setPosition(robot.getBar2FrontPos());

            robot.getTwister().setPosition(robot.getTwisterDownPos());
        }
    }

    public void grab() throws InterruptedException
    {
        if(robot.opMode.gamepad2.left_trigger > robot.getGrabberClosePos() && !blockGrabberInput)
        {
            robot.getGrabber().setPosition(grabberState ? robot.getGrabberClosePos() : robot.getGrabberOpenPos());
            grabberState = !grabberState;
            blockGrabberInput = true;
        }
        else if (robot.opMode.gamepad2.left_trigger < robot.getGrabberOpenPos() && blockGrabberInput)
        {
            blockGrabberInput = false;
        }

    }



    public void servoArmsHigh()
    {
        if (robot.opMode.gamepad2.y) // raise servo on 6bar
        {
            robot.getArm1().setPosition(robot.getArm1HighPos());
            robot.getArm2().setPosition(robot.getArm2HighPos());
        }

    }

    public void highBackwards()
    {
        if(robot.opMode.gamepad2.dpad_up) // max height backwards
        {

            slides(0.5, 953);
            servoArmsHigh();

            robot.getFourBar1().setPosition(robot.getBar1BackPos());
            robot.getFourBar2().setPosition(robot.getBar2BackPos());

            robot.getTwister().setPosition(robot.getTwisterUpPos());

        }
    }

    public void highForwards()
    {
        if(robot.opMode.gamepad2.dpad_left) // max height forwards
        {
//                    move motors to max height
            slides(0.5, 953);
            servoArmsHigh();

            robot.getFourBar1().setPosition(robot.getBar1FrontPos());
            robot.getFourBar2().setPosition(robot.getBar2FrontPos());

            robot.getTwister().setPosition(robot.getTwisterDownPos());
        }
    }

    public void mediumBackwards()
    {
        if(robot.opMode.gamepad2.dpad_right) //  medium height backwards
        {
//                     move motors to mid height
            slides(0.5, 700);
            servoArmsHigh();

            robot.getFourBar1().setPosition(robot.getBar1BackPos());
            robot.getFourBar2().setPosition(robot.getBar2BackPos());

            robot.getTwister().setPosition(robot.getTwisterUpPos());

        }
    }

    public void mediumForwards()
    {
        if(robot.opMode.gamepad2.dpad_down) // medium height forwards
        {
//                    move motors to mid height
            slides(0.5, 700);
            servoArmsHigh();

            robot.getFourBar1().setPosition(robot.getBar1FrontPos());
            robot.getFourBar2().setPosition(robot.getBar2FrontPos());

            robot.getTwister().setPosition(robot.getTwisterDownPos());
        }
    }

    public void stackHeight5()
    {
        if(robot.opMode.gamepad2.left_stick_y > .5) // 5 stack height
        {
            robot.getArm1().setPosition(.5);
            robot.getArm2().setPosition(.5);
        }
    }

    public void stackHeight4()
    {
        if(robot.opMode.gamepad2.left_stick_x < -0.5) // 4 stack height
        {
            robot.getArm1().setPosition(.48);
            robot.getArm2().setPosition(.52);
        }
    }

    public void stackHeight3()
    {
        if(robot.opMode.gamepad2.left_stick_x > 0.5) // 3 stack height
        {

            robot.getArm1().setPosition(.45);
            robot.getArm2().setPosition(.55);
        }
    }

    public void stackHeight2()
    {
        if(robot.opMode.gamepad2.left_stick_y < -0.5) // 2 stack height
        {
            robot.getArm1().setPosition(.43);
            robot.getArm2().setPosition(.57);
        }
    }

    public void openGrabber()
    {
        if (robot.opMode.gamepad2.b)
        {
            robot.getGrabber().setPosition(robot.getGrabberOpenPos());
        }
    }

    public void holdSlidePos()
    {
        if(robot.opMode.gamepad2.left_trigger > 0)
        {
            robot.getSlide1().setPower(-0.9);
            robot.getSlide2().setPower(0.9);
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
