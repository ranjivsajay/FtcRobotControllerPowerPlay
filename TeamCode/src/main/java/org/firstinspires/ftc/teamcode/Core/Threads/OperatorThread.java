package org.firstinspires.ftc.teamcode.Core.Threads;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;

public class OperatorThread extends Thread
{


    private UpliftRobot robot;

    private static final String OPERATOR_NAME = "OperatorThreadName";

    private boolean shutDown = false;

    private boolean grabberState;
    private boolean blockGrabberInput;

    private boolean fourBarState;
    private boolean block4BarInput;



    private boolean twisterState;
    private boolean blockTwisterInput;



    public OperatorThread(UpliftRobot robot)
    {
        this.robot = robot;

        this.grabberState = true;
        this.blockGrabberInput = false;

        this.fourBarState = true;
        this.block4BarInput = false;

    }

    @Override
    public void run()
    {
        while(!shutDown)
        {
            try
            {
                toggleGrab();

                toggle4Bar();

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

                holdSlidePos();

//                senseCone();

                if(robot.opMode.gamepad2.left_bumper)
                {
                    robot.getFourBar1().setPosition(robot.getFourBar1().getPosition() + 0.05);
                    robot.getFourBar2().setPosition(robot.getFourBar2().getPosition() - 0.05);
                }

                if(robot.opMode.gamepad2.right_bumper)
                {
                    robot.getFourBar1().setPosition(robot.getFourBar1().getPosition() - 0.05);
                    robot.getFourBar2().setPosition(robot.getFourBar2().getPosition() + 0.05);
                }

                robot.getSlide1().setPower(0.75 * Range.clip(robot.opMode.gamepad2.right_stick_y, -1, 1));
                robot.getSlide2().setPower(0.75 * Range.clip(-robot.opMode.gamepad2.right_stick_y, -1, 1));

                if(robot.getMagnet().isPressed())
                {
                    robot.getArm1().setPosition(robot.getArm1HighPos());
                    robot.getArm2().setPosition(robot.getArm2HighPos());
                }

//                robot.opMode.telemetry.addData("magnet", robot.getMagnet().isPressed());
//                robot.opMode.telemetry.addData("odoRight" , robot.getOdoRight().getCurrentPosition());
//                robot.opMode.telemetry.update();




                // todo: validate user responsiveness and set sleep
//                sleep(50);
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

    public void toggle4Bar()
    {
        //allows the 4bar to move to its forwards and backwards position with the same button
        if(robot.opMode.gamepad2.x && !block4BarInput)
        {
            robot.getFourBar1().setPosition(fourBarState ? robot.getBar1FrontPos() : 0.38);
            robot.getFourBar2().setPosition(fourBarState ? robot.getBar2FrontPos() : 0.62);
            fourBarState = !fourBarState;
            block4BarInput = true;
        }
        else if(!robot.opMode.gamepad2.x && block4BarInput)
        {
            block4BarInput = false;
        }
    }

    public void toggleGrab() throws InterruptedException
    {
        if(robot.opMode.gamepad2.right_trigger > robot.getGrabberOpenPos() && !blockGrabberInput)
        {
        robot.getGrabber().setPosition(grabberState ? robot.getGrabberOpenPos() : robot.getGrabberClosePos());
        grabberState = !grabberState;
        blockGrabberInput = true;
        }
        else if (robot.opMode.gamepad2.right_trigger < robot.getGrabberClosePos() && blockGrabberInput)
        {
            blockGrabberInput = false;
        }
    }

    public void senseCone()
    {

        double value = robot.getConeDetector().getDistance(DistanceUnit.CM);
        if(value <= 5 && robot.opMode.gamepad2.right_trigger == 0)
        {
            robot.getGrabber().setPosition(robot.getGrabberClosePos());
        }
        else
        {
            if(robot.opMode.gamepad2.right_trigger > robot.getGrabberOpenPos() && !blockGrabberInput)
            {
                robot.getGrabber().setPosition(grabberState ? robot.getGrabberOpenPos() : robot.getGrabberClosePos());
                grabberState = !grabberState;
                blockGrabberInput = true;
            }
            else if (robot.opMode.gamepad2.right_trigger < robot.getGrabberClosePos() && blockGrabberInput)
            {
                blockGrabberInput = false;
            }
        }
    }

    public void reset()
    {
        if (robot.opMode.gamepad2.a) // reset grabber pos
        {
            //move 6 bar motors down
//            while(!robot.getMagnet().isPressed())
//            {
//                robot.getSlide1().setPower(0.2);
//                robot.getSlide2().setPower(-0.2);
//            }

            robot.getArm1().setPosition(robot.getArm1LowPos());
            robot.getArm2().setPosition(robot.getArm2LowPos());

            robot.getFourBar1().setPosition(robot.getBar1FrontPos());
            robot.getFourBar2().setPosition(robot.getBar2FrontPos());

            robot.getTwister().setPosition(robot.getTwisterDownPos());
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

//            slides(0.5, 953);
            robot.getArm1().setPosition(robot.getArm1HighPos());
            robot.getArm2().setPosition(robot.getArm2HighPos());


            robot.getFourBar1().setPosition(robot.getBar1BackPos());
            robot.getFourBar2().setPosition(robot.getBar2BackPos());

            robot.getTwister().setPosition(robot.getTwisterUpPos());

        }
    }

    public void highForwards()
    {
        if(robot.opMode.gamepad2.dpad_left) // max height forwards
        {
//            slides(0.5, 953);
            robot.getArm1().setPosition(robot.getArm1HighPos());
            robot.getArm2().setPosition(robot.getArm2HighPos());

            robot.getFourBar1().setPosition(robot.getBar1FrontPos());
            robot.getFourBar2().setPosition(robot.getBar2FrontPos());

            robot.getTwister().setPosition(robot.getTwisterDownPos());
        }
    }

    public void mediumBackwards()
    {
        if(robot.opMode.gamepad2.dpad_right) //  medium height backwards
        {
////            slides(0.5, 700);
//            robot.getArm1().setPosition(robot.getArm1HighPos());
//            robot.getArm2().setPosition(robot.getArm2HighPos());
//
//            robot.getFourBar1().setPosition(robot.getBar1BackPos());
//            robot.getFourBar2().setPosition(robot.getBar2BackPos());
//
//            robot.getTwister().setPosition(robot.getTwisterUpPos());
//
//            robot.getFourBar1().setPosition(robot.getBar1FrontPos());
//            robot.getFourBar2().setPosition(robot.getBar2FrontPos());


        }
    }

    public void mediumForwards()
    {
        if(robot.opMode.gamepad2.dpad_down) // medium height forwards
        {
//            slides(0.5, 700);
            robot.getArm1().setPosition(robot.getArm1HighPos());
            robot.getArm2().setPosition(robot.getArm2HighPos());

            robot.getFourBar1().setPosition(robot.getBar1FrontPos());
            robot.getFourBar2().setPosition(robot.getBar2FrontPos());

            robot.getTwister().setPosition(robot.getTwisterDownPos());

            robot.getFourBar1().setPosition(.38);
            robot.getFourBar2().setPosition(.62);
        }
    }

    public void stackHeight5()
    {
        if(robot.opMode.gamepad2.left_stick_y > .5) // 5 stack height
        {
            robot.getArm1().setPosition(robot.getArm1StackPos5());
            robot.getArm2().setPosition(robot.getArm2StackPos5());
        }
    }

    public void stackHeight4()
    {
        if(robot.opMode.gamepad2.left_stick_x < -0.5) // 4 stack height
        {
            robot.getArm1().setPosition(robot.getArm1StackPos4());
            robot.getArm2().setPosition(robot.getArm2StackPos4());
        }
    }

    public void stackHeight3()
    {
        if(robot.opMode.gamepad2.left_stick_x > 0.5) // 3 stack height
        {

            robot.getArm1().setPosition(robot.getArm1StackPos3());
            robot.getArm2().setPosition(robot.getArm2StackPos3());
        }
    }

    public void stackHeight2()
    {
        if(robot.opMode.gamepad2.left_stick_y < -0.5) // 2 stack height
        {
            robot.getArm1().setPosition(robot.getArm1StackPos2());
            robot.getArm2().setPosition(robot.getArm2StackPos2());
        }
    }



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
