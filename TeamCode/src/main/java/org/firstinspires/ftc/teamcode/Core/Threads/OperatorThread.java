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
                hold();

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

                if(robot.opMode.gamepad2.left_bumper)
                {
//                    robot.getFourBar1().setPosition(robot.getFourBar1().getPosition() + 0.01);
//                    robot.getFourBar2().setPosition(robot.getFourBar2().getPosition() - 0.01);
                }

                if(robot.opMode.gamepad2.right_bumper)
                {
//                    robot.getFourBar1().setPosition(robot.getFourBar1().getPosition() - 0.01);
//                    robot.getFourBar2().setPosition(robot.getFourBar2().getPosition() + 0.01);
                }

                robot.getSlide1().setPower(Range.clip(robot.opMode.gamepad2.right_stick_y, -1, 1));
                robot.getSlide2().setPower(Range.clip(-robot.opMode.gamepad2.right_stick_y, -1, 1));



//                if(robot.getMagnet().isPressed())
//                {
////                    robot.getArmLeft().setPosition(robot.getArm1HighPos());
////                    robot.getArmRight().setPosition(robot.getArm2HighPos());
//                }

                if (robot.opMode.gamepad2.right_trigger > .1)
                {
                    robot.getGrabber1().setPosition(robot.getGrabber1ClosePos());
                }

                if (robot.opMode.gamepad2.b)
                {
                    robot.getGrabber1().setPosition(robot.getGrabber1OpenPos());
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

    public void slidesPower(double power) {
        robot.getSlide1().setPower(-power);
        robot.getSlide2().setPower(power);
    }

    public void armHigh()
    {
        robot.getArmRight().setPosition(robot.getArmRightHighPos());
        robot.getArmLeft().setPosition(robot.getArmLeftHighPos());
    }

    public void slidesDist(double power, int dist)
    {
        robot.getSlide1().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getSlide2().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.getSlide1().setTargetPosition(-dist);
        robot.getSlide2().setTargetPosition(dist);

        slidesPower(power);

        robot.getSlide1().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getSlide2().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(robot.getSlide1().isBusy() && robot.getSlide2().isBusy())
        {
            telemetry.addData("status", "arm moving");
            telemetry.update();
        }

        robot.getSlide1().setPower(0);
        robot.getSlide2().setPower(0);


    }



    public void toggle4Bar()
    {
        //allows the 4bar to move to its forwards and backwards position with the same button
        if(robot.opMode.gamepad2.x && !block4BarInput)
        {
            robot.getFourBar().setPosition(fourBarState ? 0.25 : .75);
            fourBarState = !fourBarState;
            block4BarInput = true;
        }
        else if(!robot.opMode.gamepad2.x && block4BarInput)
        {
            block4BarInput = false;
        }
    }


    public void reset() throws InterruptedException {

        if (robot.opMode.gamepad2.a) // reset 6Bar
        {
            //   while(!robot.getMagnet().isPressed())
//            {
//                robot.getSlide1().setPower(0.5);
//                robot.getSlide2().setPower(-0.5);
//            }

            robot.getSlide1().setPower(0);
            robot.getSlide2().setPower(0);

            robot.getTwister().setPosition(robot.getTwisterDownPos());

            robot.getFourBar().setPosition(robot.getBarFrontPos());
            robot.getFourBar().setPosition(robot.getBarFrontPos());

            robot.getArmLeft().setPosition(robot.getArm1LowPos());
            robot.getArmRight().setPosition(robot.getArm2LowPos());

        }
    }


    public void servoArmsHigh()
    {
        if (robot.opMode.gamepad2.y) // raise servo on 6bar
        {
            armHigh();
        }

    }

    public void highBackwards()
    {
        if(robot.opMode.gamepad2.dpad_up) // max height backwards
        {
            robot.getFourBar().setPosition(robot.getBarBackPos());

            robot.getTwister().setPosition(robot.getTwisterUpPos());

            armHigh();

            slidesDist(0.5, 953);
        }
    }

    public void highForwards()
    {
        if(robot.opMode.gamepad2.dpad_left) // max height forwards
        {
            robot.getFourBar().setPosition(robot.getBarFrontPos());

            robot.getTwister().setPosition(robot.getTwisterDownPos());

            armHigh();

            slidesDist(0.5, 953);
        }
    }

    public void mediumBackwards()
    {
        if(robot.opMode.gamepad2.dpad_right) //  medium height backwards
        {
            robot.getFourBar().setPosition(robot.getBarBackPos());

            robot.getTwister().setPosition(robot.getTwisterUpPos());

            armHigh();

            slidesDist(0.5, 500);
        }
    }

    public void mediumForwards()
    {
        if(robot.opMode.gamepad2.dpad_down) // medium height forwards
        {
            robot.getFourBar().setPosition(robot.getBarFrontPos());

            robot.getTwister().setPosition(robot.getTwisterDownPos());

            armHigh();

            slidesDist(0.5, 500);
        }
    }

    public void stackHeight5()
    {
        if(robot.opMode.gamepad2.left_stick_y > .5) // 5 stack height
        {
            robot.getArmLeft().setPosition(robot.getArm1StackPos5());
            robot.getArmRight().setPosition(robot.getArm2StackPos5());
        }
    }

    public void stackHeight4()
    {
        if(robot.opMode.gamepad2.left_stick_x < -0.5) // 4 stack height
        {
            robot.getArmLeft().setPosition(robot.getArm1StackPos4());
            robot.getArmRight().setPosition(robot.getArm2StackPos4());
        }
    }

    public void stackHeight3()
    {
        if(robot.opMode.gamepad2.left_stick_x > 0.5) // 3 stack height
        {

            robot.getArmLeft().setPosition(robot.getArm1StackPos3());
            robot.getArmRight().setPosition(robot.getArm2StackPos3());
        }
    }

    public void stackHeight2()
    {
        if(robot.opMode.gamepad2.left_stick_y < -0.5) // 2 stack height
        {
            robot.getArmLeft().setPosition(robot.getArm1StackPos2());
            robot.getArmRight().setPosition(robot.getArm2StackPos2());
        }
    }

    public void hold()
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
