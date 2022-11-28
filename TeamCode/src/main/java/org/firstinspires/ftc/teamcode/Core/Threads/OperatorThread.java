package org.firstinspires.ftc.teamcode.Core.Threads;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.main.UpliftTele;

import java.io.PrintWriter;
import java.io.StringWriter;

public class OperatorThread extends Thread
{


    private UpliftRobot robot;

    private static final String OPERATOR_NAME = "OperatorThreadName";

    private boolean shutDown = false;

    private boolean grabberState;
    private boolean blockGrabberInput;

    double arm1HighPos;
    double arm2HighPos;


    public OperatorThread(UpliftRobot robot)
    {
        this.robot = robot;

        this.grabberState = true;
        this.blockGrabberInput = false;

        this.arm1HighPos = .4;
        this.arm2HighPos = .0;
    }

    public void end()
    {
        shutDown = true;

        telemetry.addData("Operator Thread stopped ", shutDown);
    }

    @Override
    public void run()
    {
        while(!shutDown)
        {
            try
            {

                grab();

                armHigh();

                holdSlidePos();

                robot.getSlide1().setPower(Range.clip(gamepad2.right_stick_y, -1, 1));
                robot.getSlide2().setPower(Range.clip(-gamepad2.right_stick_y, -1, 1));

                // todo: validate user responsiveness and set sleep
                sleep(50);
            } catch (Exception e) {
                e.printStackTrace();
                e.printStackTrace();

                StringWriter sw = new StringWriter();
                PrintWriter pw = new PrintWriter(sw);
                e.printStackTrace(pw);

                telemetry.addData("Operator error ", e.getMessage());
                telemetry.addData("Operator error stack", sw.toString());
            }
        }
    }

    public void slides(double power, double dist)
    {
        double initialPos2 = robot.getSlide2().getCurrentPosition();

        while (Math.abs(robot.getSlide2().getCurrentPosition() - (initialPos2 + dist)) > 50)
        {
            robot.getSlide1().setPower(-power);
            robot.getSlide2().setPower(power);
        }
        robot.getSlide1().setPower(0);
        robot.getSlide2().setPower(0);
    }


    public void grab() throws InterruptedException {
        if(gamepad2.right_trigger > robot.getGrabberClosePos() && !blockGrabberInput)
        {
            robot.getGrabber().setPosition(grabberState ? robot.getGrabberClosePos() : robot.getGrabberOpenPos());
            grabberState = !grabberState;
            blockGrabberInput = true;
        }
        else if (gamepad2.right_trigger < robot.getGrabberOpenPos() && blockGrabberInput)
        {
            blockGrabberInput = false;
        }
    }

    public void armHigh()
    {
        if(gamepad2.dpad_up)
        {

            robot.getArm2().setPosition(arm2HighPos);
            robot.getArm1().setPosition(arm1HighPos);

            slides(0.5, 953);


        }
    }

    public void holdSlidePos()
    {
        if(gamepad2.left_trigger > 0)
        {
            robot.getSlide1().setPower(-0.1);
            robot.getSlide2().setPower(0.1);
        }
    }

    @Override
    public String toString() {
        return "OperatorThread{" +
                "name=" + OPERATOR_NAME +
                '}';
    }
}
