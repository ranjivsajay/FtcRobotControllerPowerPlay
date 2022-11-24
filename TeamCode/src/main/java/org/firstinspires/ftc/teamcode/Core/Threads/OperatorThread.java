package org.firstinspires.ftc.teamcode.Core.Threads;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;

public class OperatorThread implements Runnable
{


    private UpliftRobot robot;

    private LinearOpMode opMode;


    private boolean grabberState;
    private boolean blockGrabberInput;


    public OperatorThread(UpliftRobot robot, LinearOpMode opMode)
    {
        this.robot = robot;
        this.grabberState = true;
        this.blockGrabberInput = false;
        this.opMode = opMode;
    }

    @Override
    public void run()
    {

        if(gamepad2.dpad_left)
        {
            robot.getSlide1().setPower(.1);
        }

        if(gamepad2.dpad_up)
        {
            robot.getSlide1().setPower(-.1);
        }
        if(gamepad2.dpad_down)
        {
            robot.getSlide2().setPower(-.1);
        }

        if(gamepad2.dpad_right)
        {
            robot.getSlide2().setPower(.1);
        }

        robot.getSlide1().setPower(
                (1.0 - 0.8 * gamepad2.left_trigger) * Range.clip(gamepad2.right_stick_y, -1, 1));
        robot.getSlide2().setPower(
                (1.0 - 0.8 * gamepad2.left_trigger) * Range.clip(-gamepad2.right_stick_y, -1, 1));

        try {
            grab();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        try {
            cap();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
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

    public void cap() throws InterruptedException {
        if(gamepad2.left_trigger > 0)
        {
            robot.getGrabber().setPosition(.13);

        }
    }
}
