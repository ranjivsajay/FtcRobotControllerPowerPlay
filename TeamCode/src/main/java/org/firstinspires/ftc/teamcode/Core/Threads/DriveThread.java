package org.firstinspires.ftc.teamcode.Core.Threads;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.main.UpliftTele;
import org.firstinspires.ftc.teamcode.Core.toolkit.UpliftMath;

import java.io.PrintWriter;
import java.io.StringWriter;

public class DriveThread extends Thread
{
    private UpliftRobot robot;
    private static final String DRIVER_NAME = "DriverThreadName";

    private boolean shutDown = false;

    public DriveThread(UpliftRobot robot)
    {
        this.robot = robot;
    }


    @Override
    public void run()
    {
        while(!shutDown)
        {
            try
            {
                double leftY =(.7 * Range.clip(-robot.opMode.gamepad1.left_stick_y, -1, 1));
                double rightX = (.7 * Range.clip(robot.opMode.gamepad1.right_stick_x, -1, 1));
                double leftX = ( .7 * Range.clip(robot.opMode.gamepad1.left_stick_x, -1, 1));

                double angle = 90 - Math.toDegrees(UpliftMath.atan2UL(leftY, leftX));
                double magnitude = 0.8 * Range.clip(sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2)), -1, 1);

                teleDrive(angle, magnitude, rightX, robot.opMode.gamepad1.right_trigger,robot.opMode.gamepad1.left_trigger, robot);

                if(robot.opMode.gamepad1.left_bumper)
                {
                    robot.getFourBar1().setPosition(.38);
                    robot.getFourBar2().setPosition(.62);
                }


                // todo: validate user responsiveness and set sleep
                sleep(50);
            } catch (Exception e) {
                e.printStackTrace();
//
//                StringWriter sw = new StringWriter();
//                PrintWriter pw = new PrintWriter(sw);
//                e.printStackTrace(pw);
//
//                telemetry.addData("Drive error", e.getMessage());
//                telemetry.addData("Drive error stack", sw.toString());
            }
        }
    }

    public static void teleDrive ( double joystickAngle, double speedVal,
                                   double turnVal, float slowModeInput, float fastModeInput, UpliftRobot robot) {
        double lfPow = sin(toRadians(joystickAngle) + (0.25 * PI)) * speedVal + turnVal;
        double rfPow = sin(toRadians(joystickAngle) - (0.25 * PI)) * speedVal - turnVal;
        double lbPow = sin(toRadians(joystickAngle) - (0.25 * PI)) * speedVal + turnVal;
        double rbPow = sin(toRadians(joystickAngle) + (0.25 * PI)) * speedVal - turnVal;
//
        // find max total input out of the 4 motors
        double maxVal = abs(lfPow);
        if (abs(rfPow) > maxVal) {
            maxVal = abs(rfPow);
        }
        if (abs(lbPow) > maxVal) {
            maxVal = abs(lbPow);
        }
        if (abs(rbPow) > maxVal) {
            maxVal = abs(rbPow);
        }

        if (maxVal < (1 / sqrt(2))) {
            maxVal = 1 / sqrt(2);
        }

        // set the scaled powers
        float speedFactor = 1.0f;
        if (slowModeInput > 0.1f)
            speedFactor = 0.5f;

        if(fastModeInput > 0.1f)
            speedFactor = 1.1f;

        robot.getLeftFront().setPower(speedFactor * (lfPow / maxVal));
        robot.getLeftBack().setPower(speedFactor * (lbPow / maxVal));
        robot.getRightBack().setPower(speedFactor * (rbPow / maxVal));
        robot.getRightFront().setPower(speedFactor * (rfPow / maxVal));
    }

    public void end()
    {
        shutDown = true;

        robot.opMode.telemetry.addData("Driver Thread stopped ", shutDown);

        robot.opMode.telemetry.update();

    }

    @Override
    public String toString() {
        return "DriveThread{" +
                "name=" + DRIVER_NAME +
                '}';
    }
}
