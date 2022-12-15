package org.firstinspires.ftc.teamcode.Core.Threads;

import static java.lang.Math.sqrt;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.toolkit.UpliftMath;

public class DriveAuto extends Thread{

    private UpliftRobot robot;
    private boolean shutDown = false;

    public DriveAuto(UpliftRobot robot)
    {
        this.robot = robot;
    }


    @Override
    public void run() {
        while(!shutDown)
        {
            try
            {
                high();

            } catch (Exception e)
            {
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

    public void high()
    {
        servoArmsHigh();

        slides(0.7, 953);

        fourBarBack();

        robot.getTwister().setPosition(robot.getTwisterDownPos());
    }

    public void servoArmsHigh()
    {
        robot.getArm1().setPosition(robot.getArm1HighPos());
        robot.getArm2().setPosition(robot.getArm2HighPos());
    }

    public void slides(double power, double dist) {

//        double initialPos1 = robot.getSlide2().getCurrentPosition();
        double initialPos2 = robot.getSlide2().getCurrentPosition();

        while (Math.abs(robot.getSlide2().getCurrentPosition() - (initialPos2 + dist)) > 50) {
            robot.getSlide1().setPower(-power);
            robot.getSlide2().setPower(power);

        }
        robot.getSlide1().setPower(0);
        robot.getSlide2().setPower(0);
    }

    public void fourBarBack()
    {
        robot.getFourBar1().setPosition(robot.getBar1BackPos());
        robot.getFourBar2().setPosition(robot.getBar2BackPos());
    }

    public void end()
    {
        shutDown = true;

        robot.opMode.telemetry.addData("Driver Thread stopped ", shutDown);

        robot.opMode.telemetry.update();

    }

}
