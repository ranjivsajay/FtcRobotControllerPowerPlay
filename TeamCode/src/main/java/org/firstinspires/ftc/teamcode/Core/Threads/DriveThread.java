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
import org.firstinspires.ftc.teamcode.Core.toolkit.UpliftMath;

public class DriveThread implements Runnable
{
    private UpliftRobot robot;
    private LinearOpMode opMode;

    public DriveThread(UpliftRobot robot, LinearOpMode opMode)
    {
        this.robot = robot;
        this.opMode = opMode;
    }

    @Override
    public void run()
    {
        telemetry.addData("magnetic sensor", robot.getMagneticSensor().isPressed());

        telemetry.addData("left front motor" , robot.getLeftFront().getCurrentPosition());
        telemetry.addData("right front motor" , robot.getRightFront().getCurrentPosition());
        telemetry.addData("left back motor" , robot.getLeftBack().getCurrentPosition());
        telemetry.addData("right back motor" , robot.getRightBack().getCurrentPosition());
        telemetry.addData("left front power" , robot.getLeftFront().getPower());
        telemetry.addData("right front power" , robot.getRightFront().getPower());
        telemetry.addData("left back power" , robot.getLeftBack().getPower());
        telemetry.addData("right back power" , robot.getRightBack().getPower());
        telemetry.update();
        double leftY =(.7 * Range.clip(-gamepad1.left_stick_y, -1, 1));
        double rightX = (.7 * Range.clip(gamepad1.right_stick_x, -1, 1));
        double leftX = ( .7 * Range.clip(gamepad1.left_stick_x, -1, 1));


        double angle = 90 - Math.toDegrees(UpliftMath.atan2UL(leftY, leftX));
        double magnitude = 0.8 * Range.clip(sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2)), -1, 1);

        teleDrive(angle, magnitude, rightX, gamepad1.right_trigger, robot);



    }



    public static void teleDrive ( double joystickAngle, double speedVal,
                                   double turnVal, float slowModeInput, UpliftRobot robot) {
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

        robot.getLeftFront().setPower(speedFactor * (lfPow / maxVal));
        robot.getLeftBack().setPower(speedFactor * (lbPow / maxVal));
        robot.getRightBack().setPower(speedFactor * (rbPow / maxVal));
        robot.getRightFront().setPower(speedFactor * (rfPow / maxVal));
    }



}
