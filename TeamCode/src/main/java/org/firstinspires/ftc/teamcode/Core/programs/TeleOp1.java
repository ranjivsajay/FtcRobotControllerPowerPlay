package org.firstinspires.ftc.teamcode.Core.programs;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.message.redux.StopOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.Threads.DriveThread;
import org.firstinspires.ftc.teamcode.Core.Threads.OperatorThread;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.main.UpliftTele;
import org.firstinspires.ftc.teamcode.Core.toolkit.UpliftMath;

@TeleOp (name = "teleOp", group = "Opmodes")
public class TeleOp1 extends UpliftTele {

    UpliftRobot robot;

    DriveThread driverThread;
    OperatorThread operatorThread;
    boolean grabberState = true;
    boolean blockGrabberInput = false;
    boolean threadOn = false;

    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);

        createDriveThread(robot);
        createOperatorThread(robot);

    }

    @Override
    public void initAction() {






//        robot.getFourBar().setPosition(.75);
//        robot.getArmLeft().setPosition(robot.getArm1LowPos());
//        robot.getArmRight().setPosition(robot.getArm2LowPos());
//
//        robot.getGrabber1().setPosition(robot.getGrabber1ClosePos());
//        robot.getTwister().setPosition(robot.getTwisterDownPos());
//
//        robot.getgPosition().setPosition(robot.getPositionStack5());

        //testing grabber 2 pos
//        robot.getgPosition().setPosition(robot.getPositionStack1());
//        sleep(2000);
//        robot.getgPosition().setPosition(robot.getPositionStack2());
//        sleep(2000);
//        robot.getgPosition().setPosition(robot.getPositionStack3());
//        sleep(2000);
//        robot.getgPosition().setPosition(robot.getPositionStack4());
//        sleep(2000);
//        robot.getgPosition().setPosition(robot.getPositionStack5());
//        sleep(2000);
//        robot.getgPosition().setPosition(robot.getgPositionStore());

        //testing extension
          robot.getExtensionRight().setPosition(robot.getExtensionRightIn());
          robot.getExtensionLeft().setPosition(robot.getExtensionLeftIn());
          sleep(2000);
          robot.getExtensionRight().setPosition(robot.getExtensionRightOut());
          robot.getExtensionLeft().setPosition(robot.getExtensionLeftOut())
          ;

//        robot.getOdoMid().setPosition(robot.getOdoMidDown());
//        sleep(2000);
//        robot.getOdoMid().setPosition(robot.getOdoMidUp());



        driverThread.start();
        operatorThread.start();

    }

    @Override

    public void bodyLoop() throws InterruptedException {

        //telemetry.addData("magnet:" , robot.getMagnet().getValue());
        telemetry.update();



        double leftY = (.7 * Range.clip(-gamepad1.left_stick_y, -1, 1));
        double rightX = (.7 * Range.clip(gamepad1.right_stick_x, -1, 1));
        double leftX = (.7 * Range.clip(gamepad1.left_stick_x, -1, 1));

        double angle = 90 - Math.toDegrees(UpliftMath.atan2UL(leftY, leftX));
        double magnitude = 0.8 * Range.clip(sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2)), -1, 1);

        teleDrive(angle, magnitude, rightX, gamepad1.right_trigger, robot);


        robot.getSlide1().setPower(Range.clip(gamepad2.right_stick_y, -1, 1));
        robot.getSlide2().setPower(Range.clip(-gamepad2.right_stick_y, -1, 1));



    }

    @Override
    public void exit()
    {
        operatorThread.end();
        driverThread.end();

    }

    public void createDriveThread(UpliftRobot robot1)
    {

        driverThread = new DriveThread(robot1);
        telemetry.addData("Driver Thread started", driverThread.toString());

    }

    public void createOperatorThread(UpliftRobot robot1) {

        operatorThread = new OperatorThread(robot1);
        telemetry.addData("Operator Thread started", operatorThread.toString());

    }


    public static void teleDrive(double joystickAngle, double speedVal,
                                 double turnVal, float slowModeInput, UpliftRobot robot) {



    }
}



