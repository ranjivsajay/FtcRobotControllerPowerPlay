package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;

@Autonomous(name = "TestFieldCentric", group = "Opmodes")
public class TestTurn extends UpliftAutoImpl {

    @Override
    public void initAction()
    {
        telemetry.addData("Angle ", getAbsoluteAngle());
        telemetry.update();
        robot.getTwister().setPosition(robot.getTwisterDownPos());
        robot.getGrabber().setPosition(robot.getGrabberClosePos());

        robot.getFourBar1().setPosition(0.25);
        robot.getFourBar2().setPosition(0.75);


    }
    @Override
    public void body() throws InterruptedException {

        telemetry.addData("Angle ", getAbsoluteAngle());
        telemetry.update();

        while (opModeIsActive() && Math.abs(88 - getAbsoluteAngle()) > 1) {

            fieldCentricMove(0.75, .1, -0.3);
            servoArmsHigh();

        }
        stopMotors();

        fourBarBack();
        robot.getTwister().setPosition(robot.getTwisterUpPos());

        robot.getSlide1().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getSlide2().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.getSlide1().setTargetPosition(-1000);
        robot.getSlide2().setTargetPosition(1000);

        robot.getSlide1().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getSlide2().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.getSlide1().setPower(-0.6);
        robot.getSlide2().setPower(0.6);

        while (opModeIsActive() && robot.getSlide1().isBusy() && robot.getSlide2().isBusy()) {

            while (opModeIsActive() && Math.abs(145 - getAbsoluteAngle()) > 1) {

                fieldCentricMove(0.41, 0.17, -0.21);

            }
            stopMotors();
        }

//        while(robot.getPoleDetector().getDistance(DistanceUnit.CM) > 5)
//        {
//            moveBackward(0.3);
//        }
//        stopMotors();
//
//        robot.getFourBar1().setPosition(.1);
//        robot.getFourBar2().setPosition(.9);
//        Thread.sleep(300);
//
//        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
//        Thread.sleep(200);
//
//        while (opModeIsActive() && Math.abs(105 - getAbsoluteAngle()) > 1) {
//
//            fieldCentricMove(-0.3, -0.54, 0.29);
//
//        }
//        stopMotors();



    }
}



