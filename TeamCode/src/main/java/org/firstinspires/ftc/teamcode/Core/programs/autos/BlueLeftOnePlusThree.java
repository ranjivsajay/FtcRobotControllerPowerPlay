package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;

@Autonomous(name = "1 + 3", group = "Opmodes")
public class BlueLeftOnePlusThree extends UpliftAutoImpl {
    @Override
    public void initAction() {

        robot.getTwister().setPosition(robot.getTwisterDownPos());
        robot.getGrabber().setPosition(robot.getGrabberClosePos());

        robot.getFourBar1().setPosition(0.25);
        robot.getFourBar2().setPosition(0.75);


    }

    @Override
    public void body() throws InterruptedException {
        int parkLocation = robot.pipeline1.location;
        robot.getWebcam().setPipeline(robot.pipeline2);

        while (opModeIsActive() && Math.abs(88 - getAbsoluteAngle()) > 1) {

            fieldCentricMove(0.75, .1, -0.3);

        }
        stopMotors();

        while (opModeIsActive() && Math.abs(145 - getAbsoluteAngle()) > 1) {

            fieldCentricMove(0.4, 0.17, -0.3);

        }
        stopMotors();

        moveBackwardHigh(0.2, 0.9, 450, 900);

        robot.getFourBar1().setPosition(.1);
        robot.getFourBar2().setPosition(.9);
        Thread.sleep(300);

        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
        Thread.sleep(200);

        while (opModeIsActive() && Math.abs(105 - getAbsoluteAngle()) > 1) {

            fieldCentricMove(-0.3, -0.54, 0.29);

        }
        stopMotors();

        robot.getGrabber().setPosition(robot.getGrabberClosePos());

        robot.getSlide1().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getSlide2().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.getSlide1().setPower(0.1);
        robot.getSlide2().setPower(-0.1);
        Thread.sleep(400);

        turnToPID(93);

        robot.getArm1().setPosition(robot.getArm1StackPos5());
        robot.getArm2().setPosition(robot.getArm2StackPos5());
        robot.getGrabber().setPosition(robot.getGrabberOpenPos());

        robot.getTwister().setPosition(robot.getTwisterDownPos());
        fourBarFront();
        turnToPID(93);
        Thread.sleep(500);

        while (robot.getConeDetector().getDistance(DistanceUnit.CM) > 8) {
            moveForward(0.3);
        }
        stopMotors();
//
//
        robot.getGrabber().setPosition(robot.getGrabberClosePos());
        Thread.sleep(500);

        robot.getFourBar1().setPosition(0.4);
        robot.getFourBar2().setPosition(0.6);
        Thread.sleep(500);

        while (opModeIsActive() && Math.abs(135 - getAbsoluteAngle()) > 1) {

            fieldCentricMove(-0.07, 0.61, -0.19);

        }
        stopMotors();

        moveBackwardHigh(0.2, 0.9, 350, 1000);

        robot.getFourBar1().setPosition(.1);
        robot.getFourBar2().setPosition(.9);
        Thread.sleep(300);

        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
        Thread.sleep(200);

        while (opModeIsActive() && Math.abs(105 - getAbsoluteAngle()) > 1) {

            fieldCentricMove(-0.3, -0.54, 0.29);

        }
        stopMotors();

        robot.getGrabber().setPosition(robot.getGrabberClosePos());

        robot.getSlide1().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getSlide2().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.getSlide1().setPower(0.1);
        robot.getSlide2().setPower(-0.1);
        Thread.sleep(400);

        turnToPID(93);

        robot.getArm1().setPosition(robot.getArm1StackPos4());
        robot.getArm2().setPosition(robot.getArm2StackPos4());
        robot.getGrabber().setPosition(robot.getGrabberOpenPos());

        robot.getTwister().setPosition(robot.getTwisterDownPos());
        fourBarFront();
        turnToPID(93);
        Thread.sleep(500);

        while (robot.getConeDetector().getDistance(DistanceUnit.CM) > 8) {
            moveForward(0.3);
        }
        stopMotors();
//
//
        robot.getGrabber().setPosition(robot.getGrabberClosePos());
        Thread.sleep(500);

        robot.getFourBar1().setPosition(0.4);
        robot.getFourBar2().setPosition(0.6);
        Thread.sleep(500);

        while (opModeIsActive() && Math.abs(135 - getAbsoluteAngle()) > 1) {

            fieldCentricMove(-0.07, 0.62, -0.2);

        }
        stopMotors();

        moveBackwardHigh(0.2, 0.9, 350, 1100);

        robot.getFourBar1().setPosition(.1);
        robot.getFourBar2().setPosition(.9);
        Thread.sleep(300);

        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
        Thread.sleep(200);

        while (opModeIsActive() && Math.abs(105 - getAbsoluteAngle()) > 1) {

            fieldCentricMove(-0.3, -0.54, 0.29);

        }
        stopMotors();

        robot.getGrabber().setPosition(robot.getGrabberClosePos());

        robot.getSlide1().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getSlide2().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.getSlide1().setPower(0.1);
        robot.getSlide2().setPower(-0.1);
        Thread.sleep(400);

        turnToPID(93);

        robot.getArm1().setPosition(robot.getArm1StackPos3());
        robot.getArm2().setPosition(robot.getArm2StackPos3());
        robot.getGrabber().setPosition(robot.getGrabberOpenPos());

        robot.getTwister().setPosition(robot.getTwisterDownPos());
        fourBarFront();
        turnToPID(93);
        Thread.sleep(500);

        while (robot.getConeDetector().getDistance(DistanceUnit.CM) > 8) {
            moveForward(0.3);
        }
        stopMotors();
//
//
        robot.getGrabber().setPosition(robot.getGrabberClosePos());
        Thread.sleep(500);

        robot.getFourBar1().setPosition(0.4);
        robot.getFourBar2().setPosition(0.6);
        Thread.sleep(500);

        while (opModeIsActive() && Math.abs(135 - getAbsoluteAngle()) > 1) {

            fieldCentricMove(-0.07, 0.65, -0.2);

        }
        stopMotors();

        moveBackwardHigh(0.2, 0.9, 350, 1000);

        robot.getFourBar1().setPosition(.1);
        robot.getFourBar2().setPosition(.9);
        Thread.sleep(300);

        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
        Thread.sleep(200);

        while (opModeIsActive() && Math.abs(105 - getAbsoluteAngle()) > 1) {

            fieldCentricMove(-0.3, -0.54, 0.29);

        }
        stopMotors();

        robot.getGrabber().setPosition(robot.getGrabberClosePos());

        robot.getSlide1().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getSlide2().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.getSlide1().setPower(0.1);
        robot.getSlide2().setPower(-0.1);
        Thread.sleep(400);

        turnToPID(93);


        robot.getFourBar1().setPosition(0.4);
        robot.getFourBar2().setPosition(0.6);
        Thread.sleep(500);

        robot.getWebcam().setPipeline(robot.pipeline1);

        if (parkLocation == 1) {
            moveForward(.5,500);
        } else if (parkLocation == 2) {
            moveBackward(.5, 300);

        } else if (parkLocation == 3) {
            moveBackward(.6, 1250);


//        }

        }
//
//
    }
}
