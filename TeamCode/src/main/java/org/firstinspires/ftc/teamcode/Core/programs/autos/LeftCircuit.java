package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;

@Autonomous(name = "LeftCircuit", group = "Opmodes")
public class LeftCircuit extends UpliftAutoImpl {

    @Override
    public void initAction()
    {

        robot.getTwister().setPosition(robot.getTwisterDownPos());
        robot.getGrabber().setPosition(robot.getGrabberClosePos());

        robot.getFourBar1().setPosition(0.25);
        robot.getFourBar2().setPosition(0.75);


    }
    @Override
    public void body() throws InterruptedException {

        int parkLocation = robot.pipeline1.location;
        robot.getWebcam().setPipeline(robot.pipeline2);

        while (opModeIsActive() && Math.abs(-77.5 - getAbsoluteAngle()) > 1) {

            fieldCentricMove(0.57, -.1, .55);
            fourBarFront();
            robot.getArm2().setPosition(.22);
            robot.getArm1().setPosition(.75);

        }
        stopMotors();
        Thread.sleep(300);
        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
        Thread.sleep(300);

        moveBackward(.5,50);

        robot.getFourBar1().setPosition(0.25);
        robot.getFourBar2().setPosition(0.75);

        robot.getGrabber().setPosition(robot.getGrabberClosePos());


        while (opModeIsActive() && Math.abs(84 - getAbsoluteAngle()) > 1) {

            fieldCentricMove(0.48, -.05, -0.4);
            servoArmsHigh();

        }
        stopMotors();

        turnToPID(92);

        robot.getGrabber().setPosition(robot.getGrabberClosePos());

        robot.getArm1().setPosition(robot.getArm1StackPos5());
        robot.getArm2().setPosition(robot.getArm2StackPos5());

        Thread.sleep(100);


        robot.getTwister().setPosition(robot.getTwisterDownPos());
        fourBarFront();
        Thread.sleep(300);
        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
        turnToPID(92);
        Thread.sleep(100);

        moveForward(.5, 200);

        while (robot.getConeDetector().getDistance(DistanceUnit.CM) > 8) {
            moveForward(0.25);
        }
        stopMotors();


        robot.getGrabber().setPosition(robot.getGrabberClosePos());
        Thread.sleep(300);

        robot.getFourBar1().setPosition(0.45);
        robot.getFourBar2().setPosition(0.55);
        Thread.sleep(300);


        while (opModeIsActive() && Math.abs(135 - getAbsoluteAngle()) > 1) {

            fieldCentricMove(-0.12, 0.47, -0.23);
            fourBarFront();
            servoArmsHigh();

        }
        stopMotors();
        moveForward(.5,350);


        Thread.sleep(300);
        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
        Thread.sleep(300);

        moveBackward(.5,130);

        robot.getFourBar1().setPosition(0.25);
        robot.getFourBar2().setPosition(0.75);

        robot.getGrabber().setPosition(robot.getGrabberClosePos());
        turnToPID(92);
        Thread.sleep(100);

        robot.getArm1().setPosition(robot.getArm1StackPos4());
        robot.getArm2().setPosition(robot.getArm2StackPos4());

        Thread.sleep(100);


        robot.getTwister().setPosition(robot.getTwisterDownPos());
        fourBarFront();
        Thread.sleep(300);
        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
        moveForward(.5, 500);

        while (robot.getConeDetector().getDistance(DistanceUnit.CM) > 8) {
            moveForward(0.25);
        }
        stopMotors();
//
//
        robot.getGrabber().setPosition(robot.getGrabberClosePos());
        Thread.sleep(300);

        robot.getFourBar1().setPosition(0.45);
        robot.getFourBar2().setPosition(0.55);
        Thread.sleep(300);

        servoArmsHigh();

        fourBarFront();
        moveBackward(.5,350);
        robot.getSlide1().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getSlide2().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.getSlide1().setTargetPosition(-600);
        robot.getSlide2().setTargetPosition(600);

        robot.getSlide1().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getSlide2().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.getSlide1().setPower(-0.6);
        robot.getSlide2().setPower(0.6);

        while (opModeIsActive() && Math.abs(175 - getAbsoluteAngle()) > 1) {

            fieldCentricMove(-.05, 0.41, -0.27);

        }
        stopMotors();

        moveForward(.5,600);


        Thread.sleep(300);
        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
        Thread.sleep(600);

        moveBackward(.5,200);

        robot.getFourBar1().setPosition(0.25);
        robot.getFourBar2().setPosition(0.75);

        robot.getSlide1().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getSlide2().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.getSlide1().setPower(0.1);
        robot.getSlide2().setPower(-0.1);

        robot.getGrabber().setPosition(robot.getGrabberClosePos());



        while (opModeIsActive() && Math.abs(115 - getAbsoluteAngle()) > 1) {

            fieldCentricMove(0, -0.51, 0.3);

        }
        stopMotors();

        robot.getGrabber().setPosition(robot.getGrabberClosePos());
        turnToPID(92);
        Thread.sleep(100);

        robot.getArm1().setPosition(robot.getArm1StackPos3());
        robot.getArm2().setPosition(robot.getArm2StackPos3());

        Thread.sleep(100);


        robot.getTwister().setPosition(robot.getTwisterDownPos());
        fourBarFront();
        Thread.sleep(300);
        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
        moveForward(.5, 500);

        while (robot.getConeDetector().getDistance(DistanceUnit.CM) > 8) {
            moveForward(0.15);
        }
        stopMotors();

        robot.getGrabber().setPosition(robot.getGrabberClosePos());
        Thread.sleep(300);

        robot.getFourBar1().setPosition(0.55);
        robot.getFourBar2().setPosition(0.45);
        Thread.sleep(300);

        servoArmsHigh();

        fourBarFront();
        moveBackward(.5,350);


    }
}



