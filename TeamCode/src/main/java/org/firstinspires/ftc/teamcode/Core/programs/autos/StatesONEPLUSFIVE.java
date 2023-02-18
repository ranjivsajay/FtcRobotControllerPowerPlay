package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;

@Autonomous(name = "1 + 5", group = "Opmodes")
public class StatesONEPLUSFIVE extends UpliftAutoImpl {
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

        while(opModeIsActive() && Math.abs(145 - getAbsoluteAngle()) > 1)
        {

            fieldCentricMove(0.4, 0.17, -0.3);

        }
        stopMotors();

        moveBackwardHigh(0.2, 0.9, 450, 900);

        robot.getFourBar1().setPosition(.1);
        robot.getFourBar2().setPosition(.9);
        Thread.sleep(300);

        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
        Thread.sleep(200);

        while(opModeIsActive() && Math.abs(105 - getAbsoluteAngle()) > 1)
        {

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

        while(robot.getConeDetector().getDistance(DistanceUnit.CM) > 8)
        {
            moveForward(0.3);
        }
        stopMotors();
//
//
        robot.getGrabber().setPosition(robot.getGrabberClosePos());
        Thread.sleep(500);

        robot.getFourBar1().setPosition( 0.4);
        robot.getFourBar2().setPosition( 0.6);
        Thread.sleep(500);

        while(opModeIsActive() && Math.abs(135 - getAbsoluteAngle()) > 1)
        {

            fieldCentricMove(-0.07, 0.65, -0.2);

        }
        stopMotors();

        moveBackwardHigh(0.2, 0.9, 350, 1000);

        robot.getFourBar1().setPosition(.1);
        robot.getFourBar2().setPosition(.9);
        Thread.sleep(300);

        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
        Thread.sleep(200);

        while(opModeIsActive() && Math.abs(105 - getAbsoluteAngle()) > 1)
        {

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

        while(robot.getConeDetector().getDistance(DistanceUnit.CM) > 8)
        {
            moveForward(0.3);
        }
        stopMotors();
//
//
        robot.getGrabber().setPosition(robot.getGrabberClosePos());
        Thread.sleep(500);

        robot.getFourBar1().setPosition( 0.4);
        robot.getFourBar2().setPosition( 0.6);
        Thread.sleep(500);

        while(opModeIsActive() && Math.abs(135 - getAbsoluteAngle()) > 1)
        {

            fieldCentricMove(-0.07, 0.65, -0.2);

        }
        stopMotors();

        moveBackwardHigh(0.2, 0.9, 350, 1100);

        robot.getFourBar1().setPosition(.1);
        robot.getFourBar2().setPosition(.9);
        Thread.sleep(300);

        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
        Thread.sleep(200);

        while(opModeIsActive() && Math.abs(105 - getAbsoluteAngle()) > 1)
        {

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

        while(robot.getConeDetector().getDistance(DistanceUnit.CM) > 8)
        {
            moveForward(0.3);
        }
        stopMotors();
//
//
        robot.getGrabber().setPosition(robot.getGrabberClosePos());
        Thread.sleep(500);

        robot.getFourBar1().setPosition( 0.4);
        robot.getFourBar2().setPosition( 0.6);
        Thread.sleep(500);

        while(opModeIsActive() && Math.abs(135 - getAbsoluteAngle()) > 1)
        {

            fieldCentricMove(-0.07, 0.68, -0.2);

        }
        stopMotors();

        moveBackwardHigh(0.2, 0.9, 350, 1000);

        robot.getFourBar1().setPosition(.1);
        robot.getFourBar2().setPosition(.9);
        Thread.sleep(300);

        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
        Thread.sleep(200);

        while(opModeIsActive() && Math.abs(105 - getAbsoluteAngle()) > 1)
        {

            fieldCentricMove(-0.3, -0.54, 0.29);

        }
        stopMotors();






//        moveForward(0.2, 370);
//        robot.getGrabber().setPosition(robot.getGrabberClosePos());
//
//        turnToPID(91);
//
//        robot.getSlide1().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.getSlide2().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        robot.getSlide1().setPower(0.3);
//        robot.getSlide2().setPower(-0.3);
//        Thread.sleep(600);
//        moveLeft(.6,50);
//
//        //robot aligns itself with the stack of cones
//        if(robot.pipeline2.leftValue > robot.pipeline2.rightValue)
//        {
//            while(robot.pipeline2.leftValue > robot.pipeline2.rightValue)
//            {
//                moveLeft(0.3);
//            }
//            if(robot.pipeline2.rightValue > robot.pipeline2.leftValue)
//            {
//                while(robot.pipeline2.rightValue > robot.pipeline2.leftValue)
//                {
//                    moveRight(0.3);
//                }
//
//            }
//
//        }
//        else if(robot.pipeline2.rightValue > robot.pipeline2.leftValue)
//        {
//            while(robot.pipeline2.rightValue > robot.pipeline2.leftValue)
//            {
//                moveRight(0.25);
//            }
//            if(robot.pipeline2.leftValue > robot.pipeline2.rightValue)
//            {
//                while(robot.pipeline2.leftValue > robot.pipeline2.rightValue)
//                {
//                    moveLeft(0.3);
//                }
//                if(robot.pipeline2.rightValue > robot.pipeline2.leftValue + 20)
//                {
//                    while(robot.pipeline2.rightValue > robot.pipeline2.leftValue + 20)
//                    {
//                        moveRight(0.3);
//                    }
//
//                }
//
//            }
//
//        }
//        turnToPID(91);
//        stopMotors();
//        robot.getWebcam().setPipeline(robot.pipeline1);
//
//        robot.getArm1().setPosition(robot.getArm1StackPos5());
//        robot.getArm2().setPosition(robot.getArm2StackPos5());
//        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
//
//        robot.getTwister().setPosition(robot.getTwisterDownPos());
//        fourBarFront();
//        turnToPID(91);
//        Thread.sleep(500);
//        moveForward(.5, 1000);
//        while(robot.getConeDetector().getDistance(DistanceUnit.CM) > 8)
//        {
//            moveForward(0.3);
//        }
//        stopMotors();
////
////
//        robot.getGrabber().setPosition(robot.getGrabberClosePos());
//        Thread.sleep(500);
//
//        robot.getFourBar1().setPosition( 0.4);
//        robot.getFourBar2().setPosition( 0.6);
//        Thread.sleep(500);
//
//        while(opModeIsActive() && Math.abs(135 - getAbsoluteAngle()) > 1)
//        {
//
//            fieldCentricMove(-0.07, 0.54, -0.2);
//
//        }
//        stopMotors();
//
//        servoArmsHigh();
//
//        robot.getSlide1().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.getSlide2().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        robot.getSlide1().setTargetPosition(-1000);
//        robot.getSlide2().setTargetPosition(1000);
//
//        robot.getSlide1().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.getSlide2().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.getSlide1().setPower(-0.6);
//        robot.getSlide2().setPower(0.6);
//
//        while (opModeIsActive() && robot.getSlide1().isBusy() && robot.getSlide2().isBusy()) {
//
//        }
//
//        stopMotors();
//
//        fourBarBack();
//        robot.getTwister().setPosition(robot.getTwisterUpPos());
//
//
//        moveBackward(0.7, 300);
//        Thread.sleep(300);
//
//        robot.getFourBar1().setPosition(.1);
//        robot.getFourBar2().setPosition(.9);
//        Thread.sleep(300);
//
//        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
//        Thread.sleep(200);

    }
}
