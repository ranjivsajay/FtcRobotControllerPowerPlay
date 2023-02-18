package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;

@Autonomous(name = "BlueleftCycle", group = "Opmodes")
public class BlueLeftCycle extends UpliftAutoImpl {
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

        while(opModeIsActive() && Math.abs(146 - getAbsoluteAngle()) > 1)
        {

            fieldCentricMove(0.4, 0.13, -0.3);

        }
        stopMotors();


        servoArmsHigh();

        robot.getSlide1().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getSlide2().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.getSlide1().setTargetPosition(-900);
        robot.getSlide2().setTargetPosition(900);

        robot.getSlide1().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getSlide2().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.getSlide1().setPower(-0.6);
        robot.getSlide2().setPower(0.6);

        while (opModeIsActive() && robot.getSlide1().isBusy() && robot.getSlide2().isBusy()) {

        }

        stopMotors();

        fourBarBack();
        robot.getTwister().setPosition(robot.getTwisterUpPos());


        moveBackward(0.4, 420);
        Thread.sleep(500);

        robot.getFourBar1().setPosition(.1);
        robot.getFourBar2().setPosition(.9);
        Thread.sleep(300);

        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
        Thread.sleep(200);



        moveForward(0.2, 370);
        robot.getGrabber().setPosition(robot.getGrabberClosePos());

        turnToPID(92);

        robot.getSlide1().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getSlide2().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.getSlide1().setPower(0.3);
        robot.getSlide2().setPower(-0.3);
        Thread.sleep(600);
        moveLeft(.6,50);

        //robot aligns itself with the stack of cones
        if(robot.pipeline2.leftValue > robot.pipeline2.rightValue)
        {
            while(robot.pipeline2.leftValue > robot.pipeline2.rightValue)
            {
                moveLeft(0.3);
            }
            if(robot.pipeline2.rightValue > robot.pipeline2.leftValue)
            {
                while(robot.pipeline2.rightValue > robot.pipeline2.leftValue)
                {
                    moveRight(0.3);
                }

            }

        }
        else if(robot.pipeline2.rightValue > robot.pipeline2.leftValue)
        {
            while(robot.pipeline2.rightValue > robot.pipeline2.leftValue)
            {
                moveRight(0.25);
            }
            if(robot.pipeline2.leftValue > robot.pipeline2.rightValue)
            {
                while(robot.pipeline2.leftValue > robot.pipeline2.rightValue)
                {
                    moveLeft(0.3);
                }
                if(robot.pipeline2.rightValue > robot.pipeline2.leftValue + 20)
                {
                    while(robot.pipeline2.rightValue > robot.pipeline2.leftValue + 20)
                    {
                        moveRight(0.3);
                    }

                }

            }

        }
        turnToPID(92);
        stopMotors();


        robot.getArm1().setPosition(robot.getArm1StackPos5());
        robot.getArm2().setPosition(robot.getArm2StackPos5());
        robot.getGrabber().setPosition(robot.getGrabberOpenPos());

        robot.getTwister().setPosition(robot.getTwisterDownPos());
        fourBarFront();
        turnToPID(92);
        Thread.sleep(500);
        moveForward(.5, 1050);
//        while(robot.getConeDetector().getDistance(DistanceUnit.CM) > 5)
//        {
//            moveForward(0.3);
//        }
//        stopMotors();
//
//
        robot.getGrabber().setPosition(robot.getGrabberClosePos());
        Thread.sleep(500);

        robot.getFourBar1().setPosition( 0.4);
        robot.getFourBar2().setPosition( 0.6);
        Thread.sleep(500);

        while(opModeIsActive() && Math.abs(135 - getAbsoluteAngle()) > 1)
        {

            fieldCentricMove(-0.07, 0.54, -0.17);

        }
        stopMotors();

        servoArmsHigh();

        robot.getSlide1().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getSlide2().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.getSlide1().setTargetPosition(-1000);
        robot.getSlide2().setTargetPosition(1000);

        robot.getSlide1().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getSlide2().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.getSlide1().setPower(-0.6);
        robot.getSlide2().setPower(0.6);

        while (opModeIsActive() && robot.getSlide1().isBusy() && robot.getSlide2().isBusy()) {

        }

        stopMotors();

        robot.getFourBar1().setPosition(.15);
        robot.getFourBar2().setPosition(.85);
        robot.getTwister().setPosition(robot.getTwisterUpPos());


        moveBackward(0.5, 400);
        Thread.sleep(500);

//        robot.getFourBar1().setPosition(.1);
//        robot.getFourBar2().setPosition(.9);
//        Thread.sleep(300);

        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
        Thread.sleep(200);


      // third cone


        moveForward(0.2, 370);
        robot.getGrabber().setPosition(robot.getGrabberClosePos());

        turnToPID(92);

        robot.getSlide1().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getSlide2().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.getSlide1().setPower(0.3);
        robot.getSlide2().setPower(-0.3);
        Thread.sleep(600);
        moveLeft(.6,50);

        //robot aligns itself with the stack of cones
        if(robot.pipeline2.leftValue > robot.pipeline2.rightValue)
        {
            while(robot.pipeline2.leftValue > robot.pipeline2.rightValue)
            {
                moveLeft(0.3);
            }
            if(robot.pipeline2.rightValue > robot.pipeline2.leftValue)
            {
                while(robot.pipeline2.rightValue > robot.pipeline2.leftValue)
                {
                    moveRight(0.3);
                }

            }

        }
        else if(robot.pipeline2.rightValue > robot.pipeline2.leftValue)
        {
            while(robot.pipeline2.rightValue > robot.pipeline2.leftValue)
            {
                moveRight(0.25);
            }
            if(robot.pipeline2.leftValue > robot.pipeline2.rightValue)
            {
                while(robot.pipeline2.leftValue > robot.pipeline2.rightValue)
                {
                    moveLeft(0.3);
                }
                if(robot.pipeline2.rightValue > robot.pipeline2.leftValue + 20)
                {
                    while(robot.pipeline2.rightValue > robot.pipeline2.leftValue + 20)
                    {
                        moveRight(0.3);
                    }

                }

            }

        }
        turnToPID(92);
        stopMotors();


        robot.getArm1().setPosition(robot.getArm1StackPos4());
        robot.getArm2().setPosition(robot.getArm2StackPos4());
        robot.getGrabber().setPosition(robot.getGrabberOpenPos());

        robot.getTwister().setPosition(robot.getTwisterDownPos());
        fourBarFront();
        turnToPID(92);
        Thread.sleep(500);
        moveForward(.5, 1030);
//        while(robot.getConeDetector().getDistance(DistanceUnit.CM) > 5)
//        {
//            moveForward(0.3);
//        }
//        stopMotors();
//
//
        robot.getGrabber().setPosition(robot.getGrabberClosePos());
        Thread.sleep(500);

        robot.getFourBar1().setPosition( 0.4);
        robot.getFourBar2().setPosition( 0.6);
        Thread.sleep(500);

        while(opModeIsActive() && Math.abs(135 - getAbsoluteAngle()) > 1)
        {

            fieldCentricMove(-0.07, 0.54, -0.17);

        }
        stopMotors();

        servoArmsHigh();

        robot.getSlide1().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getSlide2().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.getSlide1().setTargetPosition(-1000);
        robot.getSlide2().setTargetPosition(1000);

        robot.getSlide1().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getSlide2().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.getSlide1().setPower(-0.6);
        robot.getSlide2().setPower(0.6);

        while (opModeIsActive() && robot.getSlide1().isBusy() && robot.getSlide2().isBusy()) {

        }

        stopMotors();

        robot.getFourBar1().setPosition(.15);
        robot.getFourBar2().setPosition(.85);
        robot.getTwister().setPosition(robot.getTwisterUpPos());


        moveBackward(0.5, 400);
        Thread.sleep(500);

//        robot.getFourBar1().setPosition(.1);
//        robot.getFourBar2().setPosition(.9);
//        Thread.sleep(300);

        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
        Thread.sleep(200);
////
         moveForward(0.2, 370);
        robot.getGrabber().setPosition(robot.getGrabberClosePos());

        turnToPID(92);

        robot.getSlide1().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getSlide2().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.getSlide1().setPower(0.3);
        robot.getSlide2().setPower(-0.3);
        Thread.sleep(600);
        moveLeft(.6,50);

        //robot aligns itself with the stack of cones
        if(robot.pipeline2.leftValue > robot.pipeline2.rightValue)
        {
            while(robot.pipeline2.leftValue > robot.pipeline2.rightValue)
            {
                moveLeft(0.3);
            }
            if(robot.pipeline2.rightValue > robot.pipeline2.leftValue)
            {
                while(robot.pipeline2.rightValue > robot.pipeline2.leftValue)
                {
                    moveRight(0.3);
                }

            }

        }
        else if(robot.pipeline2.rightValue > robot.pipeline2.leftValue)
        {
            while(robot.pipeline2.rightValue > robot.pipeline2.leftValue)
            {
                moveRight(0.25);
            }
            if(robot.pipeline2.leftValue > robot.pipeline2.rightValue)
            {
                while(robot.pipeline2.leftValue > robot.pipeline2.rightValue)
                {
                    moveLeft(0.3);
                }
                if(robot.pipeline2.rightValue > robot.pipeline2.leftValue + 20)
                {
                    while(robot.pipeline2.rightValue > robot.pipeline2.leftValue + 20)
                    {
                        moveRight(0.3);
                    }

                }

            }

        }
        turnToPID(92);
        stopMotors();


        robot.getArm1().setPosition(robot.getArm1StackPos3());
        robot.getArm2().setPosition(robot.getArm2StackPos3());
        robot.getGrabber().setPosition(robot.getGrabberOpenPos());

        robot.getTwister().setPosition(robot.getTwisterDownPos());
        fourBarFront();
        turnToPID(92);
        Thread.sleep(500);
        moveForward(.5, 1030);
//        while(robot.getConeDetector().getDistance(DistanceUnit.CM) > 5)
//        {
//            moveForward(0.3);
//        }
//        stopMotors();
//
//
        robot.getGrabber().setPosition(robot.getGrabberClosePos());
        Thread.sleep(500);

        robot.getFourBar1().setPosition( 0.4);
        robot.getFourBar2().setPosition( 0.6);
        Thread.sleep(500);

        while(opModeIsActive() && Math.abs(135 - getAbsoluteAngle()) > 1)
        {

            fieldCentricMove(-0.07, 0.54, -0.17);

        }
        stopMotors();

        servoArmsHigh();

        robot.getSlide1().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getSlide2().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.getSlide1().setTargetPosition(-1000);
        robot.getSlide2().setTargetPosition(1000);

        robot.getSlide1().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getSlide2().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.getSlide1().setPower(-0.6);
        robot.getSlide2().setPower(0.6);

        while (opModeIsActive() && robot.getSlide1().isBusy() && robot.getSlide2().isBusy()) {

        }

        stopMotors();

        robot.getFourBar1().setPosition(.15);
        robot.getFourBar2().setPosition(.85);
        robot.getTwister().setPosition(robot.getTwisterUpPos());


        moveBackward(0.5, 400);
        Thread.sleep(500);

//        robot.getFourBar1().setPosition(.1);
//        robot.getFourBar2().setPosition(.9);
//        Thread.sleep(300);

        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
        Thread.sleep(200);

    }
}

//robot.getWebcam().setPipeline(robot.pipeline1);