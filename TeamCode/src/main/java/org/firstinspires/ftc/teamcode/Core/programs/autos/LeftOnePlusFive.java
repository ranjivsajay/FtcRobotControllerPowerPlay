package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;

@Autonomous(name = "Left 1 + 5", group = "Opmodes")
public class LeftOnePlusFive extends UpliftAutoImpl {

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

        robot.getSlide1().setTargetPosition(-950);
        robot.getSlide2().setTargetPosition(950);

        robot.getSlide1().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getSlide2().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.getSlide1().setPower(-0.6);
        robot.getSlide2().setPower(0.6);


        while (opModeIsActive() && Math.abs(147 - getAbsoluteAngle()) > 1) {

            fieldCentricMove(0.44, 0.15, -0.2);

        }
        stopMotors();

        moveBackward(.7,150);
        robot.getFourBar1().setPosition(.2);
        robot.getFourBar2().setPosition(.8);
        Thread.sleep(300);

        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
        Thread.sleep(200);



        robot.getSlide1().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getSlide2().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.getSlide1().setPower(0.1);
        robot.getSlide2().setPower(-0.1);

        while (opModeIsActive() && Math.abs(105 - getAbsoluteAngle()) > 1) {

            fieldCentricMove(-0.28, -0.54, 0.29);

        }
        stopMotors();

        turnToPID(92);

        robot.getGrabber().setPosition(robot.getGrabberClosePos());

        robot.getArm1().setPosition(robot.getArm1StackPos5());
        robot.getArm2().setPosition(robot.getArm2StackPos5());

        Thread.sleep(100);

        robot.getGrabber().setPosition(robot.getGrabberOpenPos());

        robot.getTwister().setPosition(robot.getTwisterDownPos());
        fourBarFront();
        turnToPID(92);
        Thread.sleep(100);

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

        while (opModeIsActive() && Math.abs(135 - getAbsoluteAngle()) > 1) {

            fieldCentricMove(-0.07, 0.6, -0.2);

        }
        stopMotors();

        moveBackward(.7,300);
        robot.getFourBar1().setPosition(.1);
        robot.getFourBar2().setPosition(.9);
        Thread.sleep(300);

        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
        Thread.sleep(200);



        robot.getSlide1().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getSlide2().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.getSlide1().setPower(0.1);
        robot.getSlide2().setPower(-0.1);

        while (opModeIsActive() && Math.abs(105 - getAbsoluteAngle()) > 1) {

            fieldCentricMove(-0.28, -0.54, 0.29);

        }
        stopMotors();

        turnToPID(92);

        robot.getGrabber().setPosition(robot.getGrabberClosePos());

        robot.getArm1().setPosition(robot.getArm1StackPos4());
        robot.getArm2().setPosition(robot.getArm2StackPos4());

        Thread.sleep(100);

        robot.getGrabber().setPosition(robot.getGrabberOpenPos());

        robot.getTwister().setPosition(robot.getTwisterDownPos());
        fourBarFront();
        turnToPID(92);
        Thread.sleep(100);

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

        while (opModeIsActive() && Math.abs(135 - getAbsoluteAngle()) > 1) {

            fieldCentricMove(-0.07, 0.6, -0.2);

        }
        stopMotors();

        moveBackward(.7,300);
        robot.getFourBar1().setPosition(.1);
        robot.getFourBar2().setPosition(.9);
        Thread.sleep(300);

        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
        Thread.sleep(200);


//
//        robot.getSlide1().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.getSlide2().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        robot.getSlide1().setPower(0.1);
//        robot.getSlide2().setPower(-0.1);
//
//        while (opModeIsActive() && Math.abs(105 - getAbsoluteAngle()) > 1) {
//
//            fieldCentricMove(-0.28, -0.54, 0.29);
//
//        }
//        stopMotors();
//
//        turnToPID(92);
//
//        robot.getGrabber().setPosition(robot.getGrabberClosePos());
//
//        robot.getArm1().setPosition(robot.getArm1StackPos3());
//        robot.getArm2().setPosition(robot.getArm2StackPos3());
//
//        Thread.sleep(100);
//
//        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
//
//        robot.getTwister().setPosition(robot.getTwisterDownPos());
//        fourBarFront();
//        turnToPID(92);
//        Thread.sleep(100);
//
//        while (robot.getConeDetector().getDistance(DistanceUnit.CM) > 8) {
//            moveForward(0.25);
//        }
//        stopMotors();
////
////
//        robot.getGrabber().setPosition(robot.getGrabberClosePos());
//        Thread.sleep(300);
//
//        robot.getFourBar1().setPosition(0.4);
//        robot.getFourBar2().setPosition(0.6);
//        Thread.sleep(200);
//
//        servoArmsHigh();
//
//        fourBarBack();
//        robot.getTwister().setPosition(robot.getTwisterUpPos());
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
//        while (opModeIsActive() && Math.abs(135 - getAbsoluteAngle()) > 1) {
//
//            fieldCentricMove(-0.07, 0.6, -0.2);
//
//        }
//        stopMotors();
//
//        moveBackward(.7,300);
//        robot.getFourBar1().setPosition(.1);
//        robot.getFourBar2().setPosition(.9);
//        Thread.sleep(300);
//
//        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
//        Thread.sleep(200);
//
//
//
//        robot.getSlide1().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.getSlide2().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        robot.getSlide1().setPower(0.1);
//        robot.getSlide2().setPower(-0.1);
//
//        while (opModeIsActive() && Math.abs(105 - getAbsoluteAngle()) > 1) {
//
//            fieldCentricMove(-0.28, -0.54, 0.29);
//
//        }
//        stopMotors();
//
//        turnToPID(92);
//
//        robot.getGrabber().setPosition(robot.getGrabberClosePos());
//
//        robot.getArm1().setPosition(robot.getArm1StackPos2());
//        robot.getArm2().setPosition(robot.getArm2StackPos2());
//
//        Thread.sleep(100);
//
//        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
//
//        robot.getTwister().setPosition(robot.getTwisterDownPos());
//        fourBarFront();
//        turnToPID(92);
//        Thread.sleep(100);
//
//        while (robot.getConeDetector().getDistance(DistanceUnit.CM) > 8) {
//            moveForward(0.25);
//        }
//        stopMotors();
////
////
//        robot.getGrabber().setPosition(robot.getGrabberClosePos());
//        Thread.sleep(300);
//
//        robot.getFourBar1().setPosition(0.4);
//        robot.getFourBar2().setPosition(0.6);
//        Thread.sleep(200);
//
//        servoArmsHigh();
//
//        fourBarBack();
//        robot.getTwister().setPosition(robot.getTwisterUpPos());
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
//        while (opModeIsActive() && Math.abs(135 - getAbsoluteAngle()) > 1) {
//
//            fieldCentricMove(-0.07, 0.6, -0.2);
//
//        }
//        stopMotors();
//
//        moveBackward(.7,300);
//        robot.getFourBar1().setPosition(.1);
//        robot.getFourBar2().setPosition(.9);
//        Thread.sleep(300);
//
//        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
//        Thread.sleep(200);
//
//
//
//        robot.getSlide1().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.getSlide2().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        robot.getSlide1().setPower(0.1);
//        robot.getSlide2().setPower(-0.1);
//
//        while (opModeIsActive() && Math.abs(105 - getAbsoluteAngle()) > 1) {
//
//            fieldCentricMove(-0.28, -0.54, 0.29);
//
//        }
//        stopMotors();
//
//        turnToPID(92);
//
//        robot.getGrabber().setPosition(robot.getGrabberClosePos());
//
//        robot.getArm1().setPosition(robot.getArm1LowPos());
//        robot.getArm2().setPosition(robot.getArm2LowPos());
//
//        Thread.sleep(100);
//
//        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
//
//        robot.getTwister().setPosition(robot.getTwisterDownPos());
//        fourBarFront();
//        turnToPID(92);
//        Thread.sleep(100);







    }
}



