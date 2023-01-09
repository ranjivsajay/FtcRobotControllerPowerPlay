package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.toolkit.vision.ConeAllignment;
import org.firstinspires.ftc.teamcode.Core.toolkit.vision.PowerPlay;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "meet5Left", group = "Opmodes")
public class meet5Left extends UpliftAutoImpl
{
    @Override
    public void initAction()
    {

        robot.getTwister().setPosition(robot.getTwisterDownPos());
        robot.getGrabber().setPosition(robot.getGrabberClosePos());

        robot.getFourBar1().setPosition(.28);
        robot.getFourBar2().setPosition(.72);


    }

    @Override
    public void body() throws InterruptedException
    {
//        int parkLocation = robot.pipeline1.location;
//        robot.getWebcam().setPipeline(robot.pipeline2);
//
//
//        moveRight(0.4, 230);
//        moveForwardUp(0.75, 0.4, 2150, 1000);
//        Thread.sleep(200);
//
//        turnPID(120);
//
////        robot aligns itself with the pole
//        while(robot.getPoleDetector().getDistance(DistanceUnit.CM) > 40)
//        {
//            robot.getRightFront().setPower(0.25);
//            robot.getRightBack().setPower(0.25);
//            robot.getLeftFront().setPower(-0.25);
//            robot.getLeftBack().setPower(-0.25);
//        }
//        stopMotors();
//
//        moveBackward(0.2, 550);
//
//        stopMotors();
//        Thread.sleep(50);
//
//        robot.getGrabber().setPosition(robot.getGrabberOpenPos());
//        Thread.sleep(100);
//
//        moveForward(0.35, 300 );
//        turnToPID(94);
//
//        robot.getSlide1().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.getSlide2().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        robot.getSlide1().setPower(0.4);
//        robot.getSlide2().setPower(-0.4);
//        Thread.sleep(750);
//
//        if(robot.pipeline2.leftValue > robot.pipeline2.rightValue)
//        {
//            while(robot.pipeline2.leftValue > robot.pipeline2.rightValue)
//            {
//                moveLeft(0.2);
//            }
//            if(robot.pipeline2.rightValue > robot.pipeline2.leftValue)
//            {
//                while(robot.pipeline2.rightValue > robot.pipeline2.leftValue)
//                {
//                    moveRight(0.25);
//                }
//
//            }
//
//        }
//
//        else if(robot.pipeline2.rightValue > robot.pipeline2.leftValue)
//        {
//            while(robot.pipeline2.rightValue > robot.pipeline2.leftValue)
//            {
//                moveRight(0.2);
//            }
//
//        }

        robot.getLineDetector().enableLed(true);
        while(robot.getLineDetector().blue() <= 70)
        {
            moveForward(0.4);
        }

        robot.getLineDetector().enableLed(false);
        stopMotors();

        robot.getArm1().setPosition(robot.getArm1StackPos5());
        robot.getArm2().setPosition(robot.getArm2StackPos5());

        fourBarFront();
        robot.getTwister().setPosition(robot.getTwisterDownPos());


//        while(robot.getConeDetector().getDistance(DistanceUnit.CM) > 0.2)
//        {
//            moveForward(0.3);
//        }
//        stopMotors();
//        robot.getGrabber().setPosition(robot.getGrabberClosePos());
//
//        moveBackwardHigh(0.7,0.5, 1500, 1000);
//
//        turnPID(140);




    }

}
