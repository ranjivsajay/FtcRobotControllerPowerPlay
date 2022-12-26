//package org.firstinspires.ftc.teamcode.Core.programs.autos;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
//
//@Autonomous(name = "meet4Auto", group = "Opmodes")
//public class Meet4Auto extends UpliftAutoImpl {
//    @Override
//    public void initAction() {
//
//        robot.getGrabber().setPosition(robot.getGrabberClosePos());
//        robot.getFourBar1().setPosition(.28);
//        robot.getFourBar2().setPosition(.72);
//
//
//    }
//
//
//    public void body() throws InterruptedException {int location = robot.pipeline.location;
//    moveRight(0.5, 200);
//    moveForward(0.8, 2820);
//    high();
//    robot.getSlide1().setPower(-0.4);
//    robot.getSlide2().setPower(0.4);
//    Thread.sleep(500);
//
//    robot.getTwister().setPosition(robot.getTwisterUpPos());
//
//
//    turnLeft(.5, 93);
//    robot.getTwister().setPosition(robot.getTwisterUpPos());
//    Thread.sleep(400);
//
//    moveBackward(0.4, 170);
//    Thread.sleep(500);
//
//    robot.getGrabber().setPosition(robot.getGrabberOpenPos());
//    Thread.sleep(500);
//
//    fourBarFront();
//    Thread.sleep(750);
//    robot.getTwister().setPosition(robot.getTwisterDownPos());
//    robot.getSlide1().setPower(0.4);
//    robot.getSlide2().setPower(-0.4);
//    robot.getGrabber().setPosition(robot.getGrabberOpenPos());
//
//    Thread.sleep(750);
//
//    moveForward(0.5, 300);
//    moveBackward(.5, 210);
//    moveForward(.6, 200);
//
//
//    moveLeft(.5, 600);
//    Thread.sleep(250);
//
//    robot.getArm1().setPosition(.52);
//    robot.getArm2().setPosition(.48);
//
//    Thread.sleep(250);
////    turnLeft(.2, 9);
//    moveForward(.4, 930);
//
//    robot.getGrabber().setPosition(robot.getGrabberClosePos());
//    Thread.sleep(500);
//
//    servoArmsHigh();
//    Thread.sleep(250);
//    moveBackward(.5, 950);
//
//    turnLeft(.4, 62);
//    high();
//    robot.getSlide1().setPower(-0.4);
//    robot.getSlide2().setPower(0.4);
//    robot.getTwister().setPosition(robot.getTwisterUpPos());
//    Thread.sleep(1700);
//
//    moveBackward(0.4, 350);
//    Thread.sleep(500);
//
//    robot.getGrabber().setPosition(robot.getGrabberOpenPos());
//    Thread.sleep(500);
//
//    moveForward(.4, 330);
//    turnRight(.4, 58);
//
//    fourBarFront();
//    Thread.sleep(750);
//    robot.getTwister().setPosition(robot.getTwisterDownPos());
//    robot.getSlide1().setPower(0.4);
//    robot.getSlide2().setPower(-0.4);
//    robot.getGrabber().setPosition(robot.getGrabberOpenPos());
//
//    Thread.sleep(750);
//
//    robot.getArm1().setPosition(.49);
//    robot.getArm2().setPosition(.51);
//
//    Thread.sleep(200);
//
//    moveForward(.5, 1100);
//
//    robot.getGrabber().setPosition(robot.getGrabberClosePos());
//    Thread.sleep(500);
//
//
//    servoArmsHigh();
//    Thread.sleep(250);
//    moveBackward(.5, 850);
//
//    turnLeft(.4, 54.5);
//    high();
//    robot.getSlide1().setPower(-0.4);
//    robot.getSlide2().setPower(0.4);
//    robot.getTwister().setPosition(robot.getTwisterUpPos());
//    Thread.sleep(1700);
//
//    moveBackward(0.55, 410);
//    Thread.sleep(500);
//
//    robot.getGrabber().setPosition(robot.getGrabberOpenPos());
//    Thread.sleep(250);
//
//    moveForward(.8, 410);
//    turnRight(.8, 48);
//    Thread.sleep(300);
//
//
//    if(location == 1)
//    {
//        moveForward(1, 1050);
//
//    }
//
//    else if(location == 2)
//    {
//
//    }
//
//    else if(location == 3)
//    {
//        moveBackward(1, 550);
//
//    }
//    }
//}
