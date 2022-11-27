//package org.firstinspires.ftc.teamcode.Core.Threads;
//
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
//
//public class OperatorThread implements Runnable
//{
//
//
//    private UpliftRobot robot;
//
//    private boolean grabberState;
//    private boolean blockGrabberInput;
//
//    double arm1HighPos;
//    double arm2HighPos;
//
//
//    public OperatorThread(UpliftRobot robot)
//    {
//        this.robot = robot;
//
//        this.grabberState = true;
//        this.blockGrabberInput = false;
//
//
//        this.arm1HighPos = .4;
//        this.arm2HighPos = .0;
//    }
//
//    @Override
//    public void run()
//    {
//        armHigh();
//
//        holdSlidePos();
//
//        robot.getSlide1().setPower(Range.clip(gamepad2.right_stick_y, -1, 1));
//        robot.getSlide2().setPower(Range.clip(-gamepad2.right_stick_y, -1, 1));
//
//        try {
//            grab();
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
//
//    }
//
//    public void slides(double power, double dist)
//    {
//
//        double initialPos2 = robot.getSlide2().getCurrentPosition();
//
//        while (Math.abs(robot.getSlide2().getCurrentPosition() - (initialPos2 + dist)) > 50)
//        {
//            robot.getSlide1().setPower(-power);
//            robot.getSlide2().setPower(power);
//
//        }
//        robot.getSlide1().setPower(0);
//        robot.getSlide2().setPower(0);
//
//
//    }
//
//
//    public void grab() throws InterruptedException {
//        if(gamepad2.right_trigger > robot.getGrabberClosePos() && !blockGrabberInput)
//        {
//            robot.getGrabber().setPosition(grabberState ? robot.getGrabberClosePos() : robot.getGrabberOpenPos());
//            grabberState = !grabberState;
//            blockGrabberInput = true;
//        }
//        else if (gamepad2.right_trigger < robot.getGrabberOpenPos() && blockGrabberInput)
//        {
//            blockGrabberInput = false;
//        }
//    }
//
//    public void armHigh()
//    {
//        if(gamepad2.dpad_up)
//        {
//
//            robot.getArm2().setPosition(arm2HighPos);
//            robot.getArm1().setPosition(arm1HighPos);
//
//            slides(0.5, 953);
//
//
//        }
//    }
//
//    public void holdSlidePos()
//    {
//        if(gamepad2.left_trigger > 0)
//        {
//            robot.getSlide1().setPower(-0.1);
//            robot.getSlide2().setPower(0.1);
//        }
//    }
//
//
//}
