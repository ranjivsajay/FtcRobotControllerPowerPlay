package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.toolkit.UpliftMath;
import org.firstinspires.ftc.teamcode.Core.toolkit.vision.ConesPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "LeftMiddle", group = "Opmodes")
public class OdoLeftMiddle extends UpliftAutoImpl {


    @Override
    public void initAction() {

        robot.getFourBar().setPosition(.75);

        robot.getArmLeft().setPosition(robot.getArmLeftLowPos());
        robot.getArmRight().setPosition(robot.getArmRightLowPos());

        robot.getGrabber1().setPosition(robot.getGrabber1ClosePos());
        robot.getTwister().setPosition(robot.getTwisterDownPos());

//        robot.getOdoMid().setPosition(robot.getOdoMidDown());
//        robot.getWebcam().setPipeline(robot.pipeline2);

    }

    @Override
    public void body() throws InterruptedException {


        int parkLocation = robot.pipeline1.location;



        robot.pipeline1.doCones(true);


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Pose2d startPose = new Pose2d(-63, 36, Math.toRadians(0));
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        while (opModeIsActive() && Math.abs(92 - getAbsoluteAngle()) > 1) {

            fieldCentricMove(0.6, 0.18, -0.3);
            servoArmsHigh();
            robot.getTwister().setPosition(robot.getTwisterUpPos());

        }
        stopMotors();

//        Trajectory traj1 = drive.trajectoryBuilder(startPose)
//                .strafeRight(11)
//                .build();
//        drive.followTrajectory(traj1);
        moveRight(0.5, 600);

        turnToPole(300, 0.2);

        while (robot.getPoleDetector().getDistance(DistanceUnit.CM) > 4) {
            moveBackward(0.2);
        }
        stopMotors();

        robot.getFourBar().setPosition(robot.getBarBackPos());
        robot.getGrabber1().setPosition(robot.getGrabber1OpenPos());

        Thread.sleep(500);

//        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(), Math.toRadians(90))
//                .forward(4)
//                .build();
//        drive.followTrajectory(traj2);

        moveForward(0.5, 150);

        moveRight(0.5, 200);



//        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d())
//                .strafeRight(6)
//                .build();
//        drive.followTrajectory(traj3);

        turnToPID(90);

        robot.getFourBar().setPosition(robot.getBarFrontPos());
        robot.getArmLeft().setPosition(robot.getArm1StackPos5());
        robot.getArmRight().setPosition(robot.getArm2StackPos5());
        robot.getGrabber1().setPosition(robot.getGrabber1OpenPos());
        robot.getTwister().setPosition(robot.getTwisterDownPos());

        Thread.sleep(800);
       // robot.getWebcam().setPipeline(robot.pipeline2);


//            if (robot.getConeDetector().getDistance(DistanceUnit.CM) < 54) {
//                drive.followTrajectory(traj2);
//            } else {
//                while (robot.getConeDetector().getDistance(DistanceUnit.CM) > 54) {
//                    moveRight(.2);
//                }
//                stopMotors();
//            }

//        robot aligns itself with the stack of cones
        while(opModeIsActive() && robot.pipeline1.getError() != 0)
        {
            double var = robot.pipeline1.getError() * 0.0052;
            moveRight(-var);

        }
        stopMotors();

        while (robot.getConeDetector().getDistance(DistanceUnit.CM) > 8)
        {
            moveForward(.35);
        }
        stopMotors();






        robot.getGrabber1().setPosition(robot.getGrabber1ClosePos());

        Thread.sleep(300);


        servoArmsHigh();
        robot.getFourBar().setPosition(0.75);
      //  Thread.sleep(300);


     //   robot.getTwister().setPosition(robot.getTwisterUpPos());

        while (opModeIsActive() && Math.abs(69 - getAbsoluteAngle()) > 1) {

            fieldCentricMove(-0.08, 0.92, 0.2);

        }
        stopMotors();

        turnToPole(300, 0.2);
        robot.getTwister().setPosition(robot.getTwisterUpPos());

        robot.pipeline1.doCones(true);
        while (robot.getPoleDetector().getDistance(DistanceUnit.CM) > 4) {
            moveBackward(0.2);
        }
        stopMotors();

        fourBarBack();
        robot.getGrabber1().setPosition(robot.getGrabber1OpenPos());

        while (opModeIsActive() && Math.abs(90 - getAbsoluteAngle()) > 1) {

            fieldCentricMove(0.14, -0.2, -0.2);

        }
        stopMotors();

//        while(opModeIsActive())
//        {
//            telemetry.addData("error", robot.pipeline1.getError());
//            telemetry.update();
//        }

        turnToPID(90);

        robot.getFourBar().setPosition(robot.getBarFrontPos());
        robot.getArmLeft().setPosition(robot.getArm1StackPos4());
        robot.getArmRight().setPosition(robot.getArm2StackPos4());
        robot.getGrabber1().setPosition(robot.getGrabber1OpenPos());
        robot.getTwister().setPosition(robot.getTwisterDownPos());

        Thread.sleep(800);

//        robot aligns itself with the stack of cones
        while(opModeIsActive() && robot.pipeline1.getError() != 0)
        {
            double var = robot.pipeline1.getError() * 0.0052;
            moveRight(-var);

        }
        stopMotors();

        turnToPID(90);


        while (robot.getConeDetector().getDistance(DistanceUnit.CM) > 8)
        {
            moveForward(.35);
        }
        stopMotors();


        robot.getGrabber1().setPosition(robot.getGrabber1ClosePos());

        Thread.sleep(300);


        servoArmsHigh();
        robot.getFourBar().setPosition(0.75);
      //  Thread.sleep(500);


      //  robot.getTwister().setPosition(robot.getTwisterUpPos());

        while (opModeIsActive() && Math.abs(69 - getAbsoluteAngle()) > 1) {

            fieldCentricMove(-0.08, 0.92, 0.2);

        }
        stopMotors();

        turnToPole(300, 0.2);
        robot.getTwister().setPosition(robot.getTwisterUpPos());

        robot.pipeline1.doCones(true);
        while (robot.getPoleDetector().getDistance(DistanceUnit.CM) > 4) {
            moveBackward(0.2);
        }
        stopMotors();


        fourBarBack();
        robot.getGrabber1().setPosition(robot.getGrabber1OpenPos());

        while (opModeIsActive() && Math.abs(90 - getAbsoluteAngle()) > 1) {

            fieldCentricMove(0.15, -0.2, -0.2);

        }
        stopMotors();

        turnToPID(90);

        robot.getFourBar().setPosition(robot.getBarFrontPos());
        robot.getArmLeft().setPosition(robot.getArm1StackPos3());
        robot.getArmRight().setPosition(robot.getArm2StackPos3());
        robot.getGrabber1().setPosition(robot.getGrabber1OpenPos());
        robot.getTwister().setPosition(robot.getTwisterDownPos());

        Thread.sleep(800);

//        robot aligns itself with the stack of cones
        while(opModeIsActive() && robot.pipeline1.getError() != 0)
        {
            double var = robot.pipeline1.getError() * 0.0052;
            moveRight(-var);

        }
        stopMotors();

        turnToPID(90);


        while (robot.getConeDetector().getDistance(DistanceUnit.CM) > 8)
        {
            moveForward(.35);
        }
        stopMotors();


        robot.getGrabber1().setPosition(robot.getGrabber1ClosePos());

        Thread.sleep(500);


        servoArmsHigh();
        robot.getFourBar().setPosition(0.75);
       // Thread.sleep(500);


      //  robot.getTwister().setPosition(robot.getTwisterUpPos());

        while (opModeIsActive() && Math.abs(69 - getAbsoluteAngle()) > 1) {

            fieldCentricMove(-0.08, 0.92, 0.2);

        }
        stopMotors();

        turnToPole(300, 0.2);
        robot.getTwister().setPosition(robot.getTwisterUpPos());
        robot.pipeline1.doCones(true);
        while (robot.getPoleDetector().getDistance(DistanceUnit.CM) > 4) {
            moveBackward(0.2);
        }
        stopMotors();

        fourBarBack();
        robot.getGrabber1().setPosition(robot.getGrabber1OpenPos());

        Thread.sleep(200);

        while (opModeIsActive() && Math.abs(90 - getAbsoluteAngle()) > 1) {

            fieldCentricMove(0.15, -0.2, -0.2);

        }
        stopMotors();

        Thread.sleep(800);

//        robot aligns itself with the stack of cones
//        while(opModeIsActive() && robot.pipeline1.getError() != 0)
//        {
//            double var = robot.pipeline1.getError() * 0.0052;
//            moveRight(-var);
//
//        }
//        stopMotors();
//        robot.pipeline1.doCones(false);
        stopMotors();

        turnToPID(90);

        telemetry.addData("starting if" , 1);
//        telemetry.update();
        if (parkLocation == 1)
        {
//            moveForward(0.6, 300);
            robot.getRightFront().setPower(.5);
            robot.getLeftFront().setPower(.5);
            robot.getRightBack().setPower(.5);
            robot.getLeftBack().setPower(.5);
            sleep(700);
            stopMotors();
            telemetry.addData("location1" , 1);
//            telemetry.update();

        } else if (parkLocation == 2)
        {
//            moveBackward(0.6,150);
            telemetry.addData("location 2" , 1);
//            telemetry.update();

        } else //(parkLocation == 3)
        {
//           moveBackward(.6,500);
            robot.getRightFront().setPower(-.5);
            robot.getLeftFront().setPower(-.5);
            robot.getRightBack().setPower(-.5);
            robot.getLeftBack().setPower(-.5);
//            sleep(1000);
            telemetry.addData("location3" , 1);
//            telemetry.update();
        }
        telemetry.addData("finish if" , 1);
        telemetry.update();

        sleep(3000);
        stopMotors();


//        while(opModeIsActive())
//        {
//            telemetry.addData("finalLocation", parkLocation);
////            telemetry.update();
//        }


    }



}
