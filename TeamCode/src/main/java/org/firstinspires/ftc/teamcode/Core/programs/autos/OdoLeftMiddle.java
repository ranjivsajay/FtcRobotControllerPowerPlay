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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.toolkit.UpliftMath;
import org.firstinspires.ftc.teamcode.Core.toolkit.vision.ConesPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "OdoLeftMiddle", group = "Opmodes")
public class OdoLeftMiddle extends UpliftAutoImpl {


    @Override
    public void initAction() {

        robot.getFourBar().setPosition(.75);

        robot.getArmLeft().setPosition(robot.getArmLeftLowPos());
        robot.getArmRight().setPosition(robot.getArmRightLowPos());

        robot.getGrabber1().setPosition(robot.getGrabber1ClosePos());
        robot.getTwister().setPosition(robot.getTwisterDownPos());

        robot.getOdoMid().setPosition(robot.getOdoMidDown());

    }

    @Override
    public void body() throws InterruptedException {

        int parkLocation = robot.pipeline1.location;
        robot.getWebcam().setPipeline(robot.pipeline2);

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

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .strafeRight(11)
                .build();
        drive.followTrajectory(traj1);

        turnToPole(300, 0.2);

        while (robot.getPoleDetector().getDistance(DistanceUnit.CM) > 4) {
            moveBackward(0.2);
        }
        stopMotors();

        robot.getFourBar().setPosition(robot.getBarBackPos());
        robot.getGrabber1().setPosition(robot.getGrabber1OpenPos());

        Thread.sleep(500);

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(), Math.toRadians(90))
                .forward(4)
                .build();
        drive.followTrajectory(traj2);

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeRight(6)
                .build();
        drive.followTrajectory(traj3);

        turnToPID(90);

        robot.getFourBar().setPosition(robot.getBarFrontPos());
        robot.getArmLeft().setPosition(robot.getArm1StackPos5());
        robot.getArmRight().setPosition(robot.getArm2StackPos5());
        robot.getGrabber1().setPosition(robot.getGrabber1OpenPos());
        robot.getTwister().setPosition(robot.getTwisterDownPos());

        sleep(1000);
        robot.getWebcam().setPipeline(robot.pipeline2);


//            if (robot.getConeDetector().getDistance(DistanceUnit.CM) < 54) {
//                drive.followTrajectory(traj2);
//            } else {
//                while (robot.getConeDetector().getDistance(DistanceUnit.CM) > 54) {
//                    moveRight(.2);
//                }
//                stopMotors();
//            }

        //robot aligns itself with the stack of cones
        while(opModeIsActive() && robot.pipeline2.getError() != 0)
        {
            double var = robot.pipeline2.getError() * 0.0042;
            moveRight(-var);
            while(robot.getLeftFront().isBusy())
            {
                if (Math.abs(robot.getLeftFront().getPower() ) < .2)
                    stopMotors();
            }

        }
        stopMotors();


        if(robot.pipeline2.getError() == 0)
        {
            while (robot.getConeDetector().getDistance(DistanceUnit.CM) > 8)
            {
                moveForward(0.25);
            }

            stopMotors();
        }
        else
        {
            while(opModeIsActive() && robot.pipeline2.getError() != 0)
            {
                double var = robot.pipeline2.getError() * 0.0042;
                moveRight(-var);
                while(robot.getLeftFront().isBusy())
                {
                    if (Math.abs(robot.getLeftFront().getPower() ) < .2)
                        stopMotors();
                }
            }
            stopMotors();
        }





            robot.getGrabber1().setPosition(robot.getGrabber1ClosePos());

            Thread.sleep(300);


            servoArmsHigh();
            robot.getFourBar().setPosition(0.75);
            Thread.sleep(300);


            robot.getTwister().setPosition(robot.getTwisterUpPos());

            while (opModeIsActive() && Math.abs(69 - getAbsoluteAngle()) > 1) {

                fieldCentricMove(-0.08, 0.9, 0.2);

            }
            stopMotors();

            turnToPole(300, 0.2);

            while (robot.getPoleDetector().getDistance(DistanceUnit.CM) > 4) {
                moveBackward(0.2);
            }
            stopMotors();

            fourBarBack();
            robot.getGrabber1().setPosition(robot.getGrabber1OpenPos());

            while (opModeIsActive() && Math.abs(90 - getAbsoluteAngle()) > 1) {

                fieldCentricMove(0.05, -0.2, -0.2);

            }
            stopMotors();

            //robot aligns itself with the stack of cones
            while (opModeIsActive() && robot.pipeline2.getError() != 0) {
                double var = robot.pipeline2.getError() * 0.0042;
                moveRight(-var);
            }
            stopMotors();

            turnToPID(90);

            robot.getFourBar().setPosition(robot.getBarFrontPos());
            robot.getArmLeft().setPosition(robot.getArm1StackPos5());
            robot.getArmRight().setPosition(robot.getArm2StackPos5());
            robot.getGrabber1().setPosition(robot.getGrabber1OpenPos());
            robot.getTwister().setPosition(robot.getTwisterDownPos());

            sleep(300);


        while(opModeIsActive() && robot.pipeline2.getError() != 0)
        {
            double var = robot.pipeline2.getError() * 0.0042;
            moveRight(-var);
            while(robot.getLeftFront().isBusy())
            {
                if (Math.abs(robot.getLeftFront().getPower() ) < .2)
                    stopMotors();
            }

        }
        stopMotors();


        if(robot.pipeline2.getError() == 0)
        {
            while (robot.getConeDetector().getDistance(DistanceUnit.CM) > 8)
            {
                moveForward(0.25);
            }

            stopMotors();
        }
        else
        {
            while(opModeIsActive() && robot.pipeline2.getError() != 0)
            {
                double var = robot.pipeline2.getError() * 0.0042;
                moveRight(-var);
                while(robot.getLeftFront().isBusy())
                {
                    if (Math.abs(robot.getLeftFront().getPower() ) < .2)
                        stopMotors();
                }
            }
            stopMotors();
        }




            robot.getGrabber1().setPosition(robot.getGrabber1ClosePos());

            Thread.sleep(300);


            servoArmsHigh();
            robot.getFourBar().setPosition(0.75);
            Thread.sleep(300);


            robot.getTwister().setPosition(robot.getTwisterUpPos());


        }

    }
