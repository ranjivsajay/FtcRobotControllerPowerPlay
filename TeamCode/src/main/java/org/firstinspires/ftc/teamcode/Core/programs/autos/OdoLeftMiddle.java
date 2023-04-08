package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.toolkit.UpliftMath;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "OdoLeftMiddle", group = "Opmodes")
public class OdoLeftMiddle extends UpliftAutoImpl
{



        @Override
        public void initAction() {
            robot.getOdoMid().setPosition(robot.getOdoMidDown());

        }

        @Override
        public void body() throws InterruptedException {
//            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//            Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
//            drive.setPoseEstimate(startPose);
//
//            Trajectory traj1 = drive.trajectoryBuilder(startPose)
//                    .forward(52)
//                    .build();
//
//            Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
//                    .back(4)
//                    .build();
//
//            drive.followTrajectory(traj1);
//
//            drive.followTrajectory(traj2);


            double poleDist = 500;

            double turnAngle = robot.imu.getAngularOrientation().firstAngle;

            robot.getLeftFront().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.getRightFront().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.getLeftFront().setTargetPosition(-600);
            robot.getRightFront().setTargetPosition(600);

            robot.getLeftFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.getRightFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.getLeftFront().setPower(-0.15);
            robot.getRightFront().setPower(0.15);
            robot.getLeftBack().setPower(-0.15);
            robot.getRightBack().setPower(0.15);

        while(opModeIsActive() && robot.getRightFront().isBusy())
        {
            if(robot.getPoleDetector().getDistance(DistanceUnit.MM) < poleDist)
            {
                poleDist = robot.getPoleDetector().getDistance(DistanceUnit.MM);
                turnAngle = robot.imu.getAngularOrientation().firstAngle;
            }
            telemetry.addData("dist", poleDist);
            telemetry.addData("angle", turnAngle);
            telemetry.update();
        }

        stopMotors();

        robot.getLeftFront().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getRightFront().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turnToPID(turnAngle - 3);


        }

}
