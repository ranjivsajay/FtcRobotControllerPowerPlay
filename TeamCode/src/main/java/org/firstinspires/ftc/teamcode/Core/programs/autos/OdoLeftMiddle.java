package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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

            robot.getFourBar().setPosition(.75);

            robot.getArmLeft().setPosition(robot.getArmLeftLowPos());
            robot.getArmRight().setPosition(robot.getArmRightLowPos());

            robot.getGrabber1().setPosition(robot.getGrabber1ClosePos());
            robot.getTwister().setPosition(robot.getTwisterDownPos());

            robot.getOdoMid().setPosition(robot.getOdoMidDown());

        }

        @Override
        public void body() throws InterruptedException {
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
            drive.setPoseEstimate(startPose);

            Trajectory traj1 = drive.trajectoryBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(20, 6, Math.toRadians(90)))
                    .build();

            Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                    .lineToLinearHeading(new Pose2d(54, -1, Math.toRadians(45)))
                    .build();

            drive.followTrajectory(traj1);

            drive.followTrajectory(traj2);

            servoArmsHigh();

            sleep(5000);




        }

}
