package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
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
                    .strafeRight(12)
                    .build();
            drive.followTrajectory(traj1);

            turnToPole(300, 0.1);

            while(robot.getPoleDetector().getDistance(DistanceUnit.CM) > 5)
            {
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

            Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), Math.toRadians(90))
                    .strafeRight(5)
                    .build();
            drive.followTrajectory(traj3);










        }

}
