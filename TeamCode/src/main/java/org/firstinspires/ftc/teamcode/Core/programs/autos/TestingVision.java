package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Testing Vision", group = "Opmodes")
public class TestingVision extends UpliftAutoImpl
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

            int parkLocation = robot.pipeline1.location;


            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

           // Pose2d startPose = new Pose2d(-63, 36, Math.toRadians(0));
            Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
            drive.setPoseEstimate(startPose);

            Trajectory traj1 = drive.trajectoryBuilder(startPose)
                    .lineToConstantHeading(new Vector2d(54,0))
                    .build();
            drive.followTrajectory(traj1);
            turnToPID(90);
            Thread.sleep(1000);

            if (parkLocation == 1) {
                moveForward(.6);
                Thread.sleep(800);
                stopMotors();
                } else if (parkLocation == 2) {
                        stopMotors();

                } else if (parkLocation == 3) {
               moveBackward(.6);
               Thread.sleep(800);
                stopMotors();

                }
















        }

}
