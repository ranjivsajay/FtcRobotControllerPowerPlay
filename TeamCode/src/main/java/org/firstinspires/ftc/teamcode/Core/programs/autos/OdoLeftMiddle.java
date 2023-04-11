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

            Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
            drive.setPoseEstimate(startPose);



            Trajectory traj1 = drive.trajectoryBuilder(startPose)
                  .lineToLinearHeading(new Pose2d(30, 6, Math.toRadians(70)))
                    //.lineToLinearHeading(new Pose2d(60,0,Math.toRadians(45)))
                 //   .addTemporalMarker(2,() -> {
                   //     servoArmsHigh();
                     //   robot.getFourBar().setPosition(0.8);
                       // robot.getTwister().setPosition(robot.getTwisterUpPos());
         //   })
                    .build();

           Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                   .lineToLinearHeading(new Pose2d(54, -1, Math.toRadians(45)))
//                   .addTemporalMarker(9,() -> {
//                robot.getSlide1().setPower(-.5);
//                robot.getSlide2().setPower(.5);
//                robot.getFourBar().setPosition(robot.getBarFrontPos());
//                robot.getArmRight().setPosition(robot.getArm2StackPos5());
//                robot.getArmLeft().setPosition(robot.getArm1StackPos5());
//                robot.getTwister().setPosition(robot.getTwisterUpPos());
//                robot.getGrabber1().setPosition(robot.getGrabber1OpenPos());
//
//            })
                  .build();

           Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                   .lineToLinearHeading(new Pose2d(50,15,Math.toRadians(92)))

                           .build();


              drive.followTrajectory(traj1);
            drive.followTrajectory(traj2);
//            moveBackward(.7,325);
//            Thread.sleep(500);
//           robot.getFourBar().setPosition(.95);
//            robot.getGrabber1().setPosition(robot.getGrabber1OpenPos());
//            Thread.sleep(500);
//            moveForward(.7,325);
//            Thread.sleep(3000);
//            drive.followTrajectory(traj3);








        }

}
