package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;

@Autonomous(name = "test", group = "Opmodes")
public class test extends UpliftAutoImpl
{
    @Override
    public void initAction()
    {

//        robot.getTwister().setPosition(robot.getTwisterDownPos());
//        robot.getGrabber().setPosition(robot.getGrabberClosePos());
//
////        robot.getFourBar1().setPosition(0.25);
////        robot.getFourBar2().setPosition(0.075);
//
//        robot.getArm1().setPosition(robot.getArm1StackPos5());
//        robot.getArm2().setPosition(robot.getArm2StackPos5());
//
//        fourBarFront();
        robot.getFourBar().setPosition(robot.getBarFrontPos());
        robot.getArmLeft().setPosition(robot.getArm1StackPos5());
        robot.getArmRight().setPosition(robot.getArm2StackPos5());
        robot.getGrabber1().setPosition(robot.getGrabber1OpenPos());
        robot.getTwister().setPosition(robot.getTwisterDownPos());
robot.getWebcam().setPipeline(robot.pipeline2);

    }

    @Override
    public void body() throws InterruptedException
    {

        while (robot.getConeDetector().getDistance(DistanceUnit.CM) > 24)
        {
            moveRight(.2);
        }

//        telemetry.addData("error:" , robot.pipeline2.getError());
//        telemetry.update();
//        while(opModeIsActive())
//        {
//            double var = robot.pipeline2.getError() * 0.003;
//            moveRight(-var);
//        }
//        stopMotors();



//        double coneDist = 500;
//
//        double turnAngle = robot.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
//
//        robot.getLeftFront().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.getRightFront().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        robot.getLeftFront().setTargetPosition(350);
//        robot.getRightFront().setTargetPosition(-350);
//
//
//        robot.getLeftFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.getRightFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//
//        robot.getLeftFront().setPower(0.15);
//        robot.getRightFront().setPower(-0.15);
//        robot.getLeftBack().setPower(0.15);
//        robot.getRightBack().setPower(-0.15);
//
//        while(opModeIsActive() && robot.getRightFront().isBusy())
//        {
//            if(robot.getConeDetector().getDistance(DistanceUnit.METER) < coneDist)
//            {
//                coneDist = robot.getConeDetector().getDistance(DistanceUnit.METER);
//                turnAngle = robot.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
//            }
//            telemetry.addData("distance", coneDist);
//            telemetry.update();
//        }
//
//        stopMotors();
//
//        robot.getLeftFront().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.getRightFront().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        turnToPID(turnAngle);

//        double poleDist = 500;
//        double turnAngle = robot.imu.getAngularOrientation().firstAngle;
//
//        robot.getLeftFront().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.getRightFront().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        robot.getLeftFront().setTargetPosition(350);
//        robot.getRightFront().setTargetPosition(-350);
//
//
//        robot.getLeftFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.getRightFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//
//        robot.getLeftFront().setPower(0.15);
//        robot.getRightFront().setPower(-0.15);
//        robot.getLeftBack().setPower(0.15);
//        robot.getRightBack().setPower(-0.15);
//
//        while(opModeIsActive() && robot.getRightFront().isBusy())
//        {
//            if(robot.getPoleDetector().getDistance(DistanceUnit.MM) < poleDist)
//            {
//                poleDist = robot.getPoleDetector().getDistance(DistanceUnit.MM);
//                turnAngle = robot.imu.getAngularOrientation().firstAngle;
//            }
//            telemetry.addData("dist", poleDist);
//            telemetry.addData("angle", turnAngle);
//            telemetry.update();
//        }
//
//        stopMotors();
//
//        robot.getLeftFront().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.getRightFront().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        turnToPID(turnAngle - 3);



    }

}
