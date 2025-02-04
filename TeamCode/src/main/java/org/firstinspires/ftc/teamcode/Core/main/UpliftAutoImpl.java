package org.firstinspires.ftc.teamcode.Core.main;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class UpliftAutoImpl extends UpliftAuto {


//    private double previousAngle = 0;
//    private double integratedAngle = 0;

    private ElapsedTime timer = new ElapsedTime();
    private Orientation lastAngles = new Orientation();
    private double currAngle = 0;

    public UpliftRobot robot;


    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);

    }

    @Override
    public void initAction() {

    }


    @Override
    public void body() throws InterruptedException {

    }

    @Override
    public void exit() throws InterruptedException {

    }

    public void fourBarFront() {
        robot.getFourBar().setPosition(robot.getBarFrontPos());
    }

    public void fourBarBack() {
        robot.getFourBar().setPosition(robot.getBarBackPos());
    }

    public void stopMotors() {
        robot.getLeftFront().setPower(0);
        robot.getLeftBack().setPower(0);
        robot.getRightFront().setPower(0);
        robot.getRightBack().setPower(0);
    }

    public void moveLeft(double power, double dist) {
        double initialPos = robot.getRightFront().getCurrentPosition();

        while (robot.getRightFront().getCurrentPosition() < initialPos + dist) {
            robot.getRightFront().setPower(power);
            robot.getRightBack().setPower(-power);
            robot.getLeftFront().setPower(-power);
            robot.getLeftBack().setPower(power);
        }
        stopMotors();
    }

    public void moveLeft(double power) {
        robot.getRightFront().setPower(power);
        robot.getRightBack().setPower(-power);
        robot.getLeftFront().setPower(-power);
        robot.getLeftBack().setPower(power);
    }

    public void moveRight(double power, double dist) {
        double initialPos = robot.getRightFront().getCurrentPosition();

        while (robot.getRightFront().getCurrentPosition() > initialPos - dist) {
            robot.getRightFront().setPower(-power);
            robot.getRightBack().setPower(power);
            robot.getLeftFront().setPower(power);
            robot.getLeftBack().setPower(-power);
        }
        stopMotors();
    }

    public void moveRight(double power) {
        robot.getRightFront().setPower(-power);
        robot.getRightBack().setPower(power);
        robot.getLeftFront().setPower(power);
        robot.getLeftBack().setPower(-power);
    }

    public void moveForwardUp(double drivePower, double slidesPower, int driveDist, int slidesDist) throws InterruptedException {
        servoArmsHigh();

        robot.getLeftFront().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getRightFront().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getLeftBack().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getRightBack().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.getSlide1().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getSlide2().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.getLeftFront().setTargetPosition(driveDist);
        robot.getLeftBack().setTargetPosition(driveDist);
        robot.getRightFront().setTargetPosition(driveDist);
        robot.getRightBack().setTargetPosition(driveDist);

        robot.getSlide1().setTargetPosition(-slidesDist);
        robot.getSlide2().setTargetPosition(slidesDist);

        robot.getLeftFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getLeftBack().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getRightFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getRightBack().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.getSlide1().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getSlide2().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.getRightFront().setPower(drivePower);
        robot.getRightBack().setPower(drivePower);
        robot.getLeftFront().setPower(drivePower);
        robot.getLeftBack().setPower(drivePower);

        robot.getSlide1().setPower(-slidesPower);
        robot.getSlide2().setPower(slidesPower);

        while (opModeIsActive() && robot.getRightFront().isBusy()) {

        }

        stopMotors();

//        robot.getSlide1().setPower(0);
//        robot.getSlide2().setPower(0);

        fourBarBack();


        robot.getTwister().setPosition(robot.getTwisterUpPos());

        robot.getLeftFront().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getRightFront().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getLeftBack().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getRightBack().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void fieldCentricMoveandUp(double drive, double strafe, double turn, double slidesPower, int slidesDist) throws InterruptedException {
        servoArmsHigh();

        // Get the current orientation of the robot
        double heading = Math.toRadians(getAbsoluteAngle());

        double fieldCentricX = strafe * Math.cos(-heading) - (drive * Math.sin(-heading));
        double fieldCentricY = strafe * Math.sin(-heading) + (drive * Math.cos(-heading));

        double denominator = Math.max(Math.abs(fieldCentricY) + Math.abs(fieldCentricX) + Math.abs(turn), 1);
        double frontLeftSpeed = (fieldCentricY + fieldCentricX + turn) / denominator;
        double frontRightSpeed = (fieldCentricY - fieldCentricX - turn) / denominator;
        double backLeftSpeed = (fieldCentricY - fieldCentricX + turn) / denominator;
        double backRightSpeed = (fieldCentricY + fieldCentricX - turn) / denominator;
        robot.getSlide1().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getSlide2().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getSlide1().setTargetPosition(-slidesDist);
        robot.getSlide2().setTargetPosition(slidesDist);
        robot.getSlide1().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getSlide2().setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Set the speeds for each wheel

        robot.getLeftFront().setPower(frontLeftSpeed);
        robot.getRightFront().setPower(frontRightSpeed);
        robot.getLeftBack().setPower(backLeftSpeed);
        robot.getRightBack().setPower(backRightSpeed);
        robot.getSlide1().setPower(-slidesPower);
        robot.getSlide2().setPower(slidesPower);

        while (opModeIsActive() && robot.getRightFront().isBusy()) {

        }

        stopMotors();

//        robot.getSlide1().setPower(0);
//        robot.getSlide2().setPower(0);

        fourBarBack();


        robot.getTwister().setPosition(robot.getTwisterUpPos());
    }



    public void moveForward(double power, double dist) {
        double initialPos = robot.getRightFront().getCurrentPosition();

        while (robot.getRightFront().getCurrentPosition() < initialPos + dist) {
            robot.getRightFront().setPower(power);
            robot.getRightBack().setPower(power);
            robot.getLeftFront().setPower(power);
            robot.getLeftBack().setPower(power);
        }
        stopMotors();
    }


    public void moveForward(double power) {
        robot.getRightFront().setPower(power);
        robot.getRightBack().setPower(power);
        robot.getLeftFront().setPower(power);
        robot.getLeftBack().setPower(power);
    }

    public void moveBackward(double power, double dist) {
        double initialPos = robot.getRightFront().getCurrentPosition();

        while (robot.getRightFront().getCurrentPosition() > initialPos - Math.abs(dist)) {
            robot.getRightFront().setPower(-power);
            robot.getRightBack().setPower(-power);
            robot.getLeftFront().setPower(-power);
            robot.getLeftBack().setPower(-power);
        }
        stopMotors();
    }

    public void moveBackward(double power) {
        robot.getRightFront().setPower(-power);
        robot.getRightBack().setPower(-power);
        robot.getLeftFront().setPower(-power);
        robot.getLeftBack().setPower(-power);
    }

    public void moveBackwardHigh(double drivePower, double slidesPower, int driveDist, int slidesDist) throws InterruptedException {

        servoArmsHigh();
        Thread.sleep(200);

        robot.getLeftFront().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getRightFront().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getLeftBack().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getRightBack().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.getSlide1().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getSlide2().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.getLeftFront().setTargetPosition(-driveDist);
        robot.getLeftBack().setTargetPosition(-driveDist);
        robot.getRightFront().setTargetPosition(-driveDist);
        robot.getRightBack().setTargetPosition(-driveDist);

        robot.getSlide1().setTargetPosition(-slidesDist);
        robot.getSlide2().setTargetPosition(slidesDist);

        robot.getLeftFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getLeftBack().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getRightFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getRightBack().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.getSlide1().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getSlide2().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.getRightFront().setPower(-drivePower);
        robot.getRightBack().setPower(-drivePower);
        robot.getLeftFront().setPower(-drivePower);
        robot.getLeftBack().setPower(-drivePower);

        robot.getSlide1().setPower(-slidesPower);
        robot.getSlide2().setPower(slidesPower);

        while (opModeIsActive() && robot.getRightFront().isBusy()) {

        }

        stopMotors();

//        robot.getSlide1().setPower(0);
//        robot.getSlide2().setPower(0);

        fourBarBack();
        robot.getTwister().setPosition(robot.getTwisterUpPos());

        robot.getLeftFront().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getRightFront().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getLeftBack().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getRightBack().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

//    public void turnRight(double power, double angle) {
//        double initialAngle = getIntegratedAngle();
//
//        while (getIntegratedAngle() > initialAngle - angle + 5) {
//            robot.getRightFront().setPower(-power);
//            robot.getRightBack().setPower(-power);
//            robot.getLeftFront().setPower(power);
//            robot.getLeftBack().setPower(power);
//            telemetry.addData("angle", getIntegratedAngle());
//            telemetry.update();
//        }
//        stopMotors();
//
//    }
//
//    public void turnLeft(double power, double angle) {
//        double initialAngle = getIntegratedAngle();
//
//        while (getIntegratedAngle() < initialAngle + angle - 10) {
//            robot.getRightFront().setPower(power);
//            robot.getRightBack().setPower(power);
//            robot.getLeftFront().setPower(-power);
//            robot.getLeftBack().setPower(-power);
//            telemetry.addData("angle", getIntegratedAngle());
//            telemetry.update();
//        }
//        stopMotors();
//    }

    public void servoArmsHigh() {
        robot.getArmLeft().setPosition(robot.getArmLeftHighPos());
        robot.getArmRight().setPosition(robot.getArmRightHighPos());
    }

    public void servoArmsDown() {
        robot.getArmLeft().setPosition(robot.getArmLeftLowPos());
        robot.getArmRight().setPosition(robot.getArmRightLowPos());
    }

//    /**
//     * This method returns a value of the Z axis of the REV Expansion Hub IMU.
//     * It transforms the value from (-180, 180) to (-inf, inf).
//     * This code was taken and modified from https://ftcforum.usfirst.org/forum/ftc-technology/53477-rev-imu-questions?p=53481#post53481.
//     *
//     * @return The integrated heading on the interval (-inf, inf).
//     */
//    private double getIntegratedAngle() {
//        double currentAngle = robot.imu.getAngularOrientation().firstAngle;
//        double deltaAngle = currentAngle - previousAngle;
//
//        if (deltaAngle < -180) {
//            deltaAngle += 360;
//        } else if (deltaAngle >= 180) {
//            deltaAngle -= 360;
//        }
//
//        integratedAngle += deltaAngle;
//        previousAngle = currentAngle;
//
//        return integratedAngle;
//    }

    public void resetAngle()
    {
        lastAngles = robot.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }

    public double getAngle()
    {

        Orientation orientation = robot.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;

        if(deltaAngle > 180)
        {
            deltaAngle -= 360;
        }
        else if(deltaAngle <= 180)
        {
            deltaAngle += 360;
        }

        currAngle += deltaAngle;
        lastAngles = orientation;

        return currAngle;
    }
//
    public double getAbsoluteAngle()
    {
        return robot.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
//
    public void turnToPID(double targetAngle)
    {
        TurnPID pid = new TurnPID(targetAngle, 0.013, 0, 0.003);
        while(opModeIsActive() && Math.abs(targetAngle - getAbsoluteAngle()) > 1)
        {
            double motorPower = pid.update(getAbsoluteAngle());
            robot.getLeftFront().setPower(-motorPower);
            robot.getRightFront().setPower(motorPower);
            robot.getLeftBack().setPower(-motorPower);
            robot.getRightBack().setPower(motorPower);
        }
        stopMotors();
    }

    public void pTurnTo(double targetAngle, double motorPower)
    {
        while(opModeIsActive() && Math.abs(targetAngle - getAbsoluteAngle()) > 1)
        {
            robot.getLeftFront().setPower(-motorPower);
            robot.getRightFront().setPower(motorPower);
            robot.getLeftBack().setPower(-motorPower);
            robot.getRightBack().setPower(motorPower);
        }
        stopMotors();
    }

    public void turnPID(double degrees)
    {
        turnToPID(degrees + getAbsoluteAngle());
    }

    public void turn(double degrees)
    {
        resetAngle();

        double error = degrees;

        while(opModeIsActive() && Math.abs(error) > 2)
        {
            double motorPower = (error < 0 ? -0.3: 0.3);
            robot.getLeftFront().setPower(-motorPower);
            robot.getRightFront().setPower(motorPower);
            robot.getLeftBack().setPower(-motorPower);
            robot.getRightBack().setPower(motorPower);
            error = degrees - getAngle();

        }

        stopMotors();
    }

    public void turnTo(double degrees)
    {
        Orientation orientation = robot.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double error = degrees - orientation.firstAngle;

        if(error > 180)
        {
            error -= 360;
        }
        else if(error <= 180)
        {
            error += 360;
        }

        turn(error);
    }

    public void fieldCentricMove(double drive, double strafe, double turn) {


        // Get the current orientation of the robot
        double heading = Math.toRadians(getAbsoluteAngle());

        double fieldCentricX = strafe * Math.cos(-heading) - (drive * Math.sin(-heading));
        double fieldCentricY = strafe * Math.sin(-heading) + (drive * Math.cos(-heading));

        double denominator = Math.max(Math.abs(fieldCentricY) + Math.abs(fieldCentricX) + Math.abs(turn), 1);
        double frontLeftSpeed = (fieldCentricY + fieldCentricX + turn) / denominator;
        double frontRightSpeed = (fieldCentricY - fieldCentricX - turn) / denominator;
        double backLeftSpeed = (fieldCentricY - fieldCentricX + turn) / denominator;
        double backRightSpeed = (fieldCentricY + fieldCentricX - turn) / denominator;

        // Set the speeds for each wheel

        robot.getLeftFront().setPower(frontLeftSpeed);
        robot.getRightFront().setPower(frontRightSpeed);
        robot.getLeftBack().setPower(backLeftSpeed);
        robot.getRightBack().setPower(backRightSpeed);


    }

    public void fieldCentricMoveUp(double drive, double strafe, double turn) {


        // Get the current orientation of the robot
        double heading = Math.toRadians(getAbsoluteAngle());

        double fieldCentricX = strafe * Math.cos(-heading) - (drive * Math.sin(-heading));
        double fieldCentricY = strafe * Math.sin(-heading) + (drive * Math.cos(-heading));

        double denominator = Math.max(Math.abs(fieldCentricY) + Math.abs(fieldCentricX) + Math.abs(turn), 1);
        double frontLeftSpeed = (fieldCentricY + fieldCentricX + turn) / denominator;
        double frontRightSpeed = (fieldCentricY - fieldCentricX - turn) / denominator;
        double backLeftSpeed = (fieldCentricY - fieldCentricX + turn) / denominator;
        double backRightSpeed = (fieldCentricY + fieldCentricX - turn) / denominator;

        // Set the speeds for each wheel

        robot.getLeftFront().setPower(frontLeftSpeed);
        robot.getRightFront().setPower(frontRightSpeed);
        robot.getLeftBack().setPower(backLeftSpeed);
        robot.getRightBack().setPower(backRightSpeed);




    }




    public void slides(double power, double dist)
    {

//        double initialPos1 = robot.getSlide2().getCurrentPosition();
        double initialPos2 = robot.getSlide2().getCurrentPosition();

        while (Math.abs(robot.getSlide2().getCurrentPosition() - (initialPos2 + dist)) > 50)
        {
            robot.getSlide1().setPower(-power);
            robot.getSlide2().setPower(power);

            telemetry.addData("ting ", Math.abs(robot.getSlide2().getCurrentPosition() - (initialPos2 + dist)));
            telemetry.update();
        }
        robot.getSlide1().setPower(0);
        robot.getSlide2().setPower(0);



    }

    public void turnToPole(int distance, double power)
    {
        double poleDist = 500;

        double turnAngle = robot.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

//        robot.getLeftFront().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getRightFront().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        robot.getLeftFront().setTargetPosition(-distance);
        robot.getRightFront().setTargetPosition(-distance);

//        robot.getLeftFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getRightFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.getLeftFront().setPower(power);
        robot.getRightFront().setPower(-power);
        robot.getLeftBack().setPower(power);
        robot.getRightBack().setPower(-power);

        while(opModeIsActive() && robot.getRightFront().isBusy())
        {
            if(robot.getPoleDetector().getDistance(DistanceUnit.MM) < poleDist)
            {
                poleDist = robot.getPoleDetector().getDistance(DistanceUnit.MM);
                turnAngle = robot.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            }
            telemetry.addData("dist", poleDist);
            telemetry.addData("angle", turnAngle);
            telemetry.update();
        }

        stopMotors();

        robot.getLeftFront().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getRightFront().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turnToPID(turnAngle + 5);
    }


    public void low()
    {
        slides(0.75,1800);
//        robot.getGrabber().setPosition(0.25);

    }

    public void medium()
    {
        slides(0.75,2600);
//        robot.getGrabber().setPosition(0.25);

    }

    public void high()
    {
        servoArmsHigh();

        slides(0.7, 953);

        fourBarBack();

        robot.getTwister().setPosition(robot.getTwisterUpPos());
    }

    public void down(double slidePower, double slideDist)
    {
        robot.getTwister().setPosition(robot.getTwisterUpPos());

        fourBarFront();

        slides(-slidePower, -slideDist);

        servoArmsDown();
    }


}
