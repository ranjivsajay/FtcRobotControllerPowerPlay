package org.firstinspires.ftc.teamcode.Core.main;

import com.qualcomm.robotcore.hardware.DcMotor;

public class UpliftAutoImpl extends UpliftAuto {


    private double previousAngle = 0;
    private double integratedAngle = 0;


    public UpliftRobot robot;


    @Override
    public void initHardware()
    {
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

    public void fourBarFront()
    {
        robot.getFourBar1().setPosition(robot.getBar1FrontPos());
        robot.getFourBar2().setPosition(robot.getBar2FrontPos());
    }

    public void fourBarBack()
    {
        robot.getFourBar1().setPosition(robot.getBar1BackPos());
        robot.getFourBar2().setPosition(robot.getBar2BackPos());
    }

    public void stopMotors() {
        robot.getLeftFront().setPower(0);
        robot.getLeftBack().setPower(0);
        robot.getRightFront().setPower(0);
        robot.getRightBack().setPower(0);
    }

    public void moveLeft(double power, int dist) {
//        robot.getLeftFront().setTargetPosition(dist);
//        robot.getLeftBack().setTargetPosition(dist);
//        robot.getRightFront().setTargetPosition(dist);
//        robot.getRightBack().setTargetPosition(dist);
//
//        robot.getLeftFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.getLeftFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.getLeftFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.getLeftFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.getRightFront().setPower(power);
//        robot.getRightBack().setPower(-power);
//        robot.getLeftFront().setPower(-power);
//        robot.getLeftBack().setPower(power);


        double initialPos = robot.getRightFront().getCurrentPosition();

        while (robot.getRightFront().getCurrentPosition() < initialPos + dist)
        {
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

    public void moveRight(double power, int dist) {

//        robot.getLeftFront().setTargetPosition(dist);
//        robot.getLeftBack().setTargetPosition(dist);
//        robot.getRightFront().setTargetPosition(dist);
//        robot.getRightBack().setTargetPosition(dist);
//
//        robot.getLeftFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.getLeftFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.getLeftFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.getLeftFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.getRightFront().setPower(-power);
//        robot.getRightBack().setPower(power);
//        robot.getLeftFront().setPower(power);
//        robot.getLeftBack().setPower(-power);

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

    public void moveForwardHigh(double drivePower, double slidesPower, int driveDist, int slidesDist)
    {
        robot.getLeftFront().setTargetPosition(driveDist);
        robot.getLeftBack().setTargetPosition(driveDist);
        robot.getRightFront().setTargetPosition(driveDist);
        robot.getRightBack().setTargetPosition(driveDist);

        robot.getSlide1().setTargetPosition(-slidesDist);
        robot.getSlide2().setTargetPosition(slidesDist);

        robot.getLeftFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getLeftFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getLeftFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getLeftFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.getSlide1().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getSlide2().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.getRightFront().setPower(drivePower);
        robot.getRightBack().setPower(drivePower);
        robot.getLeftFront().setPower(drivePower);
        robot.getLeftBack().setPower(drivePower);

        robot.getSlide1().setPower(-slidesPower);
        robot.getSlide2().setPower(slidesPower);
    }

    public void moveForward(double power, int dist) {

//        robot.getLeftFront().setTargetPosition(dist);
//        robot.getLeftBack().setTargetPosition(dist);
//        robot.getRightFront().setTargetPosition(dist);
//        robot.getRightBack().setTargetPosition(dist);
//
//        robot.getLeftFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.getLeftFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.getLeftFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.getLeftFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.getRightFront().setPower(power);
//        robot.getRightBack().setPower(power);
//        robot.getLeftFront().setPower(power);
//        robot.getLeftBack().setPower(power);

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

    public void moveBackward(double power, int dist) {

//        robot.getLeftFront().setTargetPosition(dist);
//        robot.getLeftBack().setTargetPosition(dist);
//        robot.getRightFront().setTargetPosition(dist);
//        robot.getRightBack().setTargetPosition(dist);
//
//        robot.getLeftFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.getLeftFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.getLeftFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.getLeftFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.getRightFront().setPower(-power);
//        robot.getRightBack().setPower(-power);
//        robot.getLeftFront().setPower(-power);
//        robot.getLeftBack().setPower(-power);

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

    public void turnRight(double power, double angle) {
        double initialAngle = getIntegratedAngle();

        while (getIntegratedAngle() > initialAngle - angle + 5) {
            robot.getRightFront().setPower(-power);
            robot.getRightBack().setPower(-power);
            robot.getLeftFront().setPower(power);
            robot.getLeftBack().setPower(power);
            telemetry.addData("angle", getIntegratedAngle());
            telemetry.update();
        }
        stopMotors();

    }

    public void turnLeft(double power, double angle) {
        double initialAngle = getIntegratedAngle();

        while (getIntegratedAngle() < initialAngle + angle - 10) {
            robot.getRightFront().setPower(power);
            robot.getRightBack().setPower(power);
            robot.getLeftFront().setPower(-power);
            robot.getLeftBack().setPower(-power);
            telemetry.addData("angle", getIntegratedAngle());
            telemetry.update();
        }
        stopMotors();
    }

    public void servoArmsHigh()
    {
        robot.getArm1().setPosition(robot.getArm1HighPos());
        robot.getArm2().setPosition(robot.getArm2HighPos());
    }

    public void servoArmsDown()
    {
        robot.getArm1().setPosition(robot.getArm1LowPos());
        robot.getArm2().setPosition(robot.getArm2LowPos());
    }

    /**
     * This method returns a value of the Z axis of the REV Expansion Hub IMU.
     * It transforms the value from (-180, 180) to (-inf, inf).
     * This code was taken and modified from https://ftcforum.usfirst.org/forum/ftc-technology/53477-rev-imu-questions?p=53481#post53481.
     *
     * @return The integrated heading on the interval (-inf, inf).
     */
    private double getIntegratedAngle() {
        double currentAngle = robot.imu.getAngularOrientation().firstAngle;
        double deltaAngle = currentAngle - previousAngle;

        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle >= 180) {
            deltaAngle -= 360;
        }

        integratedAngle += deltaAngle;
        previousAngle = currentAngle;

        return integratedAngle;
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

        robot.getTwister().setPosition(robot.getTwisterDownPos());
    }

    public void down(double slidePower, double slideDist)
    {
        robot.getTwister().setPosition(robot.getTwisterUpPos());

        fourBarFront();

        slides(-slidePower, -slideDist);

        servoArmsDown();
    }


}
