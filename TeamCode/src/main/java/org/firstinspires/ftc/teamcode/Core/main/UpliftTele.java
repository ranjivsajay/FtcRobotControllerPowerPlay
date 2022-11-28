package org.firstinspires.ftc.teamcode.Core.main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Core.Threads.DriveThread;
import org.firstinspires.ftc.teamcode.Core.Threads.OperatorThread;

public abstract class UpliftTele extends LinearOpMode {

    protected UpliftRobot robot;

    DriveThread driverThread;
    OperatorThread operatorThread;

    public boolean isStarted, isLooping, isFinished;

    public abstract void initHardware();

    public abstract void initAction();

    public abstract void bodyLoop() throws InterruptedException;

    public abstract void exit();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Initializing", "Started");
        telemetry.update();

        initHardware();
        initAction();

        createDriveThread(robot);
        createOperatorThread(robot);


        telemetry.addData("Initializing", "Finished");
        telemetry.update();

        waitForStart();
        isStarted = true;

        driverThread.start();
        operatorThread.start();


        telemetry.addData("Body", "Started");
        telemetry.update();



        while (!isStopRequested()) {
            isLooping = true;
            bodyLoop();
        }

        telemetry.addData("Body", "Finished");

        driverThread.end();
        operatorThread.end();

        telemetry.update();


        exit();
    }

    public void createDriveThread(UpliftRobot robot1)
    {
        driverThread = new DriveThread(robot1);

        telemetry.addData("Driver Thread started", driverThread.toString());
    }

    public void createOperatorThread(UpliftRobot robot1) {
        operatorThread = new OperatorThread(robot1);

        telemetry.addData("Operator Thread started", operatorThread.toString());

    }
}