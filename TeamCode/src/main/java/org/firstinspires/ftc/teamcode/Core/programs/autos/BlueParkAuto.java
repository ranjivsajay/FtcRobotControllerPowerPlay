package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.core.UpliftAutoImpl;

@Autonomous(name = "Blue Park Auto", group = "Opmodes")
public class BlueParkAuto extends UpliftAutoImpl {

    public void body() throws InterruptedException
    {
        moveRight(0.5, 500);
        stopMotors();

    }
}
