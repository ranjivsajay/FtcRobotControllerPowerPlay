package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;

@Autonomous(name = "Red Park Auto", group = "Opmodes")
public class RedParkAuto extends UpliftAutoImpl {

    public void body() throws InterruptedException {
        moveLeft(0.5, 500);

    }
}

