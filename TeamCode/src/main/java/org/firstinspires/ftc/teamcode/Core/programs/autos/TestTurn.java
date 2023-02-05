package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;

@Autonomous(name = "TestFieldCentric", group = "Opmodes")
public class TestTurn extends UpliftAutoImpl {
    @Override
    public void body() throws InterruptedException {
//        moveAndTurn(.5, 300, 30);
        sahilFieldCentric(30, .5,.5, 500, .5);

    }
}
