package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;

@Autonomous(name = "TestFieldCentric", group = "Opmodes")
public class TestTurn extends UpliftAutoImpl {
    @Override
    public void body() throws InterruptedException {

        fieldCentricMove(0.5, 0, 0.5, 90);

    }
}
