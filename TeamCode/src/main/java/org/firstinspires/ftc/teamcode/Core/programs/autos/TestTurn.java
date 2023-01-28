package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;

@Autonomous(name = "testTurn", group = "Opmodes")
public class TestTurn extends UpliftAutoImpl
{
    @Override
    public void body() throws InterruptedException
    {
        turnPID(180);
        turnToPID(90);
    }

}
