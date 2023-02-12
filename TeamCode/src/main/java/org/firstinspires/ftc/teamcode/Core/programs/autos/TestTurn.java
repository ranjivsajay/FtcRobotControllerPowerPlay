package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;

@Autonomous(name = "TestFieldCentric", group = "Opmodes")
public class TestTurn extends UpliftAutoImpl {
    @Override
    public void body() throws InterruptedException {

        while(opModeIsActive() && Math.abs(90 - getAbsoluteAngle()) > 1)
        {

            fieldCentricMove(-0.4, -0.07, -0.2);

        }
        stopMotors();



    }
}
