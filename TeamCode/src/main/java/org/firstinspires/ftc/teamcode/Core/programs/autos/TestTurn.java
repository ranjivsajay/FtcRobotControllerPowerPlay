package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;

@Autonomous(name = "TestFieldCentric", group = "Opmodes")
public class TestTurn extends UpliftAutoImpl {
    @Override
    public void body() throws InterruptedException {

        //junction to cone stack
        while(opModeIsActive() && Math.abs(88 - getAbsoluteAngle()) > 1)
        {

            fieldCentricMove(-0.54, -0.07, 0.2);

        }
        stopMotors();



    }
}



