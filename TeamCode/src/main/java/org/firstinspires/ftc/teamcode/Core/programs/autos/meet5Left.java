package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;

@Autonomous(name = "meet5Left", group = "Opmodes")
public class meet5Left extends UpliftAutoImpl
{
    @Override
    public void initAction()
    {

    }

    @Override
    public void body() throws InterruptedException
    {
        turnToPID(90);
    }

}
