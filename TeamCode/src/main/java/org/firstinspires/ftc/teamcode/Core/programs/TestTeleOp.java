package org.firstinspires.ftc.teamcode.Core.programs;

import static java.lang.Math.sqrt;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.Threads.DriveThread;
import org.firstinspires.ftc.teamcode.Core.Threads.OperatorThread;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.main.UpliftTele;
import org.firstinspires.ftc.teamcode.Core.toolkit.UpliftMath;


@TeleOp(name = "TestTeleOp", group = "Opmodes")
public class TestTeleOp extends UpliftTele {

        UpliftRobot robot;

        @Override
        public void initHardware() {
            robot = new UpliftRobot(this);
        }

        @Override
        public void initAction() {

        }

        @Override

        public void bodyLoop() throws InterruptedException {

            telemetry.addData("Right Joystick Y Val: ", gamepad2.right_stick_y);
            telemetry.update();

        }

        @Override
        public void exit()
        {

        }

}
