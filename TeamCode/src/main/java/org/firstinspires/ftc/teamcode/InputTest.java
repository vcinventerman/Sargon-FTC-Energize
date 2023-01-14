package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TeamConf.nop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="InputTest")
public class InputTest extends LinearOpMode {

    @Override
    public void runOpMode()
    {
        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        String prevPad = gamepad1.toString();
        while (opModeIsActive())
        {
            if (!gamepad1.atRest())
            {
                nop();
            }
            String store = gamepad1.toString();
            if (!store.equals(prevPad)) {
                nop();
            }
            prevPad = store;

            if (gamepad1.a)
            {
                nop();
            }

            telemetry.addData("LeftTrigger", gamepad1.left_trigger);
            telemetry.update();
        }
    }
}
