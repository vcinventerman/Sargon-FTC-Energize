package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TeamConf.FIELD_BEARING_NORTH;
import static org.firstinspires.ftc.teamcode.TeamConf.START_POS_RED_RIGHT;
import static org.firstinspires.ftc.teamcode.TeamConf.TILE_SIZE;
import static org.firstinspires.ftc.teamcode.TeamConf.getRobot;
import static org.firstinspires.ftc.teamcode.util.PathTools.getTrajBuilder;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.PathTools;

@Config
@TeleOp
public class SlideCalibrator extends LinearOpMode {
    Robot robot;


    /**
     * Override this method and place your code here.
     * <p>
     * Please do not swallow the InterruptedException, as it is used in cases
     * where the op mode needs to be terminated early.
     *
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {
        GamepadEx g1 = new GamepadEx(gamepad1);

        robot = getRobot(hardwareMap);

        InterpLUT interpolator = robot.slide.p.STALL_POWER;

        robot.slide.winch.setRunMode(Motor.RunMode.RawPower);

        waitForStart();

        while (opModeIsActive()) {
            g1.readButtons();

            if (g1.wasJustPressed(GamepadKeys.Button.A)) {
                interpolator.add(robot.slide.winch.getCurrentPosition(), g1.getLeftY());
            }

            robot.slide.winch.set(g1.getLeftY());

            telemetry.addData("y", robot.slide.winch.getCurrentPosition());
            telemetry.addData("Power", g1.getLeftY());
            telemetry.update();
        }
    }
}
