package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TeamConf.ROBOT_DRIVE_INST;
import static org.firstinspires.ftc.teamcode.TeamConf.TILE_SIZE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.karrmedia.ftchotpatch.Supervised;
import com.karrmedia.ftchotpatch.SupervisedOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.PathTools;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Supervised(name="WiiDemo", autonomous=false, linear=false)
public class WiiDemo extends SupervisedOpMode {

    RobotA robot;

    @Override
    public void init() {
        robot = new RobotA(hardwareMap);
    }

    @Override
    public void loop() {
        gamepad1.readButtons();

        if (gamepad1.wasJustPressed(GamepadKeys.Button.A)) {
            robot.drive.cancelFollowing();

            Trajectory traj = PathTools.safeTrajTo(robot.drive.getPoseEstimate(),
                    new Pose2d(gamepad1.gamepad.touchpad_finger_1_x, gamepad1.gamepad.touchpad_finger_1_y, 0));
        }

        robot.update();
    }
}
