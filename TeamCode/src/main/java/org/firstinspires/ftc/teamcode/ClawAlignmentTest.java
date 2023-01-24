package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TeamConf.FIELD_BEARING_NORTH;
import static org.firstinspires.ftc.teamcode.TeamConf.JUNCTIONS;
import static org.firstinspires.ftc.teamcode.TeamConf.START_POS_RED_RIGHT;
import static org.firstinspires.ftc.teamcode.TeamConf.TILE_SIZE;
import static org.firstinspires.ftc.teamcode.TeamConf.getRobot;
import static org.firstinspires.ftc.teamcode.TeamConf.stringToStartPose;
import static org.firstinspires.ftc.teamcode.util.PathTools.getTrajBuilder;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.PathTools;

@Config
@TeleOp
public class ClawAlignmentTest extends LinearOpMode {
    public static Pose2d startPose = START_POS_RED_RIGHT;
    public static Pose2d endPose = new Pose2d(TILE_SIZE, 0,  FIELD_BEARING_NORTH);

    RobotA robot;


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
        Pose2d adjustedEnd = PathTools.addClawOffset(endPose);

        Trajectory traj = getTrajBuilder(startPose).lineTo(adjustedEnd.vec()).build();

        robot = getRobot(hardwareMap);
        robot.drive.setPoseEstimate(startPose);

        waitForStart();

        robot.drive.followTrajectoryAsync(traj);

        while (robot.drive.isBusy()) {
            robot.update();
        }

        while (opModeIsActive()) {
            robot.drive.setWeightedDrivePower(new Pose2d(
                    gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    -gamepad1.right_stick_x
            ));
        }
    }
}
