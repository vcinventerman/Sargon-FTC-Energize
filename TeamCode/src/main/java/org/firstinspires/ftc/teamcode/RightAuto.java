package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TeamConf.CONE_STACK_POS_RED_RIGHT;
import static org.firstinspires.ftc.teamcode.TeamConf.FIELD_BEARING_EAST;
import static org.firstinspires.ftc.teamcode.TeamConf.FIELD_BEARING_NORTH;
import static org.firstinspires.ftc.teamcode.TeamConf.JUNCTIONS;
import static org.firstinspires.ftc.teamcode.TeamConf.START_POS_RED_RIGHT;
import static org.firstinspires.ftc.teamcode.TeamConf.TILE_SIZE;
import static org.firstinspires.ftc.teamcode.TeamConf.getRobot;
import static org.firstinspires.ftc.teamcode.TeamConf.stringToStartPose;
import static org.firstinspires.ftc.teamcode.util.PathTools.addClawOffset;
import static org.firstinspires.ftc.teamcode.util.PathTools.addClawOffsetVec;
import static org.firstinspires.ftc.teamcode.util.PathTools.getTrajBuilder;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.PathTools;

@Config
@TeleOp
public class RightAuto extends LinearOpMode {
    public static Pose2d startPose = START_POS_RED_RIGHT;

    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        Trajectory[] trajectories = new Trajectory[9];

        trajectories[0] = getTrajBuilder(START_POS_RED_RIGHT) // Forward junction
                .lineTo(new Vector2d(START_POS_RED_RIGHT.getX() - TILE_SIZE / 2, START_POS_RED_RIGHT.getY()))
                .splineToLinearHeading(new Pose2d(START_POS_RED_RIGHT.getX() - TILE_SIZE, START_POS_RED_RIGHT.getY() + TILE_SIZE, FIELD_BEARING_NORTH), FIELD_BEARING_NORTH)
                .splineToSplineHeading(addClawOffsetVec(JUNCTIONS.get(1), FIELD_BEARING_NORTH + Math.PI / 4), FIELD_BEARING_NORTH + Math.PI / 4)
                .build();

        trajectories[1] = getTrajBuilder(addClawOffsetVec(JUNCTIONS.get(1), FIELD_BEARING_NORTH + Math.PI / 4)) // Cone stack
                .back(TILE_SIZE / 16)
                .splineToConstantHeading(new Vector2d(START_POS_RED_RIGHT.getX() - TILE_SIZE, START_POS_RED_RIGHT.getY() + TILE_SIZE), FIELD_BEARING_NORTH)
                .splineToSplineHeading(new Pose2d(START_POS_RED_RIGHT.getX() - TILE_SIZE, START_POS_RED_RIGHT.getY() + TILE_SIZE * (3.0 / 4.0), FIELD_BEARING_NORTH), FIELD_BEARING_NORTH)
                .splineToSplineHeading(new Pose2d(START_POS_RED_RIGHT.getX() - TILE_SIZE, START_POS_RED_RIGHT.getY() + TILE_SIZE * 2, FIELD_BEARING_NORTH), FIELD_BEARING_NORTH)
                .splineToConstantHeading(new Vector2d(TILE_SIZE, -TILE_SIZE / 2.0), FIELD_BEARING_EAST)
                .splineToConstantHeading(new Vector2d(TILE_SIZE * (3.0 / 2.0), -TILE_SIZE / 2.0), FIELD_BEARING_EAST)
                .splineToSplineHeading(addClawOffset(CONE_STACK_POS_RED_RIGHT), FIELD_BEARING_EAST)
                .build();

        trajectories[2] =getTrajBuilder(addClawOffset(CONE_STACK_POS_RED_RIGHT)) // Low junction adjacent to cone stack
                .back(TILE_SIZE / 16)
                .splineToSplineHeading(addClawOffsetVec(JUNCTIONS.get(11), Math.PI * (5.0 / 4.0)), Math.PI * (5.0 / 4.0))
                .build();

        trajectories[3] = getTrajBuilder(addClawOffsetVec(JUNCTIONS.get(11), Math.PI * (5.0 / 4.0))) // Low junction adjacent to cone stack return to cone stack
                .back(TILE_SIZE / 16)
                .splineToSplineHeading(addClawOffset(CONE_STACK_POS_RED_RIGHT), 0)
                //todo: straight approach to let claw lower
                .build();

        trajectories[4] = getTrajBuilder(addClawOffset(CONE_STACK_POS_RED_RIGHT)) // Medium junction inline with cone stack
                .back(TILE_SIZE / 16)
                .splineToSplineHeading(addClawOffsetVec(JUNCTIONS.get(5), Math.PI * (5.0 / 4.0)), Math.PI * (5.0 / 4.0))
                .build();

        trajectories[5] = getTrajBuilder(addClawOffsetVec(JUNCTIONS.get(5), Math.PI * (5.0 / 4.0))) // Medium junction inline with cone stack return
                .back(TILE_SIZE / 16)
                .splineToSplineHeading(addClawOffset(CONE_STACK_POS_RED_RIGHT), 0)
                //todo: straight approach to let claw lower
                .build();

        trajectories[6] = getTrajBuilder(addClawOffset(CONE_STACK_POS_RED_RIGHT)) // High junction inline with cone stack
                .back(TILE_SIZE / 16)
                .splineToSplineHeading(addClawOffsetVec(JUNCTIONS.get(2), Math.PI * (3.0 / 4.0)), Math.PI * (3.0 / 4.0))
                .build();

        trajectories[7] = getTrajBuilder(addClawOffsetVec(JUNCTIONS.get(2), Math.PI * (3.0 / 4.0))) // Medium junction inline with cone stack return
                .back(TILE_SIZE / 16)
                .splineToSplineHeading(addClawOffset(CONE_STACK_POS_RED_RIGHT), 0)
                //todo: straight approach to let claw lower
                .build();

        robot = getRobot(hardwareMap);
        robot.setPose(startPose);

        waitForStart();

        runTrajectory(trajectories[0]);
        runTrajectory(trajectories[1]);
        runTrajectory(trajectories[2]);
        runTrajectory(trajectories[3]);
        runTrajectory(trajectories[4]);
        runTrajectory(trajectories[5]);
        runTrajectory(trajectories[6]);
        runTrajectory(trajectories[7]);



    }

    public void runTrajectory(Trajectory trajectory) {
        robot.followTrajectory(trajectory);
        while (robot.isBusy() && opModeIsActive()) {
            robot.update();

            Pose2d poseEstimate = robot.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
