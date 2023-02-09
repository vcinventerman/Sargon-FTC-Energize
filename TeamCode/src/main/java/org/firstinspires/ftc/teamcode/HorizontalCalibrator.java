package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TeamConf.CONE_STACK_POS_RED_LEFT;
import static org.firstinspires.ftc.teamcode.TeamConf.CONE_STACK_POS_RED_RIGHT;
import static org.firstinspires.ftc.teamcode.TeamConf.FIELD_BEARING_EAST;
import static org.firstinspires.ftc.teamcode.TeamConf.FIELD_BEARING_NORTH;
import static org.firstinspires.ftc.teamcode.TeamConf.JUNCTIONS;
import static org.firstinspires.ftc.teamcode.TeamConf.START_POS_RED_RIGHT;
import static org.firstinspires.ftc.teamcode.TeamConf.TILE_SIZE;
import static org.firstinspires.ftc.teamcode.TeamConf.getRobot;
import static org.firstinspires.ftc.teamcode.util.PathTools.addClawOffset;
import static org.firstinspires.ftc.teamcode.util.PathTools.addClawOffsetVec;
import static org.firstinspires.ftc.teamcode.util.PathTools.getTrajBuilder;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(group="!CompAuto")
public class HorizontalCalibrator extends LinearOpMode {
    public static double startPoseX = START_POS_RED_RIGHT.getX();
    public static double startPoseY = START_POS_RED_RIGHT.getY();
    public static double startPoseHeading = START_POS_RED_RIGHT.getHeading();


    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d startPose = new Pose2d(startPoseX, startPoseY, startPoseHeading);

        robot = getRobot(hardwareMap);
        robot.setPose(startPose);

        // Creating these trajectories will take a while
        Trajectory[] trajectories = new Trajectory[9];

        trajectories[0] = getTrajBuilder(startPose) // Forward junction
                .splineToConstantHeading(new Vector2d(TILE_SIZE * (3.0 / 4.0), START_POS_RED_RIGHT.getY() + 6), FIELD_BEARING_NORTH)
                .splineToConstantHeading(new Vector2d(TILE_SIZE * (2.5 / 4.0), -TILE_SIZE * (1.0/2.0)), FIELD_BEARING_NORTH)
                .splineToSplineHeading(new Pose2d(TILE_SIZE * (2.5 / 4.0), START_POS_RED_RIGHT.getY() + TILE_SIZE * 2, FIELD_BEARING_EAST), FIELD_BEARING_EAST + PI/3)
                .splineToConstantHeading(new Vector2d(TILE_SIZE, -TILE_SIZE / 2.0 - 2), FIELD_BEARING_EAST)
                .splineToConstantHeading(new Vector2d(TILE_SIZE * (3.0 / 2.0), -TILE_SIZE / 2.0 - 2.5), FIELD_BEARING_EAST)
                .splineToSplineHeading(addClawOffset(CONE_STACK_POS_RED_RIGHT), FIELD_BEARING_EAST)
                .build();

        trajectories[1] = getTrajBuilder(addClawOffsetVec(JUNCTIONS.get(1), FIELD_BEARING_NORTH + Math.PI / 4)) // Cone stack
                .back(TILE_SIZE / 8)
                .splineToSplineHeading(new Pose2d(TILE_SIZE * (1.0 / 2.0), -TILE_SIZE * (3.0 / 2), FIELD_BEARING_NORTH), PI * (7.0 / 4.0))
                .splineToConstantHeading(new Vector2d(TILE_SIZE * (2.5 / 4.0), START_POS_RED_RIGHT.getY() + TILE_SIZE * 3.0/2.0), FIELD_BEARING_NORTH)
                //.splineToSplineHeading(new Pose2d(TILE_SIZE * (2.5 / 4.0), START_POS_RED_RIGHT.getY() + TILE_SIZE * (3.0 / 4.0), FIELD_BEARING_NORTH), FIELD_BEARING_NORTH)
                .splineToSplineHeading(new Pose2d(TILE_SIZE * (2.5 / 4.0), START_POS_RED_RIGHT.getY() + TILE_SIZE * 2, FIELD_BEARING_EAST), FIELD_BEARING_EAST + PI/3)
                .splineToConstantHeading(new Vector2d(TILE_SIZE, -TILE_SIZE / 2.0 - 2), FIELD_BEARING_EAST)
                .splineToConstantHeading(new Vector2d(TILE_SIZE * (3.0 / 2.0), -TILE_SIZE / 2.0 - 2.5), FIELD_BEARING_EAST)
                .splineToSplineHeading(addClawOffset(CONE_STACK_POS_RED_RIGHT), FIELD_BEARING_EAST)
                .build();

        trajectories[2] = getTrajBuilder(addClawOffset(CONE_STACK_POS_RED_RIGHT)) // Low junction adjacent to cone stack
                .lineToLinearHeading(addClawOffset(CONE_STACK_POS_RED_LEFT))
                .build();

        waitForStart();
        if (isStopRequested()) { return; }

        runTrajectory(trajectories[0]);  // approach forward high junction

        runTrajectory(trajectories[2]); // Go to cone stack

        runTrajectory(trajectories[1]); // Low junction adjacent to cone stack
    }

    public void runTrajectory(Trajectory trajectory) {
        if (!opModeIsActive()) { return; }

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
