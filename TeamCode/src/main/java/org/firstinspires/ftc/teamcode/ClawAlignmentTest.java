package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TeamConf.FIELD_BEARING_NORTH;
import static org.firstinspires.ftc.teamcode.TeamConf.JUNCTIONS;
import static org.firstinspires.ftc.teamcode.TeamConf.START_POS_RED_RIGHT;
import static org.firstinspires.ftc.teamcode.TeamConf.TILE_SIZE;
import static org.firstinspires.ftc.teamcode.TeamConf.getRobot;
import static org.firstinspires.ftc.teamcode.TeamConf.stringToStartPose;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.MecanumDriveCancelable.ACCEL_CONSTRAINT;
import static org.firstinspires.ftc.teamcode.drive.MecanumDriveCancelable.VEL_CONSTRAINT;
import static org.firstinspires.ftc.teamcode.util.PathTools.getTrajBuilder;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.PathTools;

@Config
@Autonomous
public class ClawAlignmentTest extends LinearOpMode {
    public static double startX = START_POS_RED_RIGHT.getX();
    public static double startY = START_POS_RED_RIGHT.getY();
    public static double startBearing = START_POS_RED_RIGHT.getHeading();

    public static double endBearing = 0;//FIELD_BEARING_NORTH + Math.PI / 4;

    public static boolean addClawOffset = true;

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
        Pose2d startPose = new Pose2d(startX, startY, startBearing);

        Pose2d endPose = new Pose2d(TILE_SIZE, 0,  endBearing);

        Pose2d adjustedEnd = PathTools.addClawOffset(endPose);

        if (!addClawOffset) { adjustedEnd = endPose; }

        Trajectory traj = getTrajBuilder(startPose).lineToLinearHeading(adjustedEnd).build();

        robot = getRobot(hardwareMap);
        robot.setPose(startPose);

        Trajectory trajs[] = new Trajectory[8];

        //seqBuilder.addTrajectory(getTrajBuilder(startPose).lineToLinearHeading(new Pose2d(endPose.getX(), endPose.getY(), PI * (-2) / 4.0)).build());

        for (int i = 0; i < 8; i++) {
            trajs[i] = getTrajBuilder(
                    PathTools.addClawOffset(new Pose2d(endPose.getX(), endPose.getY(), PI * (i - 1) / 4.0))
            ).lineToLinearHeading(PathTools.addClawOffset(new Pose2d(endPose.getX(), endPose.getY(), PI * (i) / 4.0))).build();
        }

        waitForStart();

        robot.followTrajectory(traj);

        robot.waitToComplete();

        for (int i = 0; opModeIsActive(); i = (i + 1) % 8) {
            robot.followTrajectory(trajs[i]);
        }
    }
}
