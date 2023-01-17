package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.TeamConf.FIELD_BEARING_NORTH;
import static org.firstinspires.ftc.teamcode.TeamConf.FIELD_BEARING_SOUTH;
import static org.firstinspires.ftc.teamcode.TeamConf.START_POS_BLUE_LEFT;
import static org.firstinspires.ftc.teamcode.TeamConf.START_POS_BLUE_RIGHT;
import static org.firstinspires.ftc.teamcode.TeamConf.START_POS_RED_LEFT;
import static org.firstinspires.ftc.teamcode.TeamConf.START_POS_RED_RIGHT;
import static org.firstinspires.ftc.teamcode.TeamConf.TILE_SIZE;
import static org.firstinspires.ftc.teamcode.TeamConf.sleep;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.karrmedia.ftchotpatch.Supervised;
import com.karrmedia.ftchotpatch.SupervisedOpMode;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

//todo: MUST construct trajectories in another thread or keep winch ticking in any way
@Supervised(name="Old?Auto", group="!CompAuto", autonomous=true, linear=true, variations={"RedLeft", "RedRight", "BlueLeft", "BlueRight"}, next="TeleOp")
public class OldAuto extends SupervisedOpMode {
    RobotA robot;
    AprilTagDetector detector;










    Trajectory[] trajectories = null;
    // Code that runs when the INIT button is pressed (mandatory)
    public void init() {
        robot = new RobotA(hardwareMap,false);
        // Detect AprilTags
        detector = new AprilTagDetector(hardwareMap, List.of(21, 22, 23));
    }
    public void initLoop() {
        detector.detect();
        sleep(100);
        if (detector.done() && trajectories == null) {
            createTrajectories(detector.getTagSeenOrDefault(2));
        }
    }
    public void start() {
        elapsedRuntime.reset();
        // Keep trying to detect for a while in case we didn't have any time after init
        while (elapsedRuntime.milliseconds() < 3000 && !detector.done()) {
            detector.detect();
            sleep(100);
        }
        if (trajectories == null) {
            createTrajectories(detector.getTagSeenOrDefault(2));
        }
        telemetry.addData("!", "Found tag " + Integer.toString(detector.tagSeen));
        telemetry.update();
        robot.slide.setClawState(robot.slide.CLAW_POS_OPEN);
        runTrajectory(trajectories[0]);
        runTrajectory(trajectories[1]);
        runTrajectory(trajectories[4]);
        /*
        // Grab the preload cone
        robot.slide.setClawState(robot.slide.CLAW_POS_OPEN);
        robot.slide.setClawState(robot.slide.CLAW_POS_CLOSED);
        robot.slide.setCurrentWinchTarget(robot.slide.SLIDE_POS_HIGH);
        // Get the signal sleeve out of the way
        runTrajectory(trajectories[0]);
        runTrajectory(trajectories[1]);
        robot.slide.waitToReachWinchTarget();
        // Go above the high junction
        runTrajectory(trajectories[2]);
        // Drop the cone and start going back down
        robot.slide.setClawState(robot.slide.CLAW_POS_OPEN);
        robot.slide.setCurrentWinchTarget(robot.slide.SLIDE_POS_BOTTOM);
        // Go to the indicated parking space
        runTrajectory(trajectories[3]);
        runTrajectory(trajectories[4]);
        robot.slide.waitToReachWinchTarget();
        */
// fix waitToReachWinchTarget
    }
    public void runTrajectory(Trajectory trajectory) {
        robot.drive.followTrajectoryAsync(trajectory);
        while (robot.drive.isBusy()) {
            robot.update();
        }
    }
    // Code that runs repeatedly after the PLAY button is pressed (optional)
    public void loop() {
        //todo: pathfinding and state machine
    }
    // Code that runs when the OpMode is stopped (optional)
    public void stop() {
    }
    // Code that runs after this OpMode is dynamically updated
    public void hotpatch() {
    }
    public TeamConf.Alliance getAlliance() {
        if (variation.contains("Red")) {
            return TeamConf.Alliance.RED;
        }
        else if (variation.contains("Blue")) {
            return TeamConf.Alliance.BLUE;
        }
        else {
            return TeamConf.Alliance.NONE;
        }
    }
    public TrajectorySequenceBuilder faceToCenter(TrajectorySequenceBuilder seq) {
        if (getAlliance() == TeamConf.Alliance.BLUE) {
            //seq.splineToLinearHeading(robot.drive.getPoseEstimate(), FIELD_BEARING_SOUTH);
            seq.lineToLinearHeading(new Pose2d(robot.drive.getPoseEstimate().getX(), robot.drive.getPoseEstimate().getY(), FIELD_BEARING_SOUTH));
        }
        else {
            //seq.splineToLinearHeading(robot.drive.getPoseEstimate(), FIELD_BEARING_NORTH);
            seq.lineToLinearHeading(new Pose2d(robot.drive.getPoseEstimate().getX(), robot.drive.getPoseEstimate().getY(), FIELD_BEARING_NORTH));
        }
        return seq;
    }
    public Pose2d getSignalSpot(int tag) {
        if (getAlliance() == TeamConf.Alliance.RED) {
            // Tile F5 start
            if (variation.contains("Right")) {
                if (tag == 1) {
                    return new Pose2d(TILE_SIZE / 2, -TILE_SIZE / 2, FIELD_BEARING_NORTH);
                } else if (tag == 2) {
                    return new Pose2d(TILE_SIZE * (3.0/2.0), -TILE_SIZE / 2, FIELD_BEARING_NORTH);
                } else if (tag == 3) {
                    return new Pose2d(TILE_SIZE * (5.0/2.0), -TILE_SIZE / 2, FIELD_BEARING_NORTH);
                }
            }
            // Tile F2 start
            else {
                if (tag == 1) {
                    return new Pose2d(-TILE_SIZE * (5.0/2.0), -TILE_SIZE / 2, FIELD_BEARING_NORTH);
                } else if (tag == 2) {
                    return new Pose2d(-TILE_SIZE * (3.0/2.0), -TILE_SIZE / 2, FIELD_BEARING_NORTH);
                } else if (tag == 3) {
                    return new Pose2d(-TILE_SIZE / 2, -TILE_SIZE / 2, FIELD_BEARING_NORTH);
                }
            }
        }
        else {
            // Tile A2 start
            if (variation.contains("Right")) {
                if (tag == 1) {
                    return new Pose2d(-TILE_SIZE / 2, TILE_SIZE / 2, FIELD_BEARING_SOUTH);
                } else if (tag == 2) {
                    return new Pose2d(-TILE_SIZE * (3.0/2.0), TILE_SIZE / 2, FIELD_BEARING_SOUTH);
                } else if (tag == 3) {
                    return new Pose2d(-TILE_SIZE * (5.0/2.0), TILE_SIZE / 2, FIELD_BEARING_SOUTH);
                }
            }
            // Tile A5 start
            else {
                if (tag == 1) {
                    return new Pose2d(TILE_SIZE * (5.0/2.0), TILE_SIZE / 2, FIELD_BEARING_SOUTH);
                } else if (tag == 2) {
                    return new Pose2d(TILE_SIZE * (3.0/2.0), TILE_SIZE / 2, FIELD_BEARING_SOUTH);
                } else if (tag == 3) {
                    return new Pose2d(TILE_SIZE / 2, TILE_SIZE / 2, FIELD_BEARING_SOUTH);
                }
            }
        }
        // Do nothing on error
        return robot.drive.getPoseEstimate();
    }
    // Returns the angle the robot needs to turn to before moving forward and the point it needs to drive to
    //todo: claw probably isn't even in the center
    Pose2d centerConeOverJunction(Vector2d junctionPos, Vector2d robotPos) {
        //Vector2d robotPos = robot.drive.getPoseEstimate().vec();
        double angle = Math.atan((junctionPos.getY() - robotPos.getY()) / (junctionPos.getX() - robotPos.getX()));
        double hyp = Math.sqrt(Math.pow(junctionPos.getX() - robotPos.getX(), 2) + Math.pow(junctionPos.getY() - robotPos.getY(), 2))
                - 10.5; // distance from center of robot to claw which is on a horizontal center axis
        double offsetX = hyp * Math.cos(angle);
        double offsetY = hyp * Math.sin(angle);
        return new Pose2d(robotPos.plus(new Vector2d(offsetX, offsetY)), angle);
    }
    void createTrajectories(int tag) {
        trajectories = new Trajectory[5];
        if (variation.equals("RedLeft")) {
            Pose2d startPos = START_POS_RED_LEFT;
            robot.drive.setPoseEstimate(startPos);
            trajectories[0] = robot.drive.trajectoryBuilder(startPos)
                    .forward(TILE_SIZE * (5.0 / 2.0)) // Get the signal cone out of the way
                    .build();
            // Wait for the winch to rise all the way
            trajectories[1] = robot.drive.trajectoryBuilder(trajectories[0].end())
                    .back(TILE_SIZE / 2)
                    .build();
            trajectories[2] = robot.drive.trajectoryBuilder(trajectories[1].end())
                    .lineToLinearHeading(centerConeOverJunction(new Vector2d(TILE_SIZE, 0), trajectories[1].end().vec()))
                    .build();
            // Release cone
            trajectories[3] = robot.drive.trajectoryBuilder(trajectories[2].end())
                    .strafeTo(new Vector2d(startPos.getX(), startPos.getY() + TILE_SIZE * 2))
                    .build();
            trajectories[4] = robot.drive.trajectoryBuilder(trajectories[1].end())
                    .lineToLinearHeading(getSignalSpot(tag))
                    .build();
        }
        else if (variation.equals("RedRight")) {
            Pose2d startPos = START_POS_RED_RIGHT;
            robot.drive.setPoseEstimate(startPos);
            trajectories[0] = robot.drive.trajectoryBuilder(startPos)
                    .forward(TILE_SIZE * (5.0 / 2.0)) // Get the signal cone out of the way
                    .build();
            // Wait for the winch to rise all the way
            trajectories[1] = robot.drive.trajectoryBuilder(trajectories[0].end())
                    .back(TILE_SIZE / 2)
                    .build();
            trajectories[2] = robot.drive.trajectoryBuilder(trajectories[1].end())
                    .lineToLinearHeading(centerConeOverJunction(new Vector2d(TILE_SIZE, 0), trajectories[1].end().vec()))
                    .build();
            // Release cone
            trajectories[3] = robot.drive.trajectoryBuilder(trajectories[2].end())
                    .strafeTo(new Vector2d(startPos.getX(), startPos.getY() + TILE_SIZE * 2))
                    .build();
            trajectories[4] = robot.drive.trajectoryBuilder(trajectories[1].end())
                    .lineToLinearHeading(getSignalSpot(tag))
                    .build();
        }
        else if (variation.equals("BlueLeft")) {
            Pose2d startPos = START_POS_BLUE_LEFT;
            robot.drive.setPoseEstimate(startPos);
            trajectories[0] = robot.drive.trajectoryBuilder(startPos)
                    .forward(TILE_SIZE * (5.0 / 2.0)) // Get the signal cone out of the way
                    .build();
            // Wait for the winch to rise all the way
            trajectories[1] = robot.drive.trajectoryBuilder(trajectories[0].end())
                    .back(TILE_SIZE / 2)
                    .build();
            trajectories[2] = robot.drive.trajectoryBuilder(trajectories[1].end())
                    .lineToLinearHeading(centerConeOverJunction(new Vector2d(TILE_SIZE, 0), trajectories[1].end().vec()))
                    .build();
            // Release cone
            trajectories[3] = robot.drive.trajectoryBuilder(trajectories[2].end())
                    .strafeTo(new Vector2d(startPos.getX(), startPos.getY() - TILE_SIZE * 2))
                    .build();
            trajectories[4] = robot.drive.trajectoryBuilder(trajectories[1].end())
                    .lineToLinearHeading(getSignalSpot(tag))
                    .build();
        }
        else { // if (variation.equals("BlueRightAuto")) {
            Pose2d startPos = START_POS_BLUE_RIGHT;
            robot.drive.setPoseEstimate(startPos);
            trajectories[0] = robot.drive.trajectoryBuilder(startPos)
                    .forward(TILE_SIZE * (5.0 / 2.0)) // Get the signal cone out of the way
                    .build();
            // Wait for the winch to rise all the way
            trajectories[1] = robot.drive.trajectoryBuilder(trajectories[0].end())
                    .back(TILE_SIZE / 2)
                    .build();
            trajectories[2] = robot.drive.trajectoryBuilder(trajectories[1].end())
                    .lineToLinearHeading(centerConeOverJunction(new Vector2d(TILE_SIZE, 0), trajectories[1].end().vec()))
                    .build();
            // Release cone
            trajectories[3] = robot.drive.trajectoryBuilder(trajectories[2].end())
                    .strafeTo(new Vector2d(startPos.getX(), startPos.getY() - TILE_SIZE * 2))
                    .build();
            trajectories[4] = robot.drive.trajectoryBuilder(trajectories[1].end())
                    .lineToLinearHeading(getSignalSpot(tag))
                    .build();
        }
    }
}