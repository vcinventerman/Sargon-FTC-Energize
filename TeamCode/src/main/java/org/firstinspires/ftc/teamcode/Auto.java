package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TeamConf.CONE_STACK_POSITIONS;
import static org.firstinspires.ftc.teamcode.TeamConf.FIELD_BEARING_NORTH;
import static org.firstinspires.ftc.teamcode.TeamConf.FIELD_BEARING_SOUTH;
import static org.firstinspires.ftc.teamcode.TeamConf.JUNCTIONS;
import static org.firstinspires.ftc.teamcode.TeamConf.ROBOT_CLAW_OFFSET;
import static org.firstinspires.ftc.teamcode.TeamConf.START_POSITIONS;
import static org.firstinspires.ftc.teamcode.TeamConf.TILE_SIZE;
import static org.firstinspires.ftc.teamcode.TeamConf.nop;
import static org.firstinspires.ftc.teamcode.TeamConf.sleep;
import static org.firstinspires.ftc.teamcode.TeamConf.stringToStartPose;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.karrmedia.ftchotpatch.Supervised;
import com.karrmedia.ftchotpatch.SupervisedOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.Semaphore;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.stream.Collectors;

//todo: MUST construct trajectories in another thread and keep winch ticking in any way
@Supervised(name="?Auto", group="!CompAuto", autonomous=true, linear=true, variations={"RedLeft", "RedRight", "BlueLeft", "BlueRight"}, next="TeleOp")
public class Auto extends SupervisedOpMode {
    RobotA robot;
    AprilTagDetector detector;
    Trajectory[] trajectories = null;

    static Trajectory[][] parkingTrajectories; // 4 locations x 3 scan results
    static Trajectory[][] allTrajectories = new Trajectory[4][10];
    static BulkTrajectoryBuilder builder = null;

    static Thread trajCreationThread = new Thread(Auto::createTrajectories);
    static AtomicBoolean ranTrajBuilder = new AtomicBoolean(false);
    static Semaphore trajBuilderDone = new Semaphore(0, true);

    // Run when the RobotController app is first started, and after the OpMode is stopped and started
    public Auto() {
        // Creating the trajectories takes a long time, so perform it before the OpMode is started and cache the result
        if (!ranTrajBuilder.get()) {
            ranTrajBuilder.set(true);
            trajCreationThread = new Thread(Auto::createTrajectories);
            trajCreationThread.start();
        }
    }

    // Code that runs when the INIT button is pressed
    public void init() {
        robot = new RobotA(hardwareMap, stringToStartPose(variation));

        // Construct and run AprilTag detector, but don't read from it yet: the field has not yet been randomized
        detector = new AprilTagDetector(hardwareMap, Arrays.asList(21, 22, 23));

        if (!ranTrajBuilder.get()) {
            ranTrajBuilder.set(true);
            trajCreationThread = new Thread(Auto::createTrajectories);
            trajCreationThread.setPriority(Thread.MAX_PRIORITY);
            trajCreationThread.start();
        }
        else {
            try { trajBuilderDone.acquire(); } catch(InterruptedException e) {
                nop(); }
            trajBuilderDone.release();
        }
    }

    public void initLoop() {

    }

    public void start() {
        elapsedRuntime.reset();

        detector.detect();

        // Keep trying to detect for a while
        while (elapsedRuntime.milliseconds() < 1000 && !detector.done()) {
            Thread.yield();
            detector.detect();
        }

        assignTrajectories(detector.getTagSeenOrDefault(2));


        telemetry.addData("!", "Found tag " + detector.tagSeen);
        telemetry.update();

        // Close on preload cone
        robot.slide.setClawState(LinearSlideA.CLAW_POS_CLOSED);
        robot.slide.setCurrentWinchTarget(LinearSlideA.SLIDE_POS_HIGH);

        // Run to a high junction to drop the first cone
        //todo: combine into one
        runTrajectory(trajectories[0]);
        runTrajectory(trajectories[1]);

        robot.slide.setClawState(LinearSlideA.CLAW_POS_OPEN);
        //robot.slide.setCurrentWinchTarget(LinearSlideA.SLIDE_POS_BOTTOM);
        robot.slide.goToNextConeStackHeight();

        int CONE_TRANSFER_ITERATIONS = 5;
        for (int i = 0; i < CONE_TRANSFER_ITERATIONS; i++) {
            // Run to cone stack
            runTrajectory(trajectories[3]);

            robot.slide.setClawState(LinearSlideA.CLAW_POS_CLOSED);
            robot.slide.setCurrentWinchTarget(LinearSlideA.SLIDE_POS_HIGH);

            // Wait so the cone doesn't knock over the stack when it drives away
            robot.slide.waitToPassConeStack();

            // Run to drop junction
            runTrajectory(trajectories[4]);

            robot.slide.setClawState(LinearSlideA.CLAW_POS_OPEN);
            //robot.slide.setCurrentWinchTarget(LinearSlideA.SLIDE_POS_BOTTOM);
            robot.slide.goToNextConeStackHeight();
        }

        // Park
        runTrajectory(trajectories[5]);



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
        while (robot.drive.isBusy() && opModeIsActive()) {
            robot.update();

            Pose2d poseEstimate = robot.drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

    // Code that runs repeatedly after the PLAY button is pressed (optional)
    public void loop() {
        //todo: pathfinding and state machine
    }

    // Code that runs when the OpMode is stopped (optional)
    public void stop() {

    }

    static public Pose2d getSignalSpot(String variation, int tag) {

        if (variation.contains("Red")) {
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
        return null;
        //return robot.drive.getPoseEstimate();
    }

    static Vector2d getNearestJunction(Vector2d pos) {
        //todo: include claw pos in calculation (check centerConeOverJunction)
        Vector2d nearest = JUNCTIONS.get(0);
        double nearestDist = 99999;
        for (Vector2d j : JUNCTIONS) {
            double dist = j.distTo(pos);

            if (dist < nearestDist) {
                nearest = j;
                nearestDist = dist;
            }
        }
        return nearest;
    }
    static Vector2d getNearestJunction(Pose2d pos) { return getNearestJunction(pos.vec()); }

    Trajectory trajToNearestJunction(Pose2d robotPos) {
        //todo: pole avoidance, offset to claw
        TrajectoryBuilder builder = new TrajectoryBuilder(robotPos, SampleMecanumDrive.VEL_CONSTRAINT, SampleMecanumDrive.ACCEL_CONSTRAINT);
        builder.splineToSplineHeading(new Pose2d(getNearestJunction(robotPos), robotPos.getHeading()), 0);

        // This part takes a while
        return builder.build();
    }

    static int trajSlot = 0;
    static void add(List<Trajectory> list) {
        allTrajectories[0][trajSlot] = list.get(0);
        allTrajectories[1][trajSlot] = list.get(1);
        allTrajectories[2][trajSlot] = list.get(2);
        allTrajectories[3][trajSlot] = list.get(3);
        trajSlot++;
    }

    static Vector2d addClawOffsetVec(Vector2d pos, double heading) {
        return addClawOffset(new Pose2d(pos, heading)).vec();
    }

    static Vector2d addClawOffsetVec(Vector2d pos) {
        return addClawOffset(new Pose2d(pos, 0.0)).vec();
    }

    static Pose2d addClawOffset(Pose2d pos) {
        return pos.plus(new Pose2d(ROBOT_CLAW_OFFSET.rotated(pos.getHeading()), 0));
    }

    static List<Pose2d> addClawOffsetList(List<Pose2d> poses) {
        return poses.stream().map(Auto::addClawOffset).collect(Collectors.toList());
    }

    static List<Vector2d> addClawOffsetListVec(List<Vector2d> poses) {
        return poses.stream().map(Auto::addClawOffsetVec).collect(Collectors.toList());
    }

    static void makeParkingTrajectories(BulkTrajectoryBuilder builder) {
        parkingTrajectories = builder.multiApply((trajBuilder, version, code) -> trajBuilder.lineToLinearHeading(getSignalSpot(version, code)), 3);
    }

    static void createTrajectories() {
        //allTrajectories.add((List<Trajectory>) new ArrayList<Trajectory>());

        builder = new BulkTrajectoryBuilder(START_POSITIONS, t -> TrajectoryCache.set(t));

        // trajectories[0]: Get the signal cone out of the way
        add(builder.apply((t, p) -> t.forward(TILE_SIZE * (5.0 / 2.0))));

        // trajectories[1]: Reverse back inline with cone stack
        //add(builder.apply((t, p) -> t.back(TILE_SIZE / 2.0)));

        // trajectories[1]: Go to high junction to drop preload cone
        add(builder.apply((t, pos) -> t.splineToSplineHeading(addClawOffset(new Pose2d(getNearestJunction(pos), pos.getHeading())), 0)));

        // trajectories[2]: Go to cone stack
        //todo: line up with claw, don't hit stack
        add(builder.splineToSplineHeadingVec(addClawOffsetListVec(CONE_STACK_POSITIONS)));

        List<Vector2d> targetAutoJunctions = Arrays.asList(
                TeamConf.JUNCTIONS.get(2), TeamConf.JUNCTIONS.get(3),
                TeamConf.JUNCTIONS.get(2), TeamConf.JUNCTIONS.get(3));
        targetAutoJunctions.forEach(Auto::addClawOffsetVec);

        // trajectories[3]: Go to drop point (loop part 1)
        //todo: line up with claw, don't hit junction, tune based on drive speed for max points
        add(builder.splineToSplineHeadingVec(targetAutoJunctions));

        // trajectories[4]: Return to cone stack to repeat pickup (loop part 2)
        //todo: line up with claw, don't hit junction, tune based on drive speed for max points
        add(builder.splineToSplineHeadingVec(addClawOffsetListVec(CONE_STACK_POSITIONS)));

        // Park
        makeParkingTrajectories(builder);

        trajBuilderDone.release();
    }

    // Pick a system of trajectories to follow based on what OpMode and barcode were picked
    void assignTrajectories(int tag) {
        // Wait up to 5 seconds for the creation thread to die off
        //todo: incremental, wait on non-parking trajectories first
        try { trajCreationThread.join(500); } catch (Exception e) {
            nop();
        }

        if (variation.equals("RedLeft")) {
            trajectories = allTrajectories[0];
            trajectories[5] = parkingTrajectories[0][tag];
        }
        else if (variation.equals("RedRight")) {
            trajectories = allTrajectories[1];
            trajectories[5] = parkingTrajectories[1][tag];
        }
        else if (variation.equals("BlueLeft")) {
            trajectories = allTrajectories[2];
            trajectories[5] = parkingTrajectories[2][tag];
        }
        else if (variation.equals("BlueRight")) {
            trajectories = allTrajectories[3];
            trajectories[5] = parkingTrajectories[3][tag];
        }
    }
}
