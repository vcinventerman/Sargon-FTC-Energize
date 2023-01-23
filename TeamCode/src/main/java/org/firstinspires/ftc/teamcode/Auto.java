package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TeamConf.CONE_STACK_POSITIONS;
import static org.firstinspires.ftc.teamcode.TeamConf.FIELD_BEARING_NORTH;
import static org.firstinspires.ftc.teamcode.TeamConf.FIELD_BEARING_SOUTH;
import static org.firstinspires.ftc.teamcode.TeamConf.JUNCTIONS;
import static org.firstinspires.ftc.teamcode.TeamConf.ROBOT_CLAW_OFFSET;
import static org.firstinspires.ftc.teamcode.TeamConf.START_POSITIONS;
import static org.firstinspires.ftc.teamcode.TeamConf.TILE_SIZE;
import static org.firstinspires.ftc.teamcode.TeamConf.getRobot;
import static org.firstinspires.ftc.teamcode.TeamConf.nop;
import static org.firstinspires.ftc.teamcode.TeamConf.stringToStartPose;
import static org.firstinspires.ftc.teamcode.util.PathTools.addClawOffset;
import static org.firstinspires.ftc.teamcode.util.PathTools.addClawOffsetList;
import static org.firstinspires.ftc.teamcode.util.PathTools.getNearestJunction;
import static org.firstinspires.ftc.teamcode.util.PathTools.getSignalSpot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.karrmedia.ftchotpatch.Supervised;
import com.karrmedia.ftchotpatch.SupervisedOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.PathTools;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

//todo: MUST construct trajectories in another thread and keep winch ticking in any way
@Supervised(name="?Auto", group="!CompAuto", autonomous=true, linear=true, variations={"RedLeft", "RedRight", "BlueLeft", "BlueRight"}, next="TeleOp")
@Config
public class Auto extends SupervisedOpMode {
    RobotA robot;
    AprilTagDetector detector;
    Trajectory[] trajectories = null;

    static Trajectory[][] parkingTrajectories; // 4 locations x 3 scan results
    static Trajectory[][] allTrajectories = new Trajectory[4][10];
    static BulkTrajectoryBuilder builder = null;

    static public Boolean FALLBACK_AUTO = false;

    static public int targetAutoJunctionHeight = LinearSlideA.SLIDE_POS_MED;

    // Run when the RobotController app is first started, and after the OpMode is stopped and started
    public Auto() {
        // Creating the trajectories takes a long time, so perform it before the OpMode is started and cache the result
        /*if (!ranTrajBuilder.get()) {
            ranTrajBuilder.set(true);
            trajCreationThread = new Thread(Auto::createTrajectories);
            trajCreationThread.start();
        }*/
    }

    // Code that runs when the INIT button is pressed
    public void init() {
        robot = getRobot(hardwareMap, stringToStartPose(variation));

        // Construct and run AprilTag detector, but don't read from it yet: the field has not yet been randomized
        detector = new AprilTagDetector(hardwareMap, Arrays.asList(21, 22, 23));

        if (builder == null) {
            createTrajectories();
        }

        /*if (!ranTrajBuilder.get()) {
            ranTrajBuilder.set(true);
            trajCreationThread = new Thread(Auto::createTrajectories);
            trajCreationThread.setPriority(Thread.MAX_PRIORITY);
            trajCreationThread.start();
        }
        else {
            try { trajBuilderDone.acquire(); } catch(InterruptedException e) {
                nop(); }
            trajBuilderDone.release();
        }*/
    }

    public void initLoop() {

    }

    public void waitToPassConeStack() {
        int safeHeight = robot.slide.CONE_STACK_HEIGHTS.get(robot.slide.coneStackState - 1) + robot.slide.CONE_STACK_OFFSET;

        if (robot.slide.currentWinchTarget <= robot.slide.CONE_STACK_HEIGHTS.get(robot.slide.coneStackState - 1)) {
            RobotLog.e("Invalid invocation of waitToPassConeStack!");
        }

        while (robot.slide.winch.getCurrentPosition() < safeHeight) {
            robot.update();
        }
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
        robot.slide.setClawState(LinearSlideA.CLAW_POS_OPEN);
        robot.slide.setClawState(LinearSlideA.CLAW_POS_CLOSED);
        robot.slide.setClawState(LinearSlideA.CLAW_POS_CLOSED);
        robot.slide.setCurrentWinchTarget(targetAutoJunctionHeight);

        // Run to a high junction to drop the first cone
        //todo: combine into one
        runTrajectory(trajectories[0]);
        runTrajectory(trajectories[1]);

        robot.slide.setClawState(LinearSlideA.CLAW_POS_OPEN);
        //robot.slide.setCurrentWinchTarget(LinearSlideA.SLIDE_POS_BOTTOM);
        robot.slide.goToNextConeStackHeight();

        int CONE_TRANSFER_ITERATIONS = 5;
        for (int i = 0; i < CONE_TRANSFER_ITERATIONS; i++) {
            // Run to junction
            runTrajectory(trajectories[3]);

            robot.slide.setClawState(LinearSlideA.CLAW_POS_CLOSED);
            robot.slide.setCurrentWinchTarget(targetAutoJunctionHeight);

            // Wait so the cone doesn't knock over the stack when it drives away
            waitToPassConeStack();

            // Run to drop junction
            runTrajectory(trajectories[4]);

            robot.slide.setClawState(LinearSlideA.CLAW_POS_OPEN);
            //robot.slide.setCurrentWinchTarget(LinearSlideA.SLIDE_POS_BOTTOM);
            robot.slide.goToNextConeStackHeight();
        }

        // Park
        runTrajectory(trajectories[5]);
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


    static int trajSlot = 0;
    static void add(List<Trajectory> list) {
        allTrajectories[0][trajSlot] = list.get(0);
        allTrajectories[1][trajSlot] = list.get(1);
        allTrajectories[2][trajSlot] = list.get(2);
        allTrajectories[3][trajSlot] = list.get(3);
        trajSlot++;
    }



    static void makeParkingTrajectories(BulkTrajectoryBuilder builder) {
        parkingTrajectories = builder.multiApply((trajBuilder, version, code) -> trajBuilder.lineToLinearHeading(getSignalSpot(version, code)), 3);
    }

    static void createTrajectories() {
        //allTrajectories.add((List<Trajectory>) new ArrayList<Trajectory>());

        builder = new BulkTrajectoryBuilder(START_POSITIONS.stream().collect(Collectors.toList()), t -> TrajectoryCache.set(t));

        // trajectories[0]: Get the signal cone out of the way
        add(builder.apply((t, p) -> t.forward(TILE_SIZE * (5.0 / 2.0))));

        // trajectories[1]: Reverse back inline with cone stack
        //add(builder.apply((t, p) -> t.back(TILE_SIZE / 2.0)));

        // trajectories[1]: Go to high junction to drop preload cone
        add(builder.apply((t, pos) -> t.splineToSplineHeading(addClawOffset(new Pose2d(getNearestJunction(pos), pos.getHeading())), 0)));

        // trajectories[2]: Go to cone stack
        //todo: line up with claw, don't hit stack
        add(builder.splineToSplineHeading(addClawOffsetList(CONE_STACK_POSITIONS)));

        List<Pose2d> targetAutoJunctions = Arrays.asList(
                new Pose2d(TeamConf.JUNCTIONS.get(7).getX(), TeamConf.JUNCTIONS.get(7).getY(), 0),
                new Pose2d(TeamConf.JUNCTIONS.get(5).getX(), TeamConf.JUNCTIONS.get(5).getY(), Math.PI),
                new Pose2d(TeamConf.JUNCTIONS.get(4).getX(), TeamConf.JUNCTIONS.get(4).getY(), Math.PI),
                new Pose2d(TeamConf.JUNCTIONS.get(6).getX(), TeamConf.JUNCTIONS.get(6).getY(), 0));
        targetAutoJunctions.forEach(PathTools::addClawOffset);

        // trajectories[3]: Go to drop point (loop part 1)
        //todo: line up with claw, don't hit junction, tune based on drive speed for max points
        add(builder.splineToSplineHeading(targetAutoJunctions));

        // trajectories[4]: Return to cone stack to repeat pickup (loop part 2)
        //todo: line up with claw, don't hit junction, tune based on drive speed for max points
        add(builder.splineToSplineHeading(addClawOffsetList(CONE_STACK_POSITIONS)));

        // Park
        makeParkingTrajectories(builder);
    }

    // Pick a system of trajectories to follow based on what OpMode and barcode were picked
    void assignTrajectories(int tag) {

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
