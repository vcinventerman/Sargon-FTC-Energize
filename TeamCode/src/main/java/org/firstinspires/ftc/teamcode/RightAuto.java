package org.firstinspires.ftc.teamcode;

import static com.outoftheboxrobotics.photoncore.PhotonCore.EXPANSION_HUB;
import static org.firstinspires.ftc.teamcode.TeamConf.CONE_STACK_POS_RED_RIGHT;
import static org.firstinspires.ftc.teamcode.TeamConf.FIELD_BEARING_EAST;
import static org.firstinspires.ftc.teamcode.TeamConf.FIELD_BEARING_NORTH;
import static org.firstinspires.ftc.teamcode.TeamConf.JUNCTIONS;
import static org.firstinspires.ftc.teamcode.TeamConf.START_POS_RED_RIGHT;
import static org.firstinspires.ftc.teamcode.TeamConf.TILE_SIZE;
import static org.firstinspires.ftc.teamcode.TeamConf.getRobot;
import static org.firstinspires.ftc.teamcode.TeamConf.sleep;
import static org.firstinspires.ftc.teamcode.TeamConf.stringToStartPose;
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
import com.karrmedia.ftchotpatch.SupervisedOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.teamcode.util.PathTools;

@Config
@Autonomous(group="!CompAuto")
public class RightAuto extends LinearOpMode {
    public static Pose2d startPose = START_POS_RED_RIGHT;

    Robot robot;
    AprilTagDetector detector;

    public static double comb1_x = 8;
    public static double comb1_y = -8;
    public static double comb2_x = -17.5;
    public static double comb2_y = -7;
    public static double comb3_x = 1.58;
    public static double comb3_y = -12.5;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = getRobot(hardwareMap);
        robot.setPose(startPose);

        detector = new AprilTagDetector(hardwareMap, List.of(21, 22, 23));
        detector.detect();

        // Initialize the claw so that it's ready to move when started
        robot.slide.claw.turnToAngle(robot.slide.p.CLAW_POS_OPEN);
        robot.slide.claw.turnToAngle(robot.slide.p.CLAW_POS_CLOSED); // Grab the preload cone

        calibrateSlide();

        // Creating these trajectories will take a while
        Trajectory[] trajectories = new Trajectory[10];

        Pose2d junc1Pos = combinePose(JUNCTIONS.get(1), comb1_x, comb1_y, FIELD_BEARING_NORTH + Math.PI / 4);
        Pose2d coneStackPos = combinePose(CONE_STACK_POS_RED_RIGHT.vec(), comb2_x, comb2_y, CONE_STACK_POS_RED_RIGHT.getHeading());
        Pose2d junc2Pos = combinePose(JUNCTIONS.get(2), comb3_x, comb3_y, FIELD_BEARING_NORTH);

        trajectories[0] = getTrajBuilder(START_POS_RED_RIGHT).lineTo(new Vector2d(TILE_SIZE * (3.0 / 4.0), START_POS_RED_RIGHT.getY() + 6)).build();
        trajectories[1] = getTrajBuilder(trajectories[0].end()).splineToLinearHeading(junc1Pos, FIELD_BEARING_NORTH + PI / 4).build();

        trajectories[2] = getTrajBuilder(trajectories[1].end()).strafeRight(TILE_SIZE * 1.0 / 2.0).build();
        trajectories[3] = getTrajBuilder(trajectories[2].end()).forward(TILE_SIZE).build();
        trajectories[4] = getTrajBuilder(trajectories[3].end()).lineToLinearHeading(TILE_SIZE * 5.0/6.0).build();
        trajectories[5] = getTrajBuilder(trajectories[4].end()).strafeLeft(TILE_SIZE * 5.0/6.0).build();
        trajectories[6] = getTrajBuilder(trajectories[5].end()).strafeLeft(TILE_SIZE * 5.0/6.0).build();


        /*trajectories[0] = getTrajBuilder(START_POS_RED_RIGHT) // Forward junction
                .splineToConstantHeading(new Vector2d(TILE_SIZE * (3.0 / 4.0), START_POS_RED_RIGHT.getY() + 6), FIELD_BEARING_NORTH)

                //.splineToSplineHeading(new Pose2d(TILE_SIZE * (1.0 / 4.0), START_POS_RED_RIGHT.getY() + TILE_SIZE, FIELD_BEARING_NORTH + PI/4), FIELD_BEARING_NORTH + PI/4)
                .splineTo(junc1Pos.vec(), FIELD_BEARING_NORTH + PI / 4)
                .build();*/

        trajectories[1] = getTrajBuilder(junc1Pos) // Cone stack
                .back(TILE_SIZE / 8)
                .splineToSplineHeading(new Pose2d(TILE_SIZE * (1.0 / 2.0), -TILE_SIZE * (3.0 / 2), FIELD_BEARING_NORTH), PI * (7.0 / 4.0))
                .splineToConstantHeading(new Vector2d(TILE_SIZE * (2.5 / 4.0), START_POS_RED_RIGHT.getY() + TILE_SIZE * 3.0/2.0), FIELD_BEARING_NORTH)
                //.splineToSplineHeading(new Pose2d(TILE_SIZE * (2.5 / 4.0), START_POS_RED_RIGHT.getY() + TILE_SIZE * (3.0 / 4.0), FIELD_BEARING_NORTH), FIELD_BEARING_NORTH)
                .splineToSplineHeading(new Pose2d(TILE_SIZE * (2.5 / 4.0), START_POS_RED_RIGHT.getY() + TILE_SIZE * 2, FIELD_BEARING_EAST), FIELD_BEARING_EAST + PI/3)
                .splineToConstantHeading(new Vector2d(TILE_SIZE, -TILE_SIZE / 2.0 - 1.5), FIELD_BEARING_EAST)
                .splineToConstantHeading(new Vector2d(TILE_SIZE * (3.0 / 2.0), -TILE_SIZE / 2.0), FIELD_BEARING_EAST)
                .splineToSplineHeading(coneStackPos, FIELD_BEARING_EAST)
                .forward(10)
                .build();

        trajectories[6] = getTrajBuilder(coneStackPos) // High junction inline with cone stack
                .back(TILE_SIZE / 16)
                .splineToSplineHeading(junc2Pos, FIELD_BEARING_NORTH)
                .build();

        trajectories[7] = getTrajBuilder(junc2Pos) // High junction inline with cone stack return
                .back(TILE_SIZE / 16)
                .splineToSplineHeading(coneStackPos, 0)
                //todo: straight approach to let claw lower
                .build();


        detectUntilStart();
        if (isStopRequested()) { return; }

        trajectories[9] = mapParkingTrajectory(detector.getTagSeenOrDefault(2));

        // Precondition: preload cone is in claw's reach and it was closed during init
        robot.slide.setCurrentWinchTarget(robot.slide.p.SLIDE_POS_HIGH);

        runTrajectory(trajectories[0]);  // approach forward high junction

        dropCone(robot.slide.getNextConeStackHeight()); // Drop cone and prepare for next

        runTrajectory(trajectories[1]); // Go to cone stack

        pickupCone(robot.slide.p.SLIDE_POS_HIGH); // Pickup cone and prepare to drop

        runTrajectory(trajectories[6]);  // approach side high junction
        dropCone(robot.slide.getNextConeStackHeight()); // Drop cone and prepare for next
        runTrajectory(trajectories[7]); // Go to cone stack
        pickupCone(robot.slide.p.SLIDE_POS_HIGH); // Pickup cone and prepare to drop

        runTrajectory(trajectories[6]);  // approach side high junction
        dropCone(robot.slide.getNextConeStackHeight()); // Drop cone and prepare for next
        runTrajectory(trajectories[7]); // Go to cone stack
        pickupCone(robot.slide.p.SLIDE_POS_HIGH); // Pickup cone and prepare to drop

        runTrajectory(trajectories[6]);  // approach side high junction
        dropCone(robot.slide.getNextConeStackHeight()); // Drop cone and prepare for next

        runTrajectory(trajectories[9]); // Park

        /*runTrajectory(trajectories[2]); // Low junction adjacent to cone stack

        dropCone(robot.slide.getNextConeStackHeight());

        runTrajectory(trajectories[3]); // return to cone stack

        pickupCone(robot.slide.p.SLIDE_POS_MED); // Pickup cone and prepare to drop

        runTrajectory(trajectories[4]); // Medium junction inline with cone stack

        dropCone(robot.slide.getNextConeStackHeight());

        runTrajectory(trajectories[5]); // Medium junction inline with cone stack return

        pickupCone(robot.slide.p.SLIDE_POS_HIGH); // Pickup cone and prepare to drop

        runTrajectory(trajectories[6]); // High junction inline with cone stack

        dropCone(0);

        runTrajectory(trajectories[7]); // Medium junction inline with cone stack return to center*/

        //todo: 6 cones

        //todo: park
    }

    public static double DROP_TIME = 150; // ms
    public static double PICKUP_TIME = 300; // ms
    public void dropCone(int nextLevel) {
        if (!opModeIsActive()) { return; }

        robot.slide.winch.setTargetPosition(nextLevel);

        long current = System.currentTimeMillis();

        // Stay in place while the cone is placed on the stack, this is much safer than dropping it
        while (System.currentTimeMillis() < current + DROP_TIME) {
            if (!opModeIsActive()) { return; }

            robot.update();
        }

        robot.slide.claw.turnToAngle(robot.slide.p.CLAW_POS_OPEN);
    }

    public static double CLAW_MOVE_TIME = 500;
    public void pickupCone(int nextJunctionLevel) {
        if (!opModeIsActive()) { return; }

        robot.slide.claw.turnToAngle(robot.slide.p.CLAW_POS_CLOSED);

        long current = System.currentTimeMillis();

        // Stay in place while we are in danger of knocking over the cone stack
        while (System.currentTimeMillis() < current + CLAW_MOVE_TIME) {
            if (!opModeIsActive()) { return; }

            robot.update();
        }

        robot.slide.winch.setTargetPosition(nextJunctionLevel);

        current = System.currentTimeMillis();

        // Stay in place while we are in danger of knocking over the cone stack
        while (System.currentTimeMillis() < current + PICKUP_TIME) {
            if (!opModeIsActive()) { return; }

            robot.update();
        }
    }

    public static double CALIBRATE_TIME = 2000;

    // Run the winch until its at zero
    public void calibrateSlide() {
        if (isStopRequested() || ! (robot.slide.winch instanceof WinchMotor)) { return; }

        long currentTime = System.currentTimeMillis();

        while (System.currentTimeMillis() < currentTime + CALIBRATE_TIME) {
            if (isStopRequested()) { return; }

            robot.update();

            ((WinchMotor)robot.slide.winch).setRawPower(((WinchMotor)robot.slide.winch).getHoldingPower());
        }

        robot.slide.winch.resetEncoder();
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

    public void detectUntilStart() {
        while (!isStarted() && !isStopRequested()) {
            detector.detect();
            sleep(100);
            if (detector.done()) {
                mapParkingTrajectory(detector.getTagSeenOrDefault(2));
            }
        }

        waitForStart();
    }

    public Trajectory mapParkingTrajectory(int tag) {
        return getTrajBuilder(addClawOffsetVec(JUNCTIONS.get(2), Math.PI * (3.0 / 4.0)))
                .lineToLinearHeading(getSignalParkingSpot(tag))
                .build();
    }

    public Pose2d getSignalParkingSpot(int tag) {
        if (tag == 1) {
            return new Pose2d(TILE_SIZE / 2, -TILE_SIZE / 2, FIELD_BEARING_NORTH);
        } else if (tag == 2) {
            return new Pose2d(TILE_SIZE * (3.0/2.0), -TILE_SIZE / 2, FIELD_BEARING_NORTH);
        } else if (tag == 3) {
            return new Pose2d(TILE_SIZE * (5.0/2.0), -TILE_SIZE / 2, FIELD_BEARING_NORTH);
        }

        return new Pose2d(TILE_SIZE * (3.0/2.0), -TILE_SIZE / 2, FIELD_BEARING_NORTH);
    }

    Vector2d combine(Vector2d in, double x, double y) {
        return new Vector2d(in.getX() + x, in.getY() + y);
    }

    Pose2d combinePose(Vector2d in, double x, double y, double heading) {
        return new Pose2d(in.getX() + x, in.getY() + y, heading);
    }
}
