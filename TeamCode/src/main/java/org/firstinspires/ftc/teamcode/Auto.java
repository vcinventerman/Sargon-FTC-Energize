package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TeamConf.FIELD_BEARING_NORTH;
import static org.firstinspires.ftc.teamcode.TeamConf.FIELD_BEARING_SOUTH;
import static org.firstinspires.ftc.teamcode.TeamConf.START_POS_RED_LEFT;
import static org.firstinspires.ftc.teamcode.TeamConf.TILE_SIZE;
import static org.firstinspires.ftc.teamcode.TeamConf.sleep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.karrmedia.ftchotpatch.Supervised;
import com.karrmedia.ftchotpatch.SupervisedOpMode;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Supervised(name="?Auto", group="!CompAuto", autonomous=true, variations={"RedLeft", "RedRight", "BlueLeft", "BlueRight"})
public class Auto extends SupervisedOpMode {
    RobotA robot;
    AprilTagDetector detector;


    // Code that runs when the INIT button is pressed (mandatory)
    public void init() {
        robot = new RobotA(hardwareMap);

        // Detect AprilTags
        detector = new AprilTagDetector(hardwareMap, List.of(21, 22, 23));
    }

    public void initLoop() {
        detector.detect();
        sleep(100);
    }

    public void start() {
        elapsedRuntime.reset();

        // Keep trying to detect for a while in case we didn't have any time after init
        while (elapsedRuntime.milliseconds() < 5000 && !detector.done()) {
            detector.detect();
            sleep(100);
        }

        int tag = detector.getTagSeenOrDefault(2);

        telemetry.addData("!", "Found tag " + Integer.toString(tag));
        telemetry.update();

        robot.slide.setClawState(robot.slide.CLAW_POS_CLOSED);
        robot.slide.setCurrentWinchTarget(robot.slide.SLIDE_POS_HIGH);

        TrajectorySequenceBuilder builder = null;

        // From the starting position to the junction to drop the preloaded cone at
        TrajectorySequence seq1 = null;

        // Move far enough away from the junction to lower the arm
        TrajectorySequence seq2 = null;

        // From the junction to the space indicated by the signal sleeve
        TrajectorySequence seq3 = null;


        if (variation.equals("RedLeftAuto")) {
             builder = robot.drive.trajectorySequenceBuilder(START_POS_RED_LEFT)
                     .forward(35) // Move forward and back to try and displace the signal sleeve
                     .back(10)
                     .turn(Math.toRadians(-90))
                     .forward(25)
                     .turn(Math.toRadians(45))
                     .forward(5);
        }
        else if (variation.equals("RedRightAuto")) {
            builder = robot.drive.trajectorySequenceBuilder(START_POS_RED_LEFT)
                    .forward(35) // Move forward and back to try and displace the signal sleeve
                    .back(10)
                    .turn(Math.toRadians(90))
                    .forward(25)
                    .turn(Math.toRadians(-45))
                    .forward(5);
        }
        else if (variation.equals("BlueLeftAuto")) {
            builder = robot.drive.trajectorySequenceBuilder(START_POS_RED_LEFT)
                    .forward(35) // Move forward and back to try and displace the signal sleeve
                    .back(10)
                    .turn(Math.toRadians(-90))
                    .forward(25)
                    .turn(Math.toRadians(45))
                    .forward(5);
        }
        else { // if (variation.equals("BlueRightAuto")) {
            builder = robot.drive.trajectorySequenceBuilder(START_POS_RED_LEFT)
                    .forward(35) // Move forward and back to try and displace the signal sleeve
                    .back(10)
                    .turn(Math.toRadians(90))
                    .forward(25)
                    .turn(Math.toRadians(-45))
                    .forward(5);
        }

        // Open the claw over the junction
        builder
                .addDisplacementMarker(() -> {
                    robot.slide.setClawState(robot.slide.CLAW_POS_OPEN);
                })
                .waitSeconds(3)
                .back(5);
        faceToCenter(builder);
        builder.splineToSplineHeading(getSignalSpot(tag), getSignalSpot(tag).getHeading());


        TrajectorySequence seq = builder.build();
        robot.drive.followTrajectorySequenceAsync(seq);

        while (robot.drive.isBusy()) {
            robot.update();
        }
    }

    // Code that runs repeatedly after the PLAY button is pressed (optional)
    public void loop() {


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
            return TeamConf.Alliance.RED;
        }
        else {
            return TeamConf.Alliance.NONE;
        }
    }

    public TrajectorySequenceBuilder faceToCenter(TrajectorySequenceBuilder seq) {
        if (getAlliance() == TeamConf.Alliance.BLUE) {
            seq.splineToLinearHeading(robot.drive.getPoseEstimate(), FIELD_BEARING_SOUTH);
        }
        else {
            seq.splineToLinearHeading(robot.drive.getPoseEstimate(), FIELD_BEARING_NORTH);
        }
        return seq;
    }

    public Pose2d getSignalSpot(int tag) {
        tag = tag - 20; // Will be 1, 2, or 3

        if (getAlliance() == TeamConf.Alliance.RED) {
            // Tile F5 start
            if (variation.contains("Right")) {
                if (tag == 1) {
                    return new Pose2d(TILE_SIZE / 2, -TILE_SIZE, FIELD_BEARING_NORTH);
                } else if (tag == 2) {
                    return new Pose2d(TILE_SIZE * (3.0/2.0), -TILE_SIZE, FIELD_BEARING_NORTH);
                } else if (tag == 3) {
                    return new Pose2d(TILE_SIZE * (5.0/2.0), -TILE_SIZE, FIELD_BEARING_NORTH);
                }
            }
            // Tile F2 start
            else {
                if (tag == 1) {
                    return new Pose2d(-TILE_SIZE * (5.0/2.0), -TILE_SIZE, FIELD_BEARING_NORTH);
                } else if (tag == 2) {
                    return new Pose2d(-TILE_SIZE * (3.0/2.0), -TILE_SIZE, FIELD_BEARING_NORTH);
                } else if (tag == 3) {
                    return new Pose2d(-TILE_SIZE / 2, -TILE_SIZE, FIELD_BEARING_NORTH);
                }
            }
        }
        else {
            // Tile A2 start
            if (variation.contains("Right")) {
                if (tag == 1) {
                    return new Pose2d(TILE_SIZE / 2, TILE_SIZE, FIELD_BEARING_SOUTH);
                } else if (tag == 2) {
                    return new Pose2d(TILE_SIZE * (3.0/2.0), TILE_SIZE, FIELD_BEARING_SOUTH);
                } else if (tag == 3) {
                    return new Pose2d(TILE_SIZE * (5.0/2.0), TILE_SIZE, FIELD_BEARING_SOUTH);
                }
            }
            // Tile A5 start
            else {
                if (tag == 1) {
                    return new Pose2d(-TILE_SIZE * (5.0/2.0), -TILE_SIZE, FIELD_BEARING_SOUTH);
                } else if (tag == 2) {
                    return new Pose2d(-TILE_SIZE * (3.0/2.0), -TILE_SIZE, FIELD_BEARING_SOUTH);
                } else if (tag == 3) {
                    return new Pose2d(-TILE_SIZE / 2, -TILE_SIZE, FIELD_BEARING_SOUTH);
                }
            }
        }

        // Do nothing on error
        return robot.drive.getPoseEstimate();
    }
}
