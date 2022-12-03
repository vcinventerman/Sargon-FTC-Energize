package com.vcinventerman.pathvisualizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class PathVisualizer {


    // Field is 11ft 9in
    public static double FIELD_WIDTH = 141;
    // Tiles are 2ft
    public static double TILE_SIZE = 24;

    // Field from the point of view of the red alliance: +y is towards blue, -y is towards red, +x is towards red right
    public static double FIELD_BEARING_NORTH = Math.PI / 2;
    public static double FIELD_BEARING_SOUTH = Math.PI * (3.0 / 2.0);
    public static double FIELD_BEARING_EAST = 0.0;
    public static double FIELD_BEARING_WEST = Math.PI;

    // Senior team robot
    public static double ROBOTA_LENGTH = 18;
    public static double ROBOTA_WIDTH = 17;

    // Choose current robot here
    public static double ROBOT_LENGTH = ROBOTA_LENGTH;
    public static double ROBOT_WIDTH = ROBOTA_WIDTH;

    public static Pose2d START_POS_RED_LEFT = new Pose2d(-35, -(FIELD_WIDTH / 2) + (ROBOT_LENGTH / 2), FIELD_BEARING_NORTH);
    public static Pose2d START_POS_RED_RIGHT = new Pose2d(35, -(FIELD_WIDTH / 2) + (ROBOT_LENGTH / 2), FIELD_BEARING_NORTH);
    public static Pose2d START_POS_BLUE_LEFT = new Pose2d(35, (FIELD_WIDTH / 2) - (ROBOT_LENGTH / 2), FIELD_BEARING_SOUTH);
    public static Pose2d START_POS_BLUE_RIGHT = new Pose2d(-35, (FIELD_WIDTH / 2) - (ROBOT_LENGTH / 2), FIELD_BEARING_SOUTH);







    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");

        MeepMeep meepMeep = new MeepMeep(800);
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL);

        myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setStartPose(START_POS_BLUE_LEFT)
                .setDimensions(ROBOT_WIDTH, ROBOT_LENGTH).build();
                        //.followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(START_POS_RED_LEFT).forward(20).build());
                /*.followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .build()
                );*/

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();

        myBot.followTrajectorySequence(myBot.getDrive().trajectorySequenceBuilder(START_POS_RED_LEFT)
                .forward(TILE_SIZE * (5.0 / 2.0)) // Get the signal cone out of the way
                .back(TILE_SIZE / 2)
                .lineToLinearHeading(centerConeOverJunction(new Vector2d(-TILE_SIZE, 0), new Vector2d(START_POS_RED_LEFT.getX(), START_POS_RED_LEFT.getY() - TILE_SIZE * 2)))
                .strafeTo(new Vector2d(START_POS_RED_LEFT.getX(), START_POS_RED_LEFT.getY() + TILE_SIZE * 2))
                .lineToLinearHeading(getSignalSpot(3))
                .build());


    }






    static String variation = "RedLeft";
    static RoadRunnerBotEntity myBot;

    static public Pose2d getSignalSpot(int tag) {
        tag = tag; // Will be 1, 2, or 3

        if (true) {
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
                    return new Pose2d(TILE_SIZE / 2, TILE_SIZE / 2, FIELD_BEARING_SOUTH);
                } else if (tag == 2) {
                    return new Pose2d(TILE_SIZE * (3.0/2.0), TILE_SIZE / 2, FIELD_BEARING_SOUTH);
                } else if (tag == 3) {
                    return new Pose2d(TILE_SIZE * (5.0/2.0), TILE_SIZE / 2, FIELD_BEARING_SOUTH);
                }
            }
            // Tile A5 start
            else {
                if (tag == 1) {
                    return new Pose2d(-TILE_SIZE * (5.0/2.0), -TILE_SIZE / 2, FIELD_BEARING_SOUTH);
                } else if (tag == 2) {
                    return new Pose2d(-TILE_SIZE * (3.0/2.0), -TILE_SIZE / 2, FIELD_BEARING_SOUTH);
                } else if (tag == 3) {
                    return new Pose2d(-TILE_SIZE / 2, -TILE_SIZE / 2, FIELD_BEARING_SOUTH);
                }
            }
        }

        // Do nothing on error
        return null;
    }

    // Returns the angle the robot needs to turn to before moving forward and the point it needs to drive to
    //todo: claw probably isn't even in the center
    static Pose2d centerConeOverJunction(Vector2d junctionPos, Vector2d robotPos) {
        //Vector2d robotPos = myBot.getPose().vec(); //robot.drive.getPoseEstimate().vec();
        double angle = Math.atan((junctionPos.getY() - robotPos.getY()) / (junctionPos.getX() - robotPos.getX()));
        double hyp = Math.sqrt(Math.pow(junctionPos.getX() - robotPos.getX(), 2) + Math.pow(junctionPos.getY() - robotPos.getY(), 2))
                - 10.5; // distance from center of robot to claw which is on a horizontal center axis
        double offsetX = hyp * Math.cos(angle);
        double offsetY = hyp * Math.sin(angle);

        return new Pose2d(robotPos.plus(new Vector2d(offsetX, offsetY)), angle);
    }
}