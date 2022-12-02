package com.vcinventerman.pathvisualizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class PathVisualizer {

    // Field is 11ft 9in
    public static double FIELD_WIDTH = 141;

    // Field from the point of view of the red alliance: +y is towards blue, -y is towards red
    public static double FIELD_BEARING_NORTH = Math.PI / 2;
    public static double FIELD_BEARING_SOUTH = Math.PI * (3.0 / 2.0);
    public static double FIELD_BEARING_EAST = 0.0;
    public static double FIELD_BEARING_WEST = Math.PI;

    // Senior team robot
    public static double ROBOTA_LENGTH = 18;
    public static double ROBOTA_WIDTH = 12;

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

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setStartPose(START_POS_BLUE_LEFT)
                .setDimensions(ROBOT_WIDTH, ROBOT_LENGTH)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(START_POS_BLUE_LEFT)
                                .forward(25)
                                .turn(Math.toRadians(-90))
                                .forward(25)
                                .turn(Math.toRadians(45))
                                .forward(5)
                                .back(5)
                                .turn(Math.toRadians(-45))
                                .back(25)
                                .build()
                );
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
    }
}