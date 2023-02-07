package com.vcinventerman.pathvisualizer;

import static com.vcinventerman.pathvisualizer.PathTools.addClawOffset;
import static com.vcinventerman.pathvisualizer.PathTools.addClawOffsetVec;
import static com.vcinventerman.pathvisualizer.PathTools.getTrajBuilder;
import static com.vcinventerman.pathvisualizer.PathTools.safeTrajTo;
import static com.vcinventerman.pathvisualizer.TeamConf.CONE_STACK_POS_RED_RIGHT;
import static com.vcinventerman.pathvisualizer.TeamConf.FIELD_BEARING_EAST;
import static com.vcinventerman.pathvisualizer.TeamConf.FIELD_BEARING_NORTH;
import static com.vcinventerman.pathvisualizer.TeamConf.FIELD_BEARING_SOUTH;
import static com.vcinventerman.pathvisualizer.TeamConf.FIELD_BEARING_WEST;
import static com.vcinventerman.pathvisualizer.TeamConf.JUNCTIONS;
import static com.vcinventerman.pathvisualizer.TeamConf.ROBOT_LENGTH;
import static com.vcinventerman.pathvisualizer.TeamConf.ROBOT_WIDTH;
import static com.vcinventerman.pathvisualizer.TeamConf.START_POS_BLUE_LEFT;
import static com.vcinventerman.pathvisualizer.TeamConf.START_POS_RED_RIGHT;
import static com.vcinventerman.pathvisualizer.TeamConf.TILE_SIZE;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.sun.tools.javac.util.List;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;

public class PathVisualizer {

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

        //RedRight
        myBot.followTrajectorySequence(myBot.getDrive().trajectorySequenceBuilder(addClawOffset(CONE_STACK_POS_RED_RIGHT))
                .addTrajectory(getTrajBuilder(addClawOffset(CONE_STACK_POS_RED_RIGHT)) // Low junction adjacent to cone stack
                        .lineToLinearHeading(addClawOffsetVec(JUNCTIONS.get(11), Math.PI * (3.0 / 2)))


                        //.back(TILE_SIZE / 16)
                        //.splineToSplineHeading(addClawOffsetVec(JUNCTIONS.get(11), Math.PI * (3.0 / 2)), Math.PI * (3.0 / 2))


                        //.lineToSplineHeading(new Pose2d(TILE_SIZE * (5.0 / 2), -TILE_SIZE / 2.0, 0))

                        //.splineToSplineHeading(new Pose2d(TILE_SIZE * (5.0 / 2), -TILE_SIZE / 2.0, Math.PI * 1.75), 0)
                        //.splineToSplineHeading(new Pose2d(TILE_SIZE * (5.0 / 2), -TILE_SIZE / 2.0, 0), 0)
                        //.splineToSplineHeading(new Pose2d(TILE_SIZE * (5.0 / 2), -TILE_SIZE / 2.0, 0), PI/4)
                        //.splineToSplineHeading(addClawOffsetVec(JUNCTIONS.get(11), Math.PI * (5.1 / 4.0)), -PI/4)
                        //.splineToSplineHeading(addClawOffsetVec(JUNCTIONS.get(11), Math.PI * (5.0 / 4.0)), Math.PI * (5.0 / 4.0))
                        .build())


                /*.addTrajectory(getTrajBuilder(START_POS_RED_RIGHT) // Forward junction
                        .splineToConstantHeading(new Vector2d(TILE_SIZE * (2.5 / 4.0), START_POS_RED_RIGHT.getY() + 3), FIELD_BEARING_NORTH)

                        //.splineToSplineHeading(new Pose2d(TILE_SIZE * (1.0 / 4.0), START_POS_RED_RIGHT.getY() + TILE_SIZE, FIELD_BEARING_NORTH + PI/4), FIELD_BEARING_NORTH + PI/4)
                        .splineTo(addClawOffsetVec(JUNCTIONS.get(1), FIELD_BEARING_NORTH + PI / 4).vec(), FIELD_BEARING_NORTH + PI / 4)
                        .build())*/

                /*.addTrajectory(getTrajBuilder(START_POS_RED_RIGHT) // Forward junction
                        .lineTo(new Vector2d(START_POS_RED_RIGHT.getX() - TILE_SIZE / 2, START_POS_RED_RIGHT.getY()))
                        .splineToLinearHeading(new Pose2d(START_POS_RED_RIGHT.getX() - TILE_SIZE, START_POS_RED_RIGHT.getY() + TILE_SIZE, FIELD_BEARING_NORTH), FIELD_BEARING_NORTH)
                        .splineToSplineHeading(addClawOffsetVec(JUNCTIONS.get(1), FIELD_BEARING_NORTH + Math.PI / 4), FIELD_BEARING_NORTH + Math.PI / 4)
                        .build())*/

                /*
                .addTrajectory(getTrajBuilder(addClawOffsetVec(JUNCTIONS.get(1), FIELD_BEARING_NORTH + Math.PI / 4)) // Cone stack
                        .back(TILE_SIZE / 16)
                        .splineToConstantHeading(new Vector2d(START_POS_RED_RIGHT.getX() - TILE_SIZE, START_POS_RED_RIGHT.getY() + TILE_SIZE), FIELD_BEARING_NORTH)
                        .splineToSplineHeading(new Pose2d(START_POS_RED_RIGHT.getX() - TILE_SIZE, START_POS_RED_RIGHT.getY() + TILE_SIZE * (3.0 / 4.0), FIELD_BEARING_NORTH), FIELD_BEARING_NORTH)
                        .splineToSplineHeading(new Pose2d(START_POS_RED_RIGHT.getX() - TILE_SIZE, START_POS_RED_RIGHT.getY() + TILE_SIZE * 2, FIELD_BEARING_NORTH), FIELD_BEARING_NORTH)
                        .splineToConstantHeading(new Vector2d(TILE_SIZE, -TILE_SIZE / 2.0), FIELD_BEARING_EAST)
                        .splineToConstantHeading(new Vector2d(TILE_SIZE * (3.0 / 2.0), -TILE_SIZE / 2.0), FIELD_BEARING_EAST)
                        .splineToSplineHeading(addClawOffset(CONE_STACK_POS_RED_RIGHT), FIELD_BEARING_EAST)
                        .build())


                /*.addTrajectory(getTrajBuilder(addClawOffset(CONE_STACK_POS_RED_RIGHT)) // Low junction adjacent to cone stack
                        .back(TILE_SIZE / 64)
                        .splineToSplineHeading(addClawOffsetVec(JUNCTIONS.get(11), Math.PI * (5.0 / 4.0)), Math.PI * (5.0 / 4.0))
                        .build())

                .addTrajectory(getTrajBuilder(addClawOffsetVec(JUNCTIONS.get(11), Math.PI * (5.0 / 4.0))) // Low junction adjacent to cone stack return to cone stack
                        .back(TILE_SIZE / 16)
                        .splineToSplineHeading(addClawOffset(CONE_STACK_POS_RED_RIGHT), 0)
                        //todo: straight approach to let claw lower
                        .build())


                .addTrajectory(getTrajBuilder(addClawOffset(CONE_STACK_POS_RED_RIGHT)) // Medium junction inline with cone stack
                        .back(TILE_SIZE / 16)
                        .splineToSplineHeading(addClawOffsetVec(JUNCTIONS.get(5), Math.PI * (5.0 / 4.0)), Math.PI * (5.0 / 4.0))
                        .build())

                .addTrajectory(getTrajBuilder(addClawOffsetVec(JUNCTIONS.get(5), Math.PI * (5.0 / 4.0))) // Medium junction inline with cone stack return
                        .back(TILE_SIZE / 16)
                        .splineToSplineHeading(addClawOffset(CONE_STACK_POS_RED_RIGHT), 0)
                        //todo: straight approach to let claw lower
                        .build())


                .addTrajectory(getTrajBuilder(addClawOffset(CONE_STACK_POS_RED_RIGHT)) // High junction inline with cone stack
                        .back(TILE_SIZE / 16)
                        .splineToSplineHeading(addClawOffsetVec(JUNCTIONS.get(2), Math.PI * (3.0 / 4.0)), Math.PI * (3.0 / 4.0))
                        .build())

                .addTrajectory(getTrajBuilder(addClawOffsetVec(JUNCTIONS.get(2), Math.PI * (3.0 / 4.0))) // Medium junction inline with cone stack return
                        .back(TILE_SIZE / 16)
                        .splineToSplineHeading(addClawOffset(CONE_STACK_POS_RED_RIGHT), 0)
                        //todo: straight approach to let claw lower
                        .build())*/

                //.splineToLinearHeading(addClawOffset(CONE_STACK_POS_RED_RIGHT), FIELD_BEARING_EAST) // Stack




                //.addTrajectory(safeTrajTo(addClawOffset(CONE_STACK_POS_RED_RIGHT), new Pose2d(TILE_SIZE / 2.0, -TILE_SIZE / 2.0, 0)))
                .build());


    }






    static String variation = "RedLeft";
    static RoadRunnerBotEntity myBot;

}