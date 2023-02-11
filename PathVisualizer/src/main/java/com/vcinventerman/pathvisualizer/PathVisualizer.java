package com.vcinventerman.pathvisualizer;

import static com.vcinventerman.pathvisualizer.PathTools.addClawOffset;
import static com.vcinventerman.pathvisualizer.PathTools.addClawOffsetVec;
import static com.vcinventerman.pathvisualizer.PathTools.getTrajBuilder;
import static com.vcinventerman.pathvisualizer.PathTools.safeTrajTo;
import static com.vcinventerman.pathvisualizer.TeamConf.CONE_STACK_POS_RED_LEFT;
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
import com.acmerobotics.roadrunner.path.ParametricCurve;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.path.PathSegment;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import com.sun.tools.javac.util.List;

import java.util.ArrayList;
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

        PathBuilder pathBuilder = new PathBuilder(addClawOffsetVec(JUNCTIONS.get(1), FIELD_BEARING_NORTH + Math.PI / 4));
        //PathBuilder e = new PathBuilder()
        //ParametricCurve e = new ParametricCurve
        //PathSegment e = new PathSegment()
        Path path = new Path(new ArrayList<>());


        Pose2d startPose = START_POS_RED_RIGHT;
        double endBearing = FIELD_BEARING_NORTH;
        Pose2d endPose = new Pose2d(TILE_SIZE, 0,  0);

        Pose2d adjustedEnd = PathTools.addClawOffset(endPose);


        TrajectorySequenceBuilder seqBuilder = myBot.getDrive().trajectorySequenceBuilder(addClawOffset(CONE_STACK_POS_RED_RIGHT));

        for (int i = 0; i < 9; i++) {
            seqBuilder.addTrajectory(getTrajBuilder(
                    PathTools.addClawOffset(new Pose2d(endPose.getX(), endPose.getY(), PI * (i - 1) / 4.0))
            ).lineToLinearHeading(PathTools.addClawOffset(new Pose2d(endPose.getX(), endPose.getY(), PI * (i) / 4.0))).build());

            seqBuilder.addTrajectory(getTrajBuilder(
                    PathTools.addClawOffsetAlt(new Pose2d(endPose.getX(), endPose.getY(), PI * (i - 1) / 4.0))
            ).lineToLinearHeading(PathTools.addClawOffsetAlt(new Pose2d(endPose.getX(), endPose.getY(), PI * (i) / 4.0))).build());
        }

        Pose2d junc1Pos = combinePose(JUNCTIONS.get(1), 0, 0, FIELD_BEARING_NORTH);
        Pose2d coneStackPos = combinePose(CONE_STACK_POS_RED_RIGHT.vec(), 0, 0, CONE_STACK_POS_RED_RIGHT.getHeading());
        Pose2d junc2Pos = combinePose(JUNCTIONS.get(2), 0, 0, FIELD_BEARING_NORTH);


        Trajectory[] trajectories = new Trajectory[50];

        trajectories[0] = getTrajBuilder(START_POS_RED_RIGHT).lineTo(new Vector2d(TILE_SIZE * (2.2 / 4.0), START_POS_RED_RIGHT.getY() + 6)).build();
        trajectories[1] = getTrajBuilder(trajectories[0].end()).splineToLinearHeading(junc1Pos, FIELD_BEARING_NORTH + PI / 4).build();

        // Forward junction to cone stack
        trajectories[2] = getTrajBuilder(trajectories[1].end()).lineTo(new Vector2d(TILE_SIZE * (1.25 / 4.0), junc1Pos.getY() -10)).build();
        trajectories[3] = getTrajBuilder(trajectories[2].end()).lineToLinearHeading(new Pose2d(TILE_SIZE * (3.0 / 4.0), -TILE_SIZE/2.0, FIELD_BEARING_EAST)).build();
        trajectories[4] = getTrajBuilder(trajectories[3].end()).lineToLinearHeading(coneStackPos).build();

        // Cone stack to high junction
        trajectories[5] = getTrajBuilder(trajectories[4].end()).lineTo(new Vector2d(TILE_SIZE, -TILE_SIZE/2.0)).build();
        trajectories[6] = getTrajBuilder(trajectories[5].end()).lineToLinearHeading(new Pose2d(trajectories[5].end().getX(), trajectories[5].end().getY() + 1, FIELD_BEARING_NORTH)).build();
        trajectories[7] = getTrajBuilder(trajectories[6].end()).lineToLinearHeading(junc2Pos).build();

        // High junction back to cone stack
        trajectories[8] = getTrajBuilder(trajectories[7].end()).lineTo(new Vector2d(TILE_SIZE, -TILE_SIZE/2.0)).build();
        trajectories[9] = getTrajBuilder(trajectories[8].end()).lineToLinearHeading(new Pose2d(trajectories[5].end().getX() + 1, trajectories[5].end().getY(), FIELD_BEARING_EAST)).build();
        trajectories[10] = getTrajBuilder(trajectories[9].end()).lineToLinearHeading(coneStackPos).build();

        // High junction to park
        trajectories[11] = getTrajBuilder(trajectories[7].end()).lineTo(new Vector2d(TILE_SIZE, -TILE_SIZE/2.0)).build();
        trajectories[12] = getTrajBuilder(trajectories[11].end()).lineTo(new Vector2d(TILE_SIZE / 2.0, -TILE_SIZE/2.0)).build();
        trajectories[13] = getTrajBuilder(trajectories[11].end()).lineTo(new Vector2d(TILE_SIZE * 3.0 / 2.0, -TILE_SIZE/2.0)).build();
        trajectories[14] = getTrajBuilder(trajectories[11].end()).lineTo(new Vector2d(TILE_SIZE * 5.0 / 2.0, -TILE_SIZE/2.0)).build();






        //myBot.followTrajectorySequence(seqBuilder.build());

        myBot.followTrajectorySequence(myBot.getDrive().trajectorySequenceBuilder(addClawOffset(CONE_STACK_POS_RED_RIGHT))

                .addTrajectory(trajectories[11])
                .addTrajectory(trajectories[12])
                .addTrajectory(trajectories[13])
                .addTrajectory(trajectories[14])

                //.addTrajectory(trajectories[3])



                .build());
    }


    static Vector2d combine(Vector2d in, double x, double y) {
        return new Vector2d(in.getX() + x, in.getY() + y);
    }

    static Pose2d combinePose(Vector2d in, double x, double y, double heading) {
        return new Pose2d(in.getX() + x, in.getY() + y, heading);
    }






    static String variation = "RedLeft";
    static RoadRunnerBotEntity myBot;

}




//RedRight
        //myBot.followTrajectorySequence(myBot.getDrive().trajectorySequenceBuilder(addClawOffset(CONE_STACK_POS_RED_RIGHT))
         //       .addTrajectory(getTrajBuilder(startPose).lineToLinearHeading(adjustedEnd).build())


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
