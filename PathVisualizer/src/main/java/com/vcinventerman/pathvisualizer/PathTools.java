package com.vcinventerman.pathvisualizer;


import static com.vcinventerman.pathvisualizer.TeamConf.CLAW_TUNE_X;
import static com.vcinventerman.pathvisualizer.TeamConf.CLAW_TUNE_Y;
import static com.vcinventerman.pathvisualizer.TeamConf.FIELD_BEARING_NORTH;
import static com.vcinventerman.pathvisualizer.TeamConf.FIELD_BEARING_SOUTH;
import static com.vcinventerman.pathvisualizer.TeamConf.JUNCTIONS;
import static com.vcinventerman.pathvisualizer.TeamConf.ROBOTA_CLAW_OFFSET;
import static com.vcinventerman.pathvisualizer.TeamConf.ROBOT_CLAW_OFFSET;
import static com.vcinventerman.pathvisualizer.TeamConf.ROBOT_DRIVE_INST;
import static com.vcinventerman.pathvisualizer.TeamConf.TILE_SIZE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.noahbres.meepmeep.roadrunner.Constraints;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;


import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class PathTools {





    public static Vector2d nearestTo(Vector2d start, List<Vector2d> search) {
        Vector2d cell = search.get(0);
        double dist = 999;

        for (int i = 0; i < search.size(); i++) {
            double newDist = start.distTo(search.get(i));

            if (dist > newDist) {
                cell = search.get(i);
                dist = newDist;
            }
        }

        return cell;
    }

    public static TrajectoryBuilder getTrajBuilder(Pose2d start) {
        return new TrajectoryBuilder(start, new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(4.1059),
                new MecanumVelocityConstraint(40.956837, 11.67)
        )), new ProfileAccelerationConstraint(41.065033847087705));
    }

    // Go to the nearest center of a cell
    // Move to the row, then the col of the target
    //
    public static Trajectory safeTrajTo(Pose2d start, Pose2d end) {
        TrajectoryBuilder builder = new TrajectoryBuilder(start, 	new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(4.1059),
                new MecanumVelocityConstraint(40.956837, 11.67)
        )), new ProfileAccelerationConstraint(41.065033847087705));

        final double[] safe = new double[]{
                -TILE_SIZE * (5.0 / 2.0), -TILE_SIZE * (3.0 / 2.0), -TILE_SIZE * (1.0 / 2.0),
                TILE_SIZE * (1.0 / 2.0), TILE_SIZE * (3.0 / 2.0), TILE_SIZE * (5.0 / 2.0)};

        List<Vector2d> safeCells = new ArrayList<>();

        for (double i = -TILE_SIZE * (5.0 / 2.0); i < TILE_SIZE * 3; i += TILE_SIZE) {
            for (double j = -TILE_SIZE * (5.0 / 2.0); j < TILE_SIZE * 3; j += TILE_SIZE) {
                safeCells.add(new Vector2d(i, j));
            }
        }

        Vector2d safeCell = nearestTo(start.vec(), safeCells);
        Vector2d safeEndCell = nearestTo(end.vec(), safeCells);

        // Drive to a safe position that may be easily driven from
        builder.lineTo(safeCell);

        // Drive to x position of end safe cell
        builder.lineToLinearHeading(new Pose2d(safeEndCell.getX(), end.getY(), end.getHeading()));

        if (!safeEndCell.epsilonEquals(new Vector2d(safeEndCell.getX(), end.getY()))) {
            // Drive to safe end cell
            builder.lineTo(safeEndCell);
        }

        if (!end.vec().epsilonEquals(new Vector2d(safeEndCell.getX(), end.getY()))) {
            // Drive to end
            builder.lineToLinearHeading(end);
        }

        return builder.build();
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

    public static Vector2d getNearestJunction(Vector2d pos) {
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
    public static Vector2d getNearestJunction(Pose2d pos) { return getNearestJunction(pos.vec()); }

    public static Trajectory trajToNearestJunction(Pose2d robotPos) {
        //todo: pole avoidance, offset to claw
        TrajectoryBuilder builder = new TrajectoryBuilder(robotPos, new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(4.1059),
                new MecanumVelocityConstraint(40.956837, 11.67)
        )), new ProfileAccelerationConstraint(41.065033847087705));
        builder.splineToSplineHeading(new Pose2d(getNearestJunction(robotPos), robotPos.getHeading()), 0);

        // This part takes a while
        return builder.build();
    }

    public static Pose2d addClawOffsetVec(Vector2d pos, double heading) {
        return addClawOffset(new Pose2d(pos, heading));
    }

    public static Pose2d addClawOffset(Pose2d pos) {
        Vector2d offset = ROBOT_CLAW_OFFSET;

        return pos.plus(new Pose2d(offset.rotated(pos.getHeading()), 0));

        /*double newX = offset.getX() * Math.cos(pos.getHeading()) - offset.getY() * Math.sin(pos.getHeading());
        double newY = offset.getX() * Math.sin(pos.getHeading()) + offset.getY() * Math.cos(pos.getHeading());

        return new Pose2d(pos.getX() + newX, pos.getY() + newY, pos.getHeading());*/
    }

    public static Pose2d addClawOffsetAlt(Pose2d pos) {
        Vector2d offset = ROBOT_CLAW_OFFSET;

        double newX = offset.getX() * Math.cos(pos.getHeading()) - offset.getY() * Math.sin(pos.getHeading());
        double newY = offset.getX() * Math.sin(pos.getHeading()) + offset.getY() * Math.cos(pos.getHeading());

        return new Pose2d(pos.getX() + newX, pos.getY() + newY, pos.getHeading());
    }

    public static List<Pose2d> addClawOffsetList(List<Pose2d> poses) {
        return poses.stream().map(PathTools::addClawOffset).collect(Collectors.toList());
    }
}
