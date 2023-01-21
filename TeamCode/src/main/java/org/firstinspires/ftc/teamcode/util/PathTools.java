package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.TeamConf.FIELD_BEARING_NORTH;
import static org.firstinspires.ftc.teamcode.TeamConf.FIELD_BEARING_SOUTH;
import static org.firstinspires.ftc.teamcode.TeamConf.JUNCTIONS;
import static org.firstinspires.ftc.teamcode.TeamConf.ROBOT_DRIVE_INST;
import static org.firstinspires.ftc.teamcode.TeamConf.TILE_SIZE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;
import java.util.List;

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

    // Go to the nearest center of a cell
    // Move to the row, then the col of the target
    //
    public static Trajectory safeTrajTo(Pose2d start, Pose2d end) {
        TrajectoryBuilder builder = new TrajectoryBuilder(start, ROBOT_DRIVE_INST.VEL_CONSTRAINT, ROBOT_DRIVE_INST.ACCEL_CONSTRAINT);

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
        builder.splineToSplineHeading(new Pose2d(safeCell.getX(), safeCell.getY(), end.getHeading()), 0);

        // Drive to x position of end safe cell
        builder.splineToSplineHeading(new Pose2d(safeEndCell.getX(), end.getY(), end.getHeading()), 0);

        // Drive to safe end cell
        builder.splineToSplineHeading(new Pose2d(safeEndCell.getX(), safeEndCell.getY(), end.getHeading()), 0);

        // Drive to end
        builder.splineToSplineHeading(end, 0);


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
        TrajectoryBuilder builder = new TrajectoryBuilder(robotPos, SampleMecanumDrive.VEL_CONSTRAINT, SampleMecanumDrive.ACCEL_CONSTRAINT);
        builder.splineToSplineHeading(new Pose2d(getNearestJunction(robotPos), robotPos.getHeading()), 0);

        // This part takes a while
        return builder.build();
    }
}
