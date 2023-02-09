package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.TeamConf.CLAW_TUNE_X;
import static org.firstinspires.ftc.teamcode.TeamConf.CLAW_TUNE_Y;
import static org.firstinspires.ftc.teamcode.TeamConf.FIELD_BEARING_NORTH;
import static org.firstinspires.ftc.teamcode.TeamConf.FIELD_BEARING_SOUTH;
import static org.firstinspires.ftc.teamcode.TeamConf.JUNCTIONS;
import static org.firstinspires.ftc.teamcode.TeamConf.ROBOTA_CLAW_OFFSET;
import static org.firstinspires.ftc.teamcode.TeamConf.ROBOT_CLAW_OFFSET;
import static org.firstinspires.ftc.teamcode.TeamConf.ROBOT_DRIVE_INST;
import static org.firstinspires.ftc.teamcode.TeamConf.TILE_SIZE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

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
                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                new MecanumVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH)
        )), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL));
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

    public static Pose2d addClawOffsetVec(Vector2d pos, double heading) {
        return addClawOffset(new Pose2d(pos, heading));
    }

    public static Pose2d addClawOffset(final Pose2d pos) {
        //Vector2D vec = new Vector2D(pos.getX(), pos.getY());

        Vector2d offset = new Vector2d(ROBOTA_CLAW_OFFSET.getY() + CLAW_TUNE_Y, ROBOTA_CLAW_OFFSET.getX() + CLAW_TUNE_X);
        double mag = Math.sqrt(offset.getX() * offset.getX() + offset.getY() * offset.getY());
        double ang = offset.angle();

        Vector2d mult = offset.rotated(pos.component3());

        //Vector2d mult = pos.vec().div(pos.vec().norm());
        //mult = new Vector2d(mult.getX() * offset.getX(), mult.getY() * offset.getY());

        /*offset = offset.rotated(pos.getHeading());

        return pos.minus(new Pose2d(offset.getX(), offset.getY(), 0));*/
        return pos.plus(new Pose2d(mult.getY(), mult.getX(), 0));

        //Vector2d rotated = offset.rotated(pos.getHeading());
        //Vector2d rotated = pos.vec().rotated(-pos.getHeading());
        //rotated = rotated.plus(offset);
        //rotated = rotated.rotated(pos.getHeading());
        //return new Pose2d(rotated.getX(), rotated.getY(), pos.getHeading());
        /*Vector2d offset = new Vector2d(ROBOTA_CLAW_OFFSET.getX() + CLAW_TUNE_X, ROBOTA_CLAW_OFFSET.getY() + CLAW_TUNE_Y);

        //Vector2d rotated = offset.rotated(pos.getHeading());
        Vector2d rotated = pos.vec().rotated(-pos.getHeading());

        Pose2d minus = pos.plus(new Pose2d(rotated, 0));
        return new Pose2d(rotated.getX() + offset.getX(), rotated.getY() + offset.getY(), pos.getHeading());*/
    }

    public static List<Pose2d> addClawOffsetList(List<Pose2d> poses) {
        return poses.stream().map(PathTools::addClawOffset).collect(Collectors.toList());
    }
}
