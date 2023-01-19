package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.TeamConf.ROBOT_DRIVE_INST;
import static org.firstinspires.ftc.teamcode.TeamConf.TILE_SIZE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

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
}
