package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TeamConf.ROBOT_DRIVE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import java.util.Arrays;
import java.util.List;
import java.util.ArrayList;
import java.util.function.BiFunction;
import java.util.function.Function;

public class BulkTrajectoryBuilder {
    List<Pose2d> heads;
    Cacher cacher;


    public BulkTrajectoryBuilder(List<Pose2d> origins, Cacher cacher) {
        this.heads = origins;
        this.cacher = cacher;
    }

    List<Trajectory> apply(BiFunction<TrajectoryBuilder, Pose2d, TrajectoryBuilder> func) {
        List<Trajectory> trajectories = new ArrayList<Trajectory>(heads.size());

        for (int i = 0; i < heads.size(); i++) {
            TrajectoryBuilder builder = new TrajectoryBuilder(heads.get(i), ROBOT_DRIVE.VEL_CONSTRAINT, ROBOT_DRIVE.ACCEL_CONSTRAINT);
            trajectories.add(func.apply(builder, heads.get(i)).build());
            //todo:test
            // Advance head by pose's movement
            heads.set(i, trajectories.get(i).getPath().end());
            cacher.cache(trajectories.get(i));
        }

        return trajectories;
    }

    List<Trajectory> splineToSplineHeading(List<Pose2d> end) {
        List<Trajectory> trajectories = new ArrayList<>(heads.size());

        for (int i = 0; i < heads.size(); i++) {
            TrajectoryBuilder builder = new TrajectoryBuilder(heads.get(i), ROBOT_DRIVE.VEL_CONSTRAINT, ROBOT_DRIVE.ACCEL_CONSTRAINT);
            trajectories.add(builder.splineToSplineHeading(end.get(i), 0).build());
            heads.set(i, trajectories.get(i).getPath().end());
            cacher.cache(trajectories.get(i));
        }

        return trajectories;
    }

    List<Trajectory> splineToSplineHeadingVec(List<Vector2d> end) {
        List<Trajectory> trajectories = new ArrayList<>(heads.size());

        for (int i = 0; i < heads.size(); i++) {
            TrajectoryBuilder builder = new TrajectoryBuilder(heads.get(i), ROBOT_DRIVE.VEL_CONSTRAINT, ROBOT_DRIVE.ACCEL_CONSTRAINT);
            trajectories.add(builder.splineToSplineHeading(new Pose2d(end.get(i), heads.get(i).getHeading()), 0).build());
            heads.set(i, trajectories.get(i).getPath().end());
            cacher.cache(trajectories.get(i));;
        }

        return trajectories;
    }

    Trajectory[][] multiApply
            (MultiApplier func, int innerVariations)
    {
        Trajectory[][] trajectories = new Trajectory[heads.size()][innerVariations];

        for (int i = 0; i < heads.size(); i++) {
            for (int j = 0; j < innerVariations; j++) {
                TrajectoryBuilder builder = new TrajectoryBuilder(heads.get(i), ROBOT_DRIVE.VEL_CONSTRAINT, ROBOT_DRIVE.ACCEL_CONSTRAINT);
                trajectories[i][j] = func.apply(builder, Arrays.asList("RedLeft", "RedRight", "BlueLeft", "BlueRight").get(i), j + 1).build();
                cacher.cache(trajectories[i][j]);
            }
        }

        return trajectories;
    }

    public interface Cacher {
        abstract public void cache(Trajectory trajectory);
    }

    public interface MultiApplier {
        abstract public TrajectoryBuilder apply(TrajectoryBuilder builder, String variation, Integer barcode);
    }
}
