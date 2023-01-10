package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TeamConf.ROBOT_DRIVE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

public class BulkTrajectoryBuilder {
    List<Pose2d> heads;
    Cacher cacher;


    public BulkTrajectoryBuilder(List<Pose2d> origins, Cacher cacher) {
        this.heads = origins;
        this.cacher = cacher;
    }

    List<Trajectory> apply(Function<TrajectoryBuilder, TrajectoryBuilder> func) {
        List<Trajectory> trajectories = new ArrayList<>(heads.size());

        for (int i = 0; i < heads.size(); i++) {
            TrajectoryBuilder builder = new TrajectoryBuilder(heads.get(i), ROBOT_DRIVE.VEL_CONSTRAINT, ROBOT_DRIVE.ACCEL_CONSTRAINT);
            trajectories.set(i, func.apply(builder).build());
            //todo:test
            // Advance head by pose's movement
            heads.set(i, trajectories.get(i).getPath().end());
        }

        cacher.cache(trajectories);
        return trajectories;
    }

    List<Trajectory> splineToSplineHeading(List<Pose2d> end) {
        List<Trajectory> trajectories = new ArrayList<>(heads.size());

        for (int i = 0; i < heads.size(); i++) {
            TrajectoryBuilder builder = new TrajectoryBuilder(heads.get(i), ROBOT_DRIVE.VEL_CONSTRAINT, ROBOT_DRIVE.ACCEL_CONSTRAINT);
            trajectories.set(i, builder.splineToSplineHeading(end.get(i), 0).build());
            heads.set(i, trajectories.get(i).getPath().end());
        }

        cacher.cache(trajectories);
        return trajectories;
    }

    public interface Cacher {
        abstract public void cache(List<Trajectory> trajectories);
    }
}
