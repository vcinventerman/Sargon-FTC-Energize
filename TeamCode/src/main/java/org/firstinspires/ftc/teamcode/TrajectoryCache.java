package org.firstinspires.ftc.teamcode;

import android.content.res.Resources;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import java.util.Map;

public class TrajectoryCache {

    static class TrajNotFoundException extends Exception {}

    static private Map<String, Trajectory> cache;

    static private String createKey(Pose2d begin, Pose2d end) {
        return begin.toString() + ";" + end.toString();
    }

    static public Trajectory get(Pose2d begin, Pose2d end) throws TrajNotFoundException {
        Trajectory traj = cache.get(createKey(begin, end));

        if (traj == null) { throw new TrajNotFoundException(); }
        else { return traj; }
    }

    static public boolean set(Trajectory traj) {
        String key = createKey(traj.start(), traj.end());

        if (cache.containsKey(key)) { return false; }
        else {
            cache.put(key, traj);
            return true;
        }
    }
}
