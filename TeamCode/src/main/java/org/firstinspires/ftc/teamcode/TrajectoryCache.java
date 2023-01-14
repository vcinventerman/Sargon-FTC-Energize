package org.firstinspires.ftc.teamcode;

import android.content.res.Resources;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

// Store trajectories that have been calculated with the current robot not to hit any junctions
// Creating trajectories is very time consuming, so it should be done mostly in advance
public class TrajectoryCache {

    static class TrajNotFoundException extends Exception {}

    static private Map<String, Trajectory> cache = new HashMap<>();

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

    static public boolean set(List<Trajectory> trajectories) {
        boolean succeed = true;

        for (Trajectory i : trajectories) {
            succeed = set(i) && succeed;
        }

        return succeed;
    }
}
