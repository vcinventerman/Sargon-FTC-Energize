package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

public abstract class Robot {
    public LinearSlideA slide;

    abstract Drive getDrive();
    abstract public void driveFieldCentric(double strafeSpeed, double forwardSpeed, double turnSpeed, double gyroAngle);

    abstract public double getHeading();

    abstract public void update();

    abstract public void setPose(Pose2d newPose);
    abstract public void updatePose();
    abstract public Pose2d getPoseEstimate();

    abstract public void followTrajectory(Trajectory traj);
    abstract public boolean isBusy();
    public void waitToComplete() {
        while (isBusy()) { update(); }
    }
}
