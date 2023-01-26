package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TeamConf.START_POSE_DEFAULT;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.MecanumDriveCancelable.getAccelerationConstraint;
import static org.firstinspires.ftc.teamcode.drive.MecanumDriveCancelable.getVelocityConstraint;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveCancelable;

public class RobotA extends Robot {
    public MecanumDriveCancelable drive;

    public MecanumDrive manualDrive;

    public RobotA(HardwareMap hardwareMap) {
        PhotonCore.enable();
        //PhotonCore.experimental.setSinglethreadedOptimized(false);

        drive = new MecanumDriveCancelable(hardwareMap);
        drive.setPoseEstimate(START_POSE_DEFAULT);

        slide = new LinearSlideA(hardwareMap);

        MotorEx rightRear = new MotorEx(hardwareMap,  "driveBackRight");
        MotorEx rightFront = new MotorEx(hardwareMap,  "driveFrontRight");
        MotorEx leftFront = new MotorEx(hardwareMap,  "driveFrontLeft");
        MotorEx leftRear = new MotorEx(hardwareMap,  "driveBackLeft");

        //rightFront.setInverted(true);
        //leftFront.setInverted(true);

        manualDrive = new MecanumDrive(false, leftFront, rightFront, leftRear, rightRear);
    }

    public RobotA(HardwareMap hardwareMap, Pose2d startPose) {
        this(hardwareMap);
        drive.setPoseEstimate(startPose);
    }

    public RobotA(HardwareMap hardwareMap, boolean fast) {
        this(hardwareMap);
        MecanumDriveCancelable.VEL_CONSTRAINT = getVelocityConstraint(10, 2, TRACK_WIDTH);
        MecanumDriveCancelable.ACCEL_CONSTRAINT = getAccelerationConstraint(20);
    }

    public void update()
    {
        slide.update();
        drive.update();
    }

    @Override
    public void setPose(Pose2d newPose) {
        drive.setPoseEstimate(newPose);
    }

    @Override
    public void updatePose() {
        drive.updatePoseEstimate();
    }

    @Override
    public Pose2d getPoseEstimate() {
        return drive.getPoseEstimate();
    }

    @Override
    public void followTrajectory(Trajectory traj) {
        drive.followTrajectoryAsync(traj);
    }

    @Override
    public boolean isBusy() {
        return drive.isBusy();
    }


    public Drive getDrive() {
        return drive;
    }

    public void driveFieldCentric(double strafeSpeed, double forwardSpeed,
                                  double turnSpeed, double gyroAngle) {
        manualDrive.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, gyroAngle);
    }

    @Override
    public double getHeading() {
        return drive.getExternalHeading();
    }
}
