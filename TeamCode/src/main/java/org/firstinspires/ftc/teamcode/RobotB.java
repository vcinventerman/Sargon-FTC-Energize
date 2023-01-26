package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.MecanumDriveCancelable.getAccelerationConstraint;
import static org.firstinspires.ftc.teamcode.drive.MecanumDriveCancelable.getVelocityConstraint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

@Config
public class RobotB extends Robot {
    public org.firstinspires.ftc.teamcode.drive.HDrive drive;

    public HDrive manualDrive;

    public static String NAME_LEFT = "BackLeft";
    public static String NAME_RIGHT = "BackRight";
    public static String NAME_CENTER = "H drive";

    public static int CENTER_CPR = 288;
    public static int CENTER_RPM = 137;

    public static String NAME_LIFT = "Lift 1";
    public static String NAME_LIFT_AUX = "lift2";
    public static String NAME_CLAW = "Servo";



    public RobotB(HardwareMap hardwareMap) {
        PhotonCore.enable();
        //PhotonCore.experimental.setSinglethreadedOptimized(false);

        drive = new org.firstinspires.ftc.teamcode.drive.HDrive(hardwareMap);

        MotorEx leader = new MotorEx(hardwareMap, NAME_LIFT);
        leader.setInverted(false);
        MotorEx follower = new MotorEx(hardwareMap, NAME_LIFT_AUX);
        follower.setInverted(true);
        Motor liftMotor = new MotorGroup(leader, follower);

        liftMotor.encoder = liftMotor.new Encoder(hardwareMap.get(DcMotorEx.class, NAME_LIFT)::getCurrentPosition);
        slide = new LinearSlideA(hardwareMap, liftMotor, new SimpleServo(hardwareMap, NAME_CLAW, 0, 360, AngleUnit.DEGREES));

        MotorEx leftMotor = new MotorEx(hardwareMap, NAME_LEFT);
        leftMotor.setRunMode(MotorEx.RunMode.RawPower);
        //leftMotor.setInverted(true);

        MotorEx rightMotor = new MotorEx(hardwareMap, NAME_RIGHT);
        rightMotor.setRunMode(Motor.RunMode.RawPower);

        //DcMotor slideMotor = hardwareMap.get(DcMotor.class, NAME_CENTER);

        MotorEx slideMotor = new MotorEx(hardwareMap, NAME_CENTER, CENTER_CPR, CENTER_RPM);
        slideMotor.setInverted(true);
        slideMotor.motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setRunMode(MotorEx.RunMode.RawPower);

        manualDrive = new HDrive(
                leftMotor,
                rightMotor,
                slideMotor,
                Math.PI * (3.0 / 2.0), Math.PI / 2.0, 0);
    }

    public RobotB(HardwareMap hardwareMap, Pose2d startPose) {
        this(hardwareMap);
        drive.setPoseEstimate(startPose);
    }

    public RobotB(HardwareMap hardwareMap, boolean fast) {
        this(hardwareMap);
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
    Drive getDrive() {
        return drive;
    }

    @Override
    public void driveFieldCentric(double strafeSpeed, double forwardSpeed, double turnSpeed, double gyroAngle) {
        manualDrive.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, gyroAngle);
    }

    @Override
    public double getHeading() {
        return drive.getExternalHeading();
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
}
