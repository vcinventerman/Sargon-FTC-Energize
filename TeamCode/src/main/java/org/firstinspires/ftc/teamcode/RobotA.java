package org.firstinspires.ftc.teamcode;

import static com.outoftheboxrobotics.photoncore.PhotonCore.CONTROL_HUB;
import static com.outoftheboxrobotics.photoncore.PhotonCore.EXPANSION_HUB;
import static org.firstinspires.ftc.teamcode.TeamConf.START_POSE_DEFAULT;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.MecanumDriveCancelable.getAccelerationConstraint;
import static org.firstinspires.ftc.teamcode.drive.MecanumDriveCancelable.getVelocityConstraint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveCancelable;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

@Config
public class RobotA extends Robot {
    public MecanumDriveCancelable drive;

    public MecanumDrive manualDrive;


    public static Double WINCH_TOLERANCE = 30.0;
    public static Double WINCH_TICKS_PER_INCH = (1390.0 / 24.0);
    public static Integer SLIDE_POS_BOTTOM = 0;
    public static Integer SLIDE_POS_GROUND = SLIDE_POS_BOTTOM + 100;
    public static Integer SLIDE_POS_LOW = SLIDE_POS_BOTTOM + 1100;
    public static Integer SLIDE_POS_MED = SLIDE_POS_BOTTOM + 1550;
    public static Integer SLIDE_POS_HIGH = SLIDE_POS_BOTTOM + 1950;
    public static List<Integer> SLIDE_POSITIONS = Arrays.asList(SLIDE_POS_HIGH, SLIDE_POS_MED, SLIDE_POS_LOW, 0);

    public static List<Integer> CONE_STACK_HEIGHTS = Stream.of(130, 110, 90, 70, 50).collect(Collectors.toList());

    public static Double CLAW_POS_FIT = 210.0; // To fit inside the size box
    public static Double CLAW_POS_CLOSED = 160.0;
    public static Double CLAW_POS_OPEN = 250.0;


    public RobotA(HardwareMap hardwareMap) {
        PhotonCore.enable();

        if(CONTROL_HUB.getBulkCachingMode() == LynxModule.BulkCachingMode.OFF){
            CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        if(EXPANSION_HUB != null && EXPANSION_HUB.getBulkCachingMode() == LynxModule.BulkCachingMode.OFF){
            EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        drive = new MecanumDriveCancelable(hardwareMap);
        drive.setPoseEstimate(START_POSE_DEFAULT);

        LinearSlideA.SlideConstants constants = new LinearSlideA.SlideConstants();
        constants.WINCH_TOLERANCE = WINCH_TOLERANCE;
        constants.WINCH_TICKS_PER_INCH = WINCH_TICKS_PER_INCH;
        constants.SLIDE_POS_BOTTOM = SLIDE_POS_BOTTOM;
        constants.SLIDE_POS_GROUND = SLIDE_POS_GROUND;
        constants.SLIDE_POS_LOW = SLIDE_POS_LOW;
        constants.SLIDE_POS_MED = SLIDE_POS_MED;
        constants.SLIDE_POS_HIGH = SLIDE_POS_HIGH;
        constants.SLIDE_POSITIONS = SLIDE_POSITIONS;

        constants.CONE_STACK_HEIGHTS = CONE_STACK_HEIGHTS;

        constants.CLAW_POS_FIT = CLAW_POS_FIT; // To fit inside the size box
        constants.CLAW_POS_CLOSED = CLAW_POS_CLOSED;
        constants.CLAW_POS_OPEN = CLAW_POS_OPEN;

        slide = new LinearSlideA(hardwareMap, constants);

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
        CONTROL_HUB.clearBulkCache();
        EXPANSION_HUB.clearBulkCache();

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
