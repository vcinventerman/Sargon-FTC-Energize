package org.firstinspires.ftc.teamcode;

import static com.outoftheboxrobotics.photoncore.PhotonCore.CONTROL_HUB;
import static com.outoftheboxrobotics.photoncore.PhotonCore.EXPANSION_HUB;
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
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

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


    public static Double WINCH_TOLERANCE = 50.0;
    public static Double WINCH_TICKS_PER_INCH = (1390.0 / 24.0);
    public static Integer SLIDE_POS_BOTTOM = 0;
    public static Integer SLIDE_POS_GROUND = SLIDE_POS_BOTTOM + 100;
    public static Integer SLIDE_POS_LOW = SLIDE_POS_BOTTOM + 950;
    public static Integer SLIDE_POS_MED = SLIDE_POS_BOTTOM + 1550;
    public static Integer SLIDE_POS_HIGH = SLIDE_POS_BOTTOM + 1950;
    public static List<Integer> SLIDE_POSITIONS = Arrays.asList(SLIDE_POS_MED, SLIDE_POS_LOW, 0);

    public static List<Integer> CONE_STACK_HEIGHTS = Stream.of(130, 110, 90, 70, 50).collect(Collectors.toList());

    public static Double CLAW_POS_FIT = 210.0; // To fit inside the size box
    public static Double CLAW_POS_CLOSED = 160.0;
    public static Double CLAW_POS_OPEN = 250.0;



    public RobotB(HardwareMap hardwareMap) {
        PhotonCore.enable();

        drive = new org.firstinspires.ftc.teamcode.drive.HDrive(hardwareMap);

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
