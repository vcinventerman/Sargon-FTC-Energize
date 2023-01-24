package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.MecanumDriveCancelable.getAccelerationConstraint;
import static org.firstinspires.ftc.teamcode.drive.MecanumDriveCancelable.getVelocityConstraint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

@Config
public class RobotB {
    public org.firstinspires.ftc.teamcode.drive.HDrive drive;
    public LinearSlideA slide;

    public HDrive manualDrive;

    public static String NAME_CLAW = "claw";
    public static String NAME_SLIDE = "lift";

    public static String NAME_LEFT = "BackLeft";
    public static String NAME_RIGHT = "BackRight";
    public static String NAME_CENTER = "H drive";

    public static int CENTER_CPR = 288;
    public static int CENTER_RPM = 137;

    public static String WINCH_NAME = "Lift 1";
    public static String CLAW_NAME = "Servo";



    public RobotB(HardwareMap hardwareMap) {
        PhotonCore.enable();
        //PhotonCore.experimental.setSinglethreadedOptimized(false);

        drive = new org.firstinspires.ftc.teamcode.drive.HDrive(hardwareMap);
        slide = new LinearSlideA(hardwareMap, WINCH_NAME, CLAW_NAME);

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
}
