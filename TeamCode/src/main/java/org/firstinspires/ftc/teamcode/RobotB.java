package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.MecanumDriveCancelable.getAccelerationConstraint;
import static org.firstinspires.ftc.teamcode.drive.MecanumDriveCancelable.getVelocityConstraint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

@Config
public class RobotB {
    public SampleTankDrive drive;
    public LinearSlideA slide;

    public HDrive manualDrive;

    public static String NAME_CLAW = "claw";
    public static String NAME_SLIDE = "lift";

    public static String NAME_LEFT = "left";
    public static String NAME_RIGHT = "right";
    public static String NAME_CENTER = "center";



    public RobotB(HardwareMap hardwareMap) {
        PhotonCore.enable();
        //PhotonCore.experimental.setSinglethreadedOptimized(false);

        drive = null;
        slide = new LinearSlideA(hardwareMap);

        manualDrive = new HDrive(
                new Motor(hardwareMap, NAME_LEFT),
                new Motor(hardwareMap, NAME_RIGHT),
                new Motor(hardwareMap, NAME_CENTER),
                0.0, 0.0, Math.PI);
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
