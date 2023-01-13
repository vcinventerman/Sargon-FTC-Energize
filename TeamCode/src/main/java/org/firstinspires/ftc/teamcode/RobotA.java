package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class RobotA {
    public SampleMecanumDrive drive;
    public LinearSlideA slide;

    public RobotA(HardwareMap hardwareMap) {
        PhotonCore.enable();
        //PhotonCore.experimental.setSinglethreadedOptimized(false);

        drive = new SampleMecanumDrive(hardwareMap);
        slide = new LinearSlideA(hardwareMap);
    }

    public RobotA(HardwareMap hardwareMap, Pose2d startPose) {
        this(hardwareMap);
        drive.setPoseEstimate(startPose);
    }

    public void update()
    {
        slide.update();
        drive.update();
    }
}
