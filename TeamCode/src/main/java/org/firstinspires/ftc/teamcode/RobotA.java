package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveCancelable;

public class RobotA {
    public MecanumDriveCancelable drive;
    public LinearSlideA slide;

    public RobotA(HardwareMap hardwareMap)
    {
        drive = new MecanumDriveCancelable(hardwareMap);
        slide = new LinearSlideA(hardwareMap);

    }

    public void update(long delta)
    {
        slide.update(delta);
    }
}
