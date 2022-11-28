package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.acmerobotics.dashboard.config.Config;

import java.util.Arrays;

@Config
public class LinearSlideA {
    public MotorEx winch;
    public int currentWinchTarget = SLIDE_POS_BOTTOM;
    public static volatile double WINCH_COEFFICIENT = 1.0;
    public static volatile int SLIDE_POS_TALL = 1000;
    public static volatile int SLIDE_POS_MED = 500;
    public static volatile int SLIDE_POS_SHORT = 200;
    public static volatile int SLIDE_POS_BOTTOM = 10;

    public static volatile int[] SLIDE_POSITIONS = { SLIDE_POS_BOTTOM, SLIDE_POS_SHORT, SLIDE_POS_MED, SLIDE_POS_TALL };


    public ServoEx claw;
    public static volatile double CLAW_POS_OFFSET = 0.0;


    public LinearSlideA(HardwareMap hardwareMap)
    {
        winch = new MotorEx(hardwareMap, "winch");
        winch.setRunMode(Motor.RunMode.PositionControl);
        winch.setPositionCoefficient(WINCH_COEFFICIENT);
        winch.setPositionTolerance(1);


        claw = new SimpleServo(hardwareMap, "claw", 0, 360);
    }

    public void update(long delta)
    {
        if (!winch.atTargetPosition()) {
            winch.set(0.75);
        }
        else {
            winch.set(0.0);
        }
    }

    public void goToTop()
    {
        winch.setTargetPosition(SLIDE_POS_TOP);
    }

    public void setCurrentWinchTarget(int target) {
        currentWinchTarget = target;
        winch.setTargetPosition(currentWinchTarget);
    }

    public void goToNextSlidePos()
    {
        int index = Arrays.asList(SLIDE_POSITIONS).indexOf(currentWinchTarget);
        setCurrentWinchTarget(SLIDE_POSITIONS[(index + 1) % (SLIDE_POSITIONS.length - 1)]);
    }
}
