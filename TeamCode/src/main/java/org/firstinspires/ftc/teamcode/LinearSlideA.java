package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TeamConf.within;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;
import java.util.List;

@Config
public class LinearSlideA {
    public static volatile double WINCH_COEFFICIENT = 1.0;
    public static volatile double WINCH_TOLERANCE = 40.0;
    public static volatile Integer SLIDE_POS_BOTTOM = 0;

    // Junction heights
    public static volatile Integer SLIDE_POS_GROUND = SLIDE_POS_BOTTOM + 1000;
    public static volatile Integer SLIDE_POS_LOW = SLIDE_POS_BOTTOM + 7800;
    public static volatile Integer SLIDE_POS_MED = SLIDE_POS_BOTTOM + 12500;
    public static volatile Integer SLIDE_POS_HIGH = SLIDE_POS_BOTTOM + 17000;
    public static List<Integer> SLIDE_POSITIONS = Arrays.asList(SLIDE_POS_BOTTOM, SLIDE_POS_GROUND, SLIDE_POS_LOW, SLIDE_POS_MED, SLIDE_POS_HIGH);

    public MotorEx winch;
    public int currentWinchTarget = SLIDE_POS_BOTTOM;
    public boolean winchActive = false;
    public boolean winchManualMode = false;


    public static volatile double CLAW_POS_FIT = 245; // To fit inside the size box
    public static volatile double CLAW_POS_CLOSED = 60;
    public static volatile double CLAW_POS_OPEN = 150;
    public static volatile double CLAW_POS_OFFSET = 0;

    Thread slideThread;

    public ServoEx claw;
    public static double clawTarget = CLAW_POS_OPEN;
    public boolean clawActive = false;
    public boolean clawManualMode = false;



    public LinearSlideA(HardwareMap hardwareMap)
    {
        winch = new MotorEx(hardwareMap, "winch");
        winch.setRunMode(Motor.RunMode.PositionControl);
        winch.setPositionCoefficient(WINCH_COEFFICIENT);
        winch.setPositionTolerance(WINCH_TOLERANCE);
        winch.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        winchActive = false;

        //claw = hardwareMap.get(Servo.class, "claw");
        claw = new SimpleServo(hardwareMap, "claw", 0, 360);
        clawManualMode = false;
        clawActive = false;
    }

    public void update()
    {
        if (!within(claw.getAngle(), clawTarget, 1) && clawActive && !clawManualMode) {
            claw.turnToAngle(clawTarget);
        }
        else if (!clawManualMode && clawActive) {
            clawActive = false;
        }
        else {

        }

        // Emergency failsafe
        if ((winch.getCurrentPosition() > (SLIDE_POS_HIGH + 100)) && !winchManualMode) {
            setCurrentWinchTarget(SLIDE_POS_LOW);
            return;
        }

        if (!winch.atTargetPosition() && winchActive && !winchManualMode) {
            winch.set(1.0);
        }
        else if (!winchManualMode && winchActive) {
            // Currently at target
            winch.set(0.0);
            winchActive = false;
        }
        else {
            // Winch will be manually moved by controller
        }


    }


    public void setCurrentWinchTarget(int target) {
        currentWinchTarget = target;
        winchActive = true;
        winch.setTargetPosition(currentWinchTarget);
    }

    public void goToNextSlidePos()
    {
        int index = SLIDE_POSITIONS.indexOf(currentWinchTarget);
        setCurrentWinchTarget(SLIDE_POSITIONS.get((index + 1) % (SLIDE_POSITIONS.size())));
    }

    public void enableAutomaticWinch() {
        winch.setRunMode(Motor.RunMode.PositionControl);
        winch.setTargetPosition(winch.getCurrentPosition());
        winchManualMode = false;
    }

    public void disableAutomaticWinch() {
        winch.setRunMode(Motor.RunMode.RawPower);
        winchActive = false;
        winchManualMode = true;
    }

    public void setClawState(double degrees) {
        claw.turnToAngle(degrees);
        //todo: stick
    }

    public void waitToReachWinchTarget() {
        while (!winch.atTargetPosition() && winchActive) {
            update();
        }
    }

    void runInThread() {

    }

    public void setClawTarget(double target) {
        clawTarget = target;
    }
}
