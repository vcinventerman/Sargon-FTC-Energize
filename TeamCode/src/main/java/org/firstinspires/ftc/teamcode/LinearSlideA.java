package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TeamConf.getDefaultTelemetry;
import static org.firstinspires.ftc.teamcode.TeamConf.within;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Arrays;
import java.util.List;

@Config
public class LinearSlideA {
    public static volatile double WINCH_COEFFICIENT = 1.0;
    public static volatile double WINCH_TOLERANCE = 40.0;
    public static volatile Integer SLIDE_POS_BOTTOM = 0;

    public static boolean ENABLE_BOTTOM_HEIGHT = false;
    public static boolean CAN_REACH_HIGH = true;

    // Junction heights
    public static volatile Integer SLIDE_POS_GROUND = SLIDE_POS_BOTTOM + 100;
    public static volatile Integer SLIDE_POS_LOW = SLIDE_POS_BOTTOM + 1161;
    public static volatile Integer SLIDE_POS_MED = SLIDE_POS_BOTTOM + 1500;
    public static volatile Integer SLIDE_POS_HIGH = SLIDE_POS_BOTTOM + 2200;
    public static List<Integer> SLIDE_POSITIONS = Arrays.asList(SLIDE_POS_GROUND, SLIDE_POS_LOW, SLIDE_POS_MED, SLIDE_POS_HIGH);

    // In linear slide ticks
    public static List<Integer> CONE_STACK_HEIGHTS = Arrays.asList(130, 110, 90, 70, 50);
    public static int CONE_STACK_OFFSET = 20;

    public MotorEx winch;
    public int currentWinchTarget = SLIDE_POS_BOTTOM;
    public boolean winchActive = false;
    public boolean winchManualMode = false;


    public static volatile double CLAW_POS_FIT = 210; // To fit inside the size box
    public static volatile double CLAW_POS_CLOSED = 110;
    public static volatile double CLAW_POS_OPEN = 180;
    public static volatile double CLAW_POS_OFFSET = 0;

    Thread slideThread;

    public ServoEx claw;
    public static double clawTarget = CLAW_POS_OPEN;
    public boolean clawActive = false;
    public boolean clawManualMode = false;



    public LinearSlideA(HardwareMap hardwareMap)
    {
        if (!CAN_REACH_HIGH) { SLIDE_POSITIONS.remove(SLIDE_POSITIONS.size() - 1); }

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
            winchActive = false;
            return;
        }

        if (winch.atTargetPosition() && !winchActive) {
            if (!within(winch.getCurrentPosition(), currentWinchTarget, 50)) {
                setCurrentWinchTarget(currentWinchTarget);
            }
            else {
                winch.setTargetPosition(winch.getCurrentPosition());
                winch.set(0.5);
            }
        }

        if (!winch.atTargetPosition() && winchActive && !winchManualMode) {
            if (within(winch.getCurrentPosition(), currentWinchTarget, 150)) {
                winch.set(0.5);
            }
            else {
                winch.set(1.0);
            }
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

    public Integer coneStackState = 0;
    public void goToNextConeStackHeight() {
        setCurrentWinchTarget(CONE_STACK_HEIGHTS.get(coneStackState));

        coneStackState = coneStackState >= CONE_STACK_HEIGHTS.size() - 1 ? 0 : coneStackState + 1;
    }

    public void waitToPassConeStack() {
        int safeHeight = CONE_STACK_HEIGHTS.get(coneStackState - 1) + CONE_STACK_OFFSET;

        if (currentWinchTarget <= CONE_STACK_HEIGHTS.get(coneStackState - 1)) {
            RobotLog.e("Invalid invocation of waitToPassConeStack!");
        }

        while (winch.getCurrentPosition() < safeHeight) {
            update();
        }
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

    void runInThread() {

    }

    public void setClawTarget(double target) {
        clawTarget = target;
        clawActive = true;
    }
}
