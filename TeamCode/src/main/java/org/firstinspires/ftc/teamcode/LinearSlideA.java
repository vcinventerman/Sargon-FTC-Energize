package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TeamConf.getDefaultTelemetry;
import static org.firstinspires.ftc.teamcode.TeamConf.within;

import android.transition.Slide;

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
    public Motor winch;
    public int currentWinchTarget;
    public boolean winchActive = false;
    public boolean winchManualMode = false;

    Thread slideThread;

    public ServoEx claw;
    public static double clawTarget;
    public boolean clawActive = false;
    public boolean clawManualMode = false;

    public static double CLAW_POS_FIT = 210; // To fit inside the size box
    public static double CLAW_POS_CLOSED = 95;
    public static double CLAW_POS_OPEN = 180;

    public static class SlideConstants {
        public double WINCH_TOLERANCE = 40.0;
        public Integer SLIDE_POS_BOTTOM = 0;
        public Integer SLIDE_POS_GROUND = SLIDE_POS_BOTTOM + 100;
        public Integer SLIDE_POS_LOW = SLIDE_POS_BOTTOM + 1100;
        public Integer SLIDE_POS_MED = SLIDE_POS_BOTTOM + 1550;
        public Integer SLIDE_POS_HIGH = SLIDE_POS_BOTTOM + 1900;
        public List<Integer> SLIDE_POSITIONS = Arrays.asList(SLIDE_POS_LOW, SLIDE_POS_MED);

        public List<Integer> CONE_STACK_HEIGHTS = Arrays.asList(130, 110, 90, 70, 50);

        /*public double CLAW_POS_FIT = 210; // To fit inside the size box
        public double CLAW_POS_CLOSED = 110;
        public double CLAW_POS_OPEN = 180;*/
    }

    public static SlideConstants p; // Slide specific parameters


    public LinearSlideA(HardwareMap hardwareMap)
    {
        p = new SlideConstants();

        currentWinchTarget = p.SLIDE_POS_BOTTOM;
        clawTarget = CLAW_POS_OPEN;

        winch = new MotorEx(hardwareMap, "winch");
        winch.setRunMode(Motor.RunMode.PositionControl);
        //winch.setPositionCoefficient(p.WINCH_COEFFICIENT);
        winch.setPositionTolerance(p.WINCH_TOLERANCE);
        winch.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        winchActive = false;

        //claw = hardwareMap.get(Servo.class, "claw");
        claw = new SimpleServo(hardwareMap, "claw", 0, 360);
        clawManualMode = false;
        clawActive = false;
    }
    public LinearSlideA(HardwareMap hardwareMap, Motor liftMotor, ServoEx claw)
    {
        p = new SlideConstants();

        currentWinchTarget = p.SLIDE_POS_BOTTOM;
        clawTarget = CLAW_POS_OPEN;

        /*winch = new MotorEx(hardwareMap, "winch");
        winch.setRunMode(Motor.RunMode.PositionControl);
        winch.setPositionCoefficient(WINCH_COEFFICIENT);
        winch.setPositionTolerance(WINCH_TOLERANCE);
        winch.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        winchActive = false;

        //claw = hardwareMap.get(Servo.class, "claw");
        claw = new SimpleServo(hardwareMap, "claw", 0, 360);
        clawManualMode = false;
        clawActive = false;*/

        this.winch = liftMotor;
        //winch.setRunMode(Motor.RunMode.PositionControl);
        winch.setRunMode(Motor.RunMode.RawPower);
        winch.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        winchActive = false;

        this.claw = claw;
        claw.setRange(0, 360);
        clawActive = false;
    }

    public LinearSlideA(HardwareMap hardwareMap, String winchName, String clawName) {
        p = new SlideConstants();

        currentWinchTarget = p.SLIDE_POS_BOTTOM;
        clawTarget = CLAW_POS_OPEN;

        winch = new MotorEx(hardwareMap, winchName);
        winch.setRunMode(Motor.RunMode.PositionControl);
        winch.setPositionTolerance(p.WINCH_TOLERANCE);
        winch.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        winchActive = false;

        //claw = hardwareMap.get(Servo.class, "claw");
        claw = new SimpleServo(hardwareMap, clawName, 0, 360);
        clawManualMode = false;
        clawActive = false;
    }

    public static Double WINCH_SPEED = 0.5;
    public static Double WINCH_LAND_SPEED = 0.1;

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
        /*if ((winch.getCurrentPosition() > (p.SLIDE_POS_HIGH + 100)) && !winchManualMode) {
            setCurrentWinchTarget(p.SLIDE_POS_LOW);
            winchActive = false;
            return;
        }*/

        //if (winch.atTargetPosition() && !winchActive) {
            //if (!within(winch.getCurrentPosition(), currentWinchTarget, 50)) {
            //    setCurrentWinchTarget(currentWinchTarget);
            //}
            /*else {
                winch.setTargetPosition(winch.getCurrentPosition());
                winch.set(0.5);
            }*/
        //}

        if (!winch.atTargetPosition() && winchActive && !winchManualMode) {
            if (winch.getCurrentPosition() < 200 && currentWinchTarget == p.SLIDE_POS_BOTTOM) {
                winch.set(WINCH_LAND_SPEED);
            }
            else if (within(winch.getCurrentPosition(), currentWinchTarget, 150)) {
                winch.set(WINCH_SPEED);
            }
            else {
                winch.set(WINCH_SPEED);
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
        int index = p.SLIDE_POSITIONS.indexOf(currentWinchTarget);
        setCurrentWinchTarget(p.SLIDE_POSITIONS.get((index + 1) % (p.SLIDE_POSITIONS.size())));
    }

    public void enableAutomaticWinch() {
        winch.setRunMode(Motor.RunMode.PositionControl);
        winch.setTargetPosition(winch.getCurrentPosition());
        winchManualMode = false;
    }

    public Integer coneStackState = 0;
    public void goToNextConeStackHeight() {
        setCurrentWinchTarget(p.CONE_STACK_HEIGHTS.get(coneStackState));

        coneStackState = coneStackState >= p.CONE_STACK_HEIGHTS.size() - 1 ? 0 : coneStackState + 1;
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
