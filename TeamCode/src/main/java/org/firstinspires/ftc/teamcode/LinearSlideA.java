package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TeamConf.CONE_HEIGHT;
import static org.firstinspires.ftc.teamcode.TeamConf.CONE_STACK_OFFSET;
import static org.firstinspires.ftc.teamcode.TeamConf.getDefaultTelemetry;
import static org.firstinspires.ftc.teamcode.TeamConf.nop;
import static org.firstinspires.ftc.teamcode.TeamConf.within;

import android.transition.Slide;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;

import java.lang.reflect.Field;
import java.util.Arrays;
import java.util.List;

@Config
public class LinearSlideA {
    public Motor winch;
    public int currentWinchTarget;
    public boolean winchActive = false;
    public boolean winchManualMode = false;

    public ServoEx claw;

    public VoltageSensor batteryVoltageSensor;

    public static class SlideConstants {
        public Double WINCH_TOLERANCE = 40.0;
        public Double WINCH_TICKS_PER_INCH = (1390.0 / 24.0);
        public Integer SLIDE_POS_BOTTOM = 0;
        public Integer SLIDE_POS_GROUND = SLIDE_POS_BOTTOM + 100;
        public Integer SLIDE_POS_LOW = SLIDE_POS_BOTTOM + 1100;
        public Integer SLIDE_POS_MED = SLIDE_POS_BOTTOM + 1550;
        public Integer SLIDE_POS_HIGH = SLIDE_POS_BOTTOM + 2000;
        public List<Integer> SLIDE_POSITIONS = Arrays.asList(SLIDE_POS_HIGH, SLIDE_POS_MED, SLIDE_POS_LOW);

        public List<Integer> CONE_STACK_HEIGHTS = Arrays.asList(130, 110, 90, 70, 50);

        public Double CLAW_POS_FIT = 210.0; // To fit inside the size box
        public Double CLAW_POS_CLOSED = 110.0;
        public Double CLAW_POS_OPEN = 180.0;

        public InterpLUT STALL_POWER = new InterpLUT();
    }

    public SlideConstants p; // Slide specific parameters


    public LinearSlideA(HardwareMap hardwareMap, SlideConstants constants)
    {
        p = (constants == null ? new SlideConstants() : constants);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        currentWinchTarget = p.SLIDE_POS_BOTTOM;

        winch = new WinchMotor(hardwareMap, "winch");
        winch.setRunMode(Motor.RunMode.PositionControl);
        //winch.setPositionCoefficient(p.WINCH_COEFFICIENT);
        winch.setPositionTolerance(p.WINCH_TOLERANCE);
        winch.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        winchActive = false;

        //claw = hardwareMap.get(Servo.class, "claw");
        claw = new SimpleServo(hardwareMap, "claw", 0, 360);
        setClawTarget(p.CLAW_POS_OPEN);
    }
    public LinearSlideA(HardwareMap hardwareMap, Motor liftMotor, ServoEx claw)
    {
        p = new SlideConstants();

        currentWinchTarget = p.SLIDE_POS_BOTTOM;

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

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

        this.claw = claw;

        this.winch = liftMotor;
        winch.setRunMode(Motor.RunMode.PositionControl);
        //winch.setRunMode(Motor.RunMode.RawPower);
        winch.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        winchActive = false;

        this.claw = claw;
        claw.setRange(0, 360);
    }

    public static Double WINCH_SPEED = 1.0;
    public static Double WINCH_LAND_SPEED = 0.1;
    public static Double WINCH_HOLD_SPEED = 0.003;
    public static Double WINCH_LOWER_SPEED = 0.2;
    public static Double WINCH_NEAR_SPEED = 0.2;

    public static Double WINCH_HOLD_TOLERANCE = 50.0;

    public void update()
    {
        if (!winchManualMode) {
            winch.set(WINCH_SPEED);
        }
    }

    public Motor.RunMode getWinchRunMode() {
        try {
            //Field f = winch.getClass().getDeclaredField("runmode");
            Field f = Motor.class.getDeclaredField("runmode");
            f.setAccessible(true);
            Motor.RunMode runMode = (Motor.RunMode) f.get(winch);

            return runMode;
        }
        catch (Exception e) {
            return Motor.RunMode.RawPower;
        }
    }


    public void setCurrentWinchTarget(int target) {
        currentWinchTarget = target;
        winchActive = true;
        if (getWinchRunMode() != Motor.RunMode.PositionControl) {
            winch.setRunMode(Motor.RunMode.PositionControl);
        }
        winch.setTargetPosition(currentWinchTarget);
    }

    public void goToNextSlidePos()
    {
        int index = p.SLIDE_POSITIONS.indexOf(currentWinchTarget);
        setCurrentWinchTarget(p.SLIDE_POSITIONS.get((index + 1) % (p.SLIDE_POSITIONS.size())));
    }

    public void enableAutomaticWinch() {
        if (getWinchRunMode() != Motor.RunMode.PositionControl) {
            winch.setRunMode(Motor.RunMode.PositionControl);
        }
        currentWinchTarget = winch.getCurrentPosition();
        winch.setTargetPosition(winch.getCurrentPosition());
        winchManualMode = false;
    }

    public Integer coneStackState = 0;
    public void goToNextConeStackHeight() {
        List<Double> coneStackHeights = Arrays.asList(CONE_HEIGHT + CONE_STACK_OFFSET * 4,
                CONE_HEIGHT + CONE_STACK_OFFSET * 3, CONE_HEIGHT + CONE_STACK_OFFSET * 2,
                CONE_HEIGHT + CONE_STACK_OFFSET * 1, CONE_HEIGHT);

        setCurrentWinchTarget((int)(coneStackHeights.get(coneStackState) * p.WINCH_TICKS_PER_INCH));

        coneStackState = coneStackState >= p.CONE_STACK_HEIGHTS.size() - 1 ? 0 : coneStackState + 1;
    }

    public int getNextConeStackHeight() {
        List<Double> coneStackHeights = Arrays.asList(CONE_HEIGHT + CONE_STACK_OFFSET * 4,
                CONE_HEIGHT + CONE_STACK_OFFSET * 3, CONE_HEIGHT + CONE_STACK_OFFSET * 2,
                CONE_HEIGHT + CONE_STACK_OFFSET * 1, CONE_HEIGHT);

        int prevConeStackState = coneStackState;

        coneStackState = coneStackState >= p.CONE_STACK_HEIGHTS.size() - 1 ? 0 : coneStackState + 1;

        return ((int)(coneStackHeights.get(prevConeStackState) * p.WINCH_TICKS_PER_INCH));
    }

    public void disableAutomaticWinch() {
        winch.setRunMode(Motor.RunMode.RawPower);
        winchActive = false;
        winchManualMode = true;
    }

    public void setClawTarget(double degrees) {
        claw.turnToAngle(degrees);
    }
}
