package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TeamConf.nop;
import static org.firstinspires.ftc.teamcode.TeamConf.within;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.commands.core.LynxResetMotorEncoderCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.lang.reflect.Field;

@Config
public class WinchMotor extends MotorEx {
    public static double ks = 0.0;
    public static double kv = 1.0;
    public static double ka = 0.0;

    public static double kp = 0.006;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;

    private double lastKs = ks;
    private double lastKv = kv;
    private double lastKa = ka;
    private double lastKp = kp;

    private double lastKP = kp;
    private double lastKI = kI;
    private double lastKD = kD;
    private double lastKF = kF;


    public static double holdingPower = 0.1;
    public static boolean hold = true;
    public static double HOME_THRESHOLD = 200;

    public VoltageSensor batteryVoltageSensor;

    public double lastTargetPosition = 0;
    public int lastTargetPositionCurrent = 0;

    public WinchMotor(@NonNull HardwareMap hMap, String id) {
        super(hMap, id);

        batteryVoltageSensor = hMap.voltageSensor.iterator().next();

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.setFeedforwardCoefficients(ks, kv, ka);

        this.setPositionCoefficient(kp);

        //this.setVeloCoefficients(velKp, velKi, velKd);

        lastKP = kp;
        lastKI = kI;
        lastKD = kD;
        lastKF = kF;

        positionController.setP(kp);
        positionController.setI(kI);
        positionController.setD(kD);
        positionController.setF(kF);
    }

    public static double ERROR_THRESHOLD = 200;
    public static double LOWERING_MULT = 0.000001;

    // Threshold at which we can start floating instead of holding down and still reach the setpoint
    public static double DOWN_SLOW_THRESHOLD = 400;

    // Threshold considered "close enough" to goal
    public static double TARGET_THRESHOLD = 30;
    public static double TARGET_UP_THRESHOLD = 20;

    @Override
    public void set(double output) {
        if (lastKP != kp ||
                lastKI != kI ||
                lastKD != kD ||
                lastKF != kF) {
            lastKP = kp;
            lastKI = kI;
            lastKD = kD;
            lastKF = kF;

            positionController.setP(kp);
            positionController.setI(kI);
            positionController.setD(kD);
            positionController.setF(kF);
        }

        if (runmode == RunMode.VelocityControl) {
            double speed = bufferFraction * output * ACHIEVABLE_MAX_TICKS_PER_SECOND;
            double velocity = veloController.calculate(getVelocity(), speed) + feedforward.calculate(speed, encoder.getAcceleration());
            motor.setPower(velocity / ACHIEVABLE_MAX_TICKS_PER_SECOND + getHoldingPower());
        } else if (runmode == RunMode.PositionControl) {
            output = Math.abs(output);
            double setpoint = positionController.getSetPoint();
            int current = getCurrentPosition();
            double error = positionController.calculate(getDistance());

            // Whether the target position was set while beneath it, meaning we want to rise
            boolean startedBelowSetpoint = lastTargetPositionCurrent < setpoint;

            // Targeting home position
            if (setpoint == 0) {
                if (within(current, 0, HOME_THRESHOLD)) {
                    motor.setPower(0.0);
                } else if (startedBelowSetpoint) {
                    // Return to the home point
                    motor.setPower(getHoldingPower());
                } else if (current > DOWN_SLOW_THRESHOLD) {
                    motor.setPower(-1.0);
                } else {
                    // Coast the rest of the way down
                    motor.setPower(0.0);
                }
            }

            // Rising to setpoint
            else if (startedBelowSetpoint) {
                if (within(current, setpoint, TARGET_UP_THRESHOLD)) {
                    // At target, hold
                    motor.setPower(getHoldingPower());
                } else {
                    motor.setPower(output * error);
                }
            }
            // Falling to setpoint
            else {
                if (current + TARGET_THRESHOLD < setpoint) {
                    // We have fallen beneath the setpoint, this should never happen
                    motor.setPower(output * error);
                } else if (within(current, setpoint, TARGET_THRESHOLD)) {
                    // At target, hold
                    motor.setPower(getHoldingPower());
                }
                else if (within(current, setpoint, DOWN_SLOW_THRESHOLD)) {
                    // Pretty close, coast
                    motor.setPower(0.0);
                } else {
                    // Go full speed
                    motor.setPower(-1.0);
                }
            }

            return;
        } else {
            motor.setPower(getHoldingPower() + output);
        }
    }

    public void setHold(boolean status) {
        hold = status;
    }

    public double getHoldingPower() {
        return holdingPower * 12 / batteryVoltageSensor.getVoltage();
    }


    public void setRawPower(double power) {
        motor.setPower(power);
    }

    @Override
    public void setTargetPosition(int target) {
        lastTargetPositionCurrent = getCurrentPosition();
        lastTargetPosition = positionController.getSetPoint();

        super.setTargetPosition(target);
    }

    @Override
    public void resetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        encoder.reset();
    }

    public void forceResetEncoder() {
        try {
            Field f = encoder.getClass().getDeclaredField("resetVal");
            f.setAccessible(true);
            f.setInt(encoder, 0);
        }
        catch (Exception e) {
            nop();
        }
    }
}



/*
if (hold && output < holdingPower && !(holdingPower < 0.0)) {
                motor.setPower(output + getHoldingPower());
            } else {
                motor.setPower(output);
            }
 */

/*

            if (within(getCurrentPosition(), positionController.getSetPoint(), HOME_THRESHOLD) &&
                    positionController.getSetPoint() == 0) {
                // Don't ever apply any holding pressure if our target is zero
                motor.setPower(positionController.getSetPoint() == 0 ? 0.0 : getHoldingPower());
                return;
            }

            double error = positionController.calculate(getDistance());

            double power = output * error;

            if (power < 0.0) {
                // Go down slow, gravity exists
                //motor.setPower(power * LOWERING_MULT);

                // Only coast downwards unless we are going to zero, where it doesn't matter where we land
                //motor.setPower(positionController.getSetPoint() == 0 ? power * LOWERING_MULT : 0.0);

                //motor.setPower(power + getHoldingPower());

                if (within(getCurrentPosition(), positionController.getSetPoint(), DOWN_SLOW_THRESHOLD)) {
                    if (within(getCurrentPosition(), positionController.getSetPoint(), ERROR_THRESHOLD)) {
                        motor.setPower(getHoldingPower());
                    }
                    else {
                        // Float down with whatever velocity is left
                        motor.setPower(0.0);
                    }
                }
                else {
                    motor.setPower(power * LOWERING_MULT);
                }
            }
            else {
                motor.setPower(power + getHoldingPower());
            }
 */