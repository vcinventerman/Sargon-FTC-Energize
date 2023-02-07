package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TeamConf.within;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.commands.core.LynxResetMotorEncoderCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

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
    public static double DOWN_SLOW_THRESHOLD = 200;
    public static double LOWERING_MULT = 0.000001;

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
        }
        else {
            if (hold && output < holdingPower && !(holdingPower < 0.0)) {
                motor.setPower(output + getHoldingPower());
            } else {
                motor.setPower(output);
            }
        }
    }

    public void setHold(boolean status) {
        hold = status;
    }
    public double getHoldingPower() {
        return holdingPower * 12 / batteryVoltageSensor.getVoltage();
    }


    public void setRawPower(int power) {
        motor.setPower(power);
    }
}
