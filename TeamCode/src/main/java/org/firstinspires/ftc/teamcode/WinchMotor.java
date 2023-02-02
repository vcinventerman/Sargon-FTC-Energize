package org.firstinspires.ftc.teamcode;

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

    public static double holdingPower = 0.05;

    public static boolean hold = true;

    public VoltageSensor batteryVoltageSensor;

    public WinchMotor(@NonNull HardwareMap hMap, String id) {
        super(hMap, id);

        batteryVoltageSensor = hMap.voltageSensor.iterator().next();

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.setFeedforwardCoefficients(ks, kv, ka);
    }

    public static double ERROR_THRESHOLD = 100;
    public static double LOWERING_MULT = 0.1;

    @Override
    public void set(double output) {
        if (runmode == RunMode.VelocityControl) {
            double speed = bufferFraction * output * ACHIEVABLE_MAX_TICKS_PER_SECOND;
            double velocity = veloController.calculate(getVelocity(), speed) + feedforward.calculate(speed, encoder.getAcceleration());
            motor.setPower(velocity / ACHIEVABLE_MAX_TICKS_PER_SECOND + getHoldingPower());
        } else if (runmode == RunMode.PositionControl) {
            double error = positionController.calculate(getDistance());

            double power = output * error;

            // Gravity helps us, so reduce the power of going down unless the error is huge
            if (power < 0.0 && error < ERROR_THRESHOLD) {
                power *= LOWERING_MULT;
            }

            if (hold && output < holdingPower && !(holdingPower < 0.0)) {
                motor.setPower(power + getHoldingPower());
            }
            else {
                motor.setPower(power);
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



}
