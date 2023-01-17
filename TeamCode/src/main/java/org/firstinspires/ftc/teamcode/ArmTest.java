package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.karrmedia.ftchotpatch.SupervisedOpMode;

//@Supervised(name="ArmTest", autonomous=false)
public class ArmTest extends SupervisedOpMode {
    MotorEx winch;
    ServoEx claw;

    public void init() {
        winch = new MotorEx(hardwareMap, "winch");
        claw = new SimpleServo(hardwareMap, "claw", 0, 360);

        winch.setRunMode(Motor.RunMode.RawPower);
    }

    public void loop() {
        //winch.set(gamepad1.left_stick_y);
        //claw.turnToAngle(-gamepad1.right_stick_y / 360.0);

        
    }
}
