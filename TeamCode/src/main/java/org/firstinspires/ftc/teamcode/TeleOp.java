package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.karrmedia.ftchotpatch.SupervisedOpMode;
import com.karrmedia.ftchotpatch.Supervised;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveCancelable;

@Supervised(name="?TeleOp", group="Iterative Opmode", autonomous=false, variations={"Red", "Blue"})
public class TeleOp extends SupervisedOpMode {
    MecanumDriveCancelable drive;

    // Code that runs when the INIT button is pressed (mandatory)
    public void init() {
        drive = new MecanumDriveCancelable(hardwareMap);
    }

    public void start() {

    }

    // Code that runs repeatedly after the PLAY button is pressed (optional)
    public void loop() {
        drive.setWeightedDrivePower(
                new Pose2d(
                        gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        (gamepad1.left_trigger - gamepad1.right_trigger) * 5
                )
        );
    }

    // Code that runs when the OpMode is stopped (optional)
    public void stop() {

    }

    // Code that runs after this OpMode is dynamically updated
    public void hotpatch() {

    }


}