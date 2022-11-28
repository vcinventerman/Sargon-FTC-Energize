package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.karrmedia.ftchotpatch.Supervised;
import com.karrmedia.ftchotpatch.SupervisedOpMode;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveCancelable;

@Supervised(name="?TeleOp", group="Autonomous Opmode", autonomous=true, variations={"RedLeft", "RedRight", "BlueLeft", "BlueRighta"})
public class Auto extends SupervisedOpMode {
    RobotA robot;
    SignalSleeveDetector detector;


    // Code that runs when the INIT button is pressed (mandatory)
    public void init() {
        robot = new RobotA(hardwareMap);

        // Detect AprilTags until we start
        //detector = AprilTagDetector(hardwareMap, List.of(18, 19, 20));


    }

    public void start() {
        //if (detector.)
    }

    // Code that runs repeatedly after the PLAY button is pressed (optional)
    public void loop() {



    }

    // Code that runs when the OpMode is stopped (optional)
    public void stop() {

    }

    // Code that runs after this OpMode is dynamically updated
    public void hotpatch() {

    }
}
