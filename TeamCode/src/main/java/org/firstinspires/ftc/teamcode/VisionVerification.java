package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sun.tools.javac.util.List;

// Tests if the AprilTag is visible and reports back what it sees continuously
@Autonomous(group="!CompAuto")
public class VisionVerification extends LinearOpMode {
    AprilTagDetector detector;

    @Override
    public void runOpMode()
    {
        detector = new AprilTagDetector(hardwareMap, List.of(21, 22, 23));

        while (!isStarted() && !isStopRequested())
        {
            detector.detect();

            if (detector.done()) {
                telemetry.addData("AprilTag", detector.getTagSeenOrDefault(-100));
                telemetry.update();
            }

            TeamConf.sleep(50);
        }

        detector.stop();
    }
}
