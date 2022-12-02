package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TeamConf.TAG_OFFSET;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class AprilTagDetector extends SignalSleeveDetector {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    Timing.Timer runTimer = new Timing.Timer(100, TimeUnit.MILLISECONDS); // Restrict to scanning every 100ms

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    int[] DESIRED_TAG_IDS = { 18, 19, 20 };

    List<Integer> tagsToLookFor;
    boolean detectFail = false;
    int tagSeen = -1;

    boolean isTagDesired(AprilTagDetection tag)
    {
        for (int i = 0; i < DESIRED_TAG_IDS.length; i++)
        {
            if (DESIRED_TAG_IDS[i] == tag.id) { return true; }
        }
        return false;
    }


    public AprilTagDetector(HardwareMap hardwareMap, List<Integer> desiredTags)
    {
        this.tagsToLookFor = desiredTags;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, TeamConf.ROBOT_CAMERA_ORIENTATION);
            }

            @Override
            public void onError(int errorCode)
            {
                RobotLog.e("Failed to open OpenCv Camera!");
                detectFail = true;
            }
        });

        runTimer.start();
    }

    @Override
    void detect() {
        if (detectFail || !runTimer.done()) { return; }

        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if(currentDetections.size() != 0)
        {
            for(AprilTagDetection tag : currentDetections)
            {
                if (tagsToLookFor.stream().anyMatch(i -> i == tag.id))
                {
                    tagSeen = tag.id - TAG_OFFSET;
                    break;
                }
            }
        }

        runTimer.start();
    }

    public boolean done()
    {
        return detectFail || (tagSeen != -1);
    }

    public void stop()
    {
        camera.stopStreaming();
    }

    public int getTagSeenOrDefault(int dfault)
    {
        if (tagSeen != -1) { return tagSeen; }
        else { return dfault; }
    }
}
