package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import android.os.Build;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Function;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.lang.reflect.Method;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

@com.acmerobotics.dashboard.config.Config
public class TeamConf {


    // Field is 11ft 9in
    public static double FIELD_WIDTH = 141;
    // Tiles are 2ft
    public static double TILE_SIZE = 24;

    // Field from the point of view of the red alliance: +y is towards blue, -y is towards red, +x is towards red right
    public static double FIELD_BEARING_NORTH = Math.PI / 2;
    public static double FIELD_BEARING_SOUTH = Math.PI * (3.0 / 2.0);
    public static double FIELD_BEARING_EAST = 0.0;
    public static double FIELD_BEARING_WEST = Math.PI;

    // Senior team robot
    public static double ROBOTA_LENGTH = 15;
    public static double ROBOTA_WIDTH = 14;
    public static String ROBOTA_IMU_DEFAULT = "imu1"; // imu0 is in control hub, imu1 is in expansion hub
    public static Vector2d ROBOTA_CLAW_OFFSET = new Vector2d(8.5, 0.0);

    // Freshman team robot
    public static double ROBOTB_LENGTH = 18;
    public static double ROBOTB_WIDTH = 18;
    public static String ROBOTB_IMU_DEFAULT = "imu"; // imu0 is in control hub, imu1 is in expansion hub
    public static Vector2d ROBOTB_CLAW_OFFSET = new Vector2d(8.5, 0.0);

    // Choose current robot here
    public static double ROBOT_LENGTH = ROBOTA_LENGTH;
    public static double ROBOT_WIDTH = ROBOTA_WIDTH;
    public static String ROBOT_IMU_DEFAULT = ROBOTA_IMU_DEFAULT;
    public static Class ROBOT_DRIVE = RobotA.class;
    public static MecanumDriveCancelable ROBOT_DRIVE_INST = null;
    public static Class ROBOT_MANUAL_DRIVE = MecanumDrive.class;
    // Distance from center to claw
    public static Vector2d ROBOT_CLAW_OFFSET = ROBOTA_CLAW_OFFSET;

    public static Pose2d START_POS_RED_LEFT = new Pose2d(-35, -(FIELD_WIDTH / 2) + (ROBOT_LENGTH / 2), FIELD_BEARING_NORTH);
    public static Pose2d START_POS_RED_RIGHT = new Pose2d(35, -(FIELD_WIDTH / 2) + (ROBOT_LENGTH / 2), FIELD_BEARING_NORTH);
    public static Pose2d START_POS_BLUE_LEFT = new Pose2d(35, (FIELD_WIDTH / 2) - (ROBOT_LENGTH / 2), FIELD_BEARING_SOUTH);
    public static Pose2d START_POS_BLUE_RIGHT = new Pose2d(-35, (FIELD_WIDTH / 2) - (ROBOT_LENGTH / 2), FIELD_BEARING_SOUTH);
    public static List<Pose2d> START_POSITIONS = Arrays.asList(START_POS_RED_LEFT, START_POS_RED_RIGHT, START_POS_BLUE_LEFT, START_POS_BLUE_RIGHT);

    public static Pose2d START_POSE_DEFAULT = START_POS_RED_LEFT;

    // Measurements from https://cdn.andymark.com/media/W1siZiIsIjIwMjIvMDkvMDYvMTYvMzYvNTUvODgxZjAzNDctNThhYS00MzNhLTkwMTEtYTFiYWIwMTIwNzg5L2FtLTQ4MDFfYmx1ZSBhbS00ODAxX3JlZCBDb25lIFJFVjIucGRmIl1d/am-4801_blue%20am-4801_red%20Cone%20REV2.pdf?sha=593981254c44ff81
    public static double CONE_DIAMETER = 4.0;
    public static double CONE_HEIGHT = 3.65 + 1.23;

    public static Pose2d CONE_STACK_POS_RED_LEFT = new Pose2d(-FIELD_WIDTH + CONE_DIAMETER / 2.0, -TILE_SIZE / 2.0, Math.PI);
    public static Pose2d CONE_STACK_POS_RED_RIGHT = new Pose2d((TILE_SIZE * 3) - CONE_DIAMETER / 2.0, -TILE_SIZE / 2.0, 0);
    public static Pose2d CONE_STACK_POS_BLUE_LEFT = new Pose2d(FIELD_WIDTH + CONE_DIAMETER / 2.0, TILE_SIZE / 2.0, 0);
    public static Pose2d CONE_STACK_POS_BLUE_RIGHT = new Pose2d(-FIELD_WIDTH - CONE_DIAMETER / 2.0, TILE_SIZE / 2.0, Math.PI);
    public static List<Pose2d> CONE_STACK_POSITIONS = Arrays.asList(CONE_STACK_POS_RED_LEFT, CONE_STACK_POS_RED_RIGHT, CONE_STACK_POS_BLUE_LEFT, CONE_STACK_POS_BLUE_RIGHT);


    public enum JunctionHeight {
        GROUND,
        LOW,
        MEDIUM,
        HIGH
    }

    public static List<Vector2d> JUNCTIONS = Arrays.asList(
            // High
            new Vector2d(0, TILE_SIZE * 1),
            new Vector2d(0, TILE_SIZE * -1),
            new Vector2d(TILE_SIZE * 1, 0),
            new Vector2d(TILE_SIZE * -1, 0),

            // Medium
            new Vector2d(TILE_SIZE * 1, TILE_SIZE * 1),
            new Vector2d(TILE_SIZE * 1, TILE_SIZE * -1),
            new Vector2d(TILE_SIZE * -1, TILE_SIZE * 1),
            new Vector2d(TILE_SIZE * -1, TILE_SIZE * -1),

            // Low
            new Vector2d(TILE_SIZE * 1, TILE_SIZE * 2),
            new Vector2d(TILE_SIZE * 2, TILE_SIZE * 1),

            new Vector2d(TILE_SIZE * 1, TILE_SIZE * -2),
            new Vector2d(TILE_SIZE * 2, TILE_SIZE * -1),

            new Vector2d(TILE_SIZE * 1, TILE_SIZE * -2),
            new Vector2d(TILE_SIZE * 2, TILE_SIZE * -1),

            new Vector2d(TILE_SIZE * 1, TILE_SIZE * 2),
            new Vector2d(TILE_SIZE * 2, TILE_SIZE * 1),

            // Ground
            new Vector2d(0, 0),
            new Vector2d(TILE_SIZE * 2, TILE_SIZE),
            new Vector2d(TILE_SIZE * -2, TILE_SIZE),
            new Vector2d(TILE_SIZE, TILE_SIZE * 2),
            new Vector2d(TILE_SIZE, TILE_SIZE * -2),
            new Vector2d(TILE_SIZE * 2, TILE_SIZE * 2),
            new Vector2d(TILE_SIZE * 2, TILE_SIZE * -2),
            new Vector2d(TILE_SIZE * -2, TILE_SIZE * 2),
            new Vector2d(TILE_SIZE * -2, TILE_SIZE * -2)
    );

    public static Map<Vector2d, JunctionHeight> junctionHeights = new Function<Integer, Map<Vector2d, JunctionHeight>>() {
        @Override
        public Map<Vector2d, JunctionHeight> apply(Integer ignore) {
            Map<Vector2d, JunctionHeight> ret = new HashMap<>();

            List<JunctionHeight> heights = Arrays.asList(JunctionHeight.HIGH, JunctionHeight.HIGH, JunctionHeight.HIGH, JunctionHeight.HIGH,
                    JunctionHeight.MEDIUM, JunctionHeight.MEDIUM, JunctionHeight.MEDIUM, JunctionHeight.MEDIUM,
                    JunctionHeight.LOW, JunctionHeight.LOW, JunctionHeight.LOW, JunctionHeight.LOW, JunctionHeight.LOW, JunctionHeight.LOW, JunctionHeight.LOW, JunctionHeight.LOW,
                    JunctionHeight.GROUND, JunctionHeight.GROUND, JunctionHeight.GROUND, JunctionHeight.GROUND, JunctionHeight.GROUND, JunctionHeight.GROUND, JunctionHeight.GROUND, JunctionHeight.GROUND, JunctionHeight.GROUND);

            int i = 0;
            for (Vector2d junction : JUNCTIONS) {
                ret.put(junction, heights.get(i));
                i++;
            }

            return ret;
        }
    }.apply(null);

    public static String VUFORIA_KEY = "AQfwG73/////AAABmaET3hUmm0WIjCN9wIx3AKA6l22iwwwVNCUbgJkn4v5KLzvswWwlRaShGcgpS2jgvjX+aBry9XKAoM0JeE1yFK1hpyDD3+mR68nn4uT/NoAKQvTDPC2a6+3rN91dN5qyCwg0UWv3oslFUIjQIX9HZBuRjVdHYfS1LU/Ea93hQ0wxulW3Hij8gdqRstJSYTi9u+IiGyYzv560wYoH5wZP2rJxbB3Av/E6O1C08lYAjKgRPMqsl27Wy1CA+lKzJ0pVYjRA3Z4+9AaQFFzFPjTKHPxXG75lzYXj0eB/aA8K91fokCK16SJp5xNJqoccpgO1t3IO7B1CVonzAEz9juq+WBsGPRffzMAxanmczBJjgh7Y";

    public enum SignalZone {
        A5_1,
        A5_2,
        A5_3,

        A2_1,
        A2_2,
        A2_3,

        F5_1,
        F5_2,
        F5_3,

        F2_1,
        F2_2,
        F2_3
    }

    public enum StartSpace {
        A5, // Blue Top
        A2, // Blue Bottom

        F5, // Red Top
        F2  // Red Bottom
    }

    public enum Alliance {
        NONE,
        BLUE,
        RED
    }


    // Does nothing - put a breakpoint on it
    public static void nop() {
    }

    public static boolean contains(int[] array, int value) {
        for (int i : array) {
            if (i == value) return true;
        }
        return false;
    }

    public static SignalZone[] getAvailableSignalZones(StartSpace space) {
        switch (space) {
            case A5:
                return new SignalZone[]{SignalZone.A5_1, SignalZone.A5_2, SignalZone.A5_3};
            case A2:
                return new SignalZone[]{SignalZone.A2_1, SignalZone.A2_2, SignalZone.A2_3};
            case F5:
                return new SignalZone[]{SignalZone.F5_1, SignalZone.F5_2, SignalZone.F5_3};
            case F2:
                return new SignalZone[]{SignalZone.F2_1, SignalZone.F2_2, SignalZone.F2_3};
        }

        return null;
    }

    public static Alliance tileToAlliance(String tile) {
        String col = tile.substring(1);

        if (col.equals("A") || col.equals("B") || col.equals("C")) {
            return Alliance.BLUE;
        } else if (col.equals("D") || col.equals("E") || col.equals("F")) {
            return Alliance.RED;
        } else {
            return Alliance.NONE;
        }
    }

    public static Pose2d stringToStartPose(String start) {
        if (start.equals("RedLeft")) {
            return START_POSITIONS.get(0);
        } else if (start.equals("RedRight")) {
            return START_POSITIONS.get(1);
        } else if (start.equals("BlueLeft")) {
            return START_POSITIONS.get(2);
        } else /* if (start.equals("BlueRight")) */ {
            return START_POSITIONS.get(3);
        }
    }

    // Our default telemetry is just ftc-dashboard bundled with the default instance
    public static Telemetry getDefaultTelemetry(Telemetry current) {
        return new MultipleTelemetry(current, FtcDashboard.getInstance().getTelemetry());
    }

    public static SignalZone barcodeToSignalZone(String barcodeData, StartSpace startSpace) {
        try {
            if (barcodeData.length() == 0) {
                return null;
            } else if (barcodeData.length() == 1) {
                return Enum.valueOf(SignalZone.class, startSpace + "_" + barcodeData);
            } else {
                // Alliance check: 0->Blue 1->Red
                /*assert ((startSpace.toString().startsWith("A") && barcodeData.startsWith("0")) ||
                        (startSpace.toString().startsWith("F") && barcodeData.startsWith("1")));*/

                return Enum.valueOf(SignalZone.class, startSpace + "_" + barcodeData.substring(barcodeData.length() - 1));
            }
        } catch (Exception e) {
            return null;
        }
    }

    public static void sleep(int ms) {
        try {
            Thread.sleep(ms);
        } catch (Exception e) {
        }
    }


    public static boolean within(double val, double target, double range) {
        return Math.abs(val - target) < range;
    }

    public static boolean within(long val, long target, long range) {
        return Math.abs(val - target) < range;
    }





    // Android only

    public static AprilTagDetectorJNI.TagFamily TAG_FAMILY = AprilTagDetectorJNI.TagFamily.TAG_25h9;
    // Offset to get from our tag values to the signal values [1,2,3]
    public static int TAG_OFFSET = 20;

    public static OpenCvCameraRotation ROBOT_CAMERA_ORIENTATION = OpenCvCameraRotation.UPRIGHT;

    static public String ROBOTA_SERIAL = "5fdd41245958fd1f";
    static public String ROBOTB_SERIAL = "d70d70756e460988";

    static RobotA robotSingleton = null;

    public static Robot getRobot(HardwareMap map) {

        if (getSerialNumber().equals(ROBOTB_SERIAL)) {
            return new RobotB(map);
        }
        else {
            return new RobotA(map);
        }

        /*if (robotSingleton == null) {
            robotSingleton = new RobotA(map);
        }
        return robotSingleton;*/
    }

    public static Robot getRobot(HardwareMap map, Pose2d startingPose) {
        return getRobot(map);
        /*
        if (robotSingleton == null) {
            robotSingleton = new RobotA(map, startingPose);
        }
        else {
            robotSingleton.drive.setPoseEstimate(startingPose);
        }
        return robotSingleton;*/
    }

    public static Executor executor = Executors.newFixedThreadPool(8);

    @SuppressLint("HardwareIds")
    public static String getSerialNumber() {
        String serialNumber;

        try {
            @SuppressLint("PrivateApi") Class<?> c = Class.forName("android.os.SystemProperties");
            Method get = c.getMethod("get", String.class);

            serialNumber = (String) get.invoke(c, "gsm.sn1");
            assert serialNumber != null;
            if (serialNumber.equals(""))
                serialNumber = (String) get.invoke(c, "ril.serialnumber");
            assert serialNumber != null;
            if (serialNumber.equals(""))
                serialNumber = (String) get.invoke(c, "ro.serialno");
            assert serialNumber != null;
            if (serialNumber.equals(""))
                serialNumber = (String) get.invoke(c, "sys.serialnumber");
            assert serialNumber != null;
            if (serialNumber.equals(""))
                serialNumber = Build.SERIAL;

            // If none of the methods above worked
            if (serialNumber.equals(""))
                serialNumber = null;
        } catch (Exception e) {
            e.printStackTrace();
            serialNumber = null;
        }

        return serialNumber;
    }
}