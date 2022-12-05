package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.yield;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.jetbrains.annotations.Contract;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Map;

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
    public static double ROBOTA_LENGTH = 18;
    public static double ROBOTA_WIDTH = 17;

    // Choose current robot here
    public static double ROBOT_LENGTH = ROBOTA_LENGTH;
    public static double ROBOT_WIDTH = ROBOTA_WIDTH;

    public static Pose2d START_POS_RED_LEFT = new Pose2d(-35, -(FIELD_WIDTH / 2) + (ROBOT_LENGTH / 2), FIELD_BEARING_NORTH);
    public static Pose2d START_POS_RED_RIGHT = new Pose2d(35, -(FIELD_WIDTH / 2) + (ROBOT_LENGTH / 2), FIELD_BEARING_NORTH);
    public static Pose2d START_POS_BLUE_LEFT = new Pose2d(35, (FIELD_WIDTH / 2) - (ROBOT_LENGTH / 2), FIELD_BEARING_SOUTH);
    public static Pose2d START_POS_BLUE_RIGHT = new Pose2d(-35, (FIELD_WIDTH / 2) - (ROBOT_LENGTH / 2), FIELD_BEARING_SOUTH);





    public enum JunctionHeight {
        GROUND,
        LOW,
        MEDIUM,
        HIGH
    }

    public static Map<Vector2d, JunctionHeight> junctions;

    public static AprilTagDetectorJNI.TagFamily TAG_FAMILY = AprilTagDetectorJNI.TagFamily.TAG_25h9;
    // Offset to get from our tag values to the signal values [1,2,3]
    public static int TAG_OFFSET = 20;

    public static OpenCvCameraRotation ROBOT_CAMERA_ORIENTATION = OpenCvCameraRotation.UPSIDE_DOWN;

    public static String VUFORIA_KEY = "AQfwG73/////AAABmaET3hUmm0WIjCN9wIx3AKA6l22iwwwVNCUbgJkn4v5KLzvswWwlRaShGcgpS2jgvjX+aBry9XKAoM0JeE1yFK1hpyDD3+mR68nn4uT/NoAKQvTDPC2a6+3rN91dN5qyCwg0UWv3oslFUIjQIX9HZBuRjVdHYfS1LU/Ea93hQ0wxulW3Hij8gdqRstJSYTi9u+IiGyYzv560wYoH5wZP2rJxbB3Av/E6O1C08lYAjKgRPMqsl27Wy1CA+lKzJ0pVYjRA3Z4+9AaQFFzFPjTKHPxXG75lzYXj0eB/aA8K91fokCK16SJp5xNJqoccpgO1t3IO7B1CVonzAEz9juq+WBsGPRffzMAxanmczBJjgh7Y";;

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
    public static void nop() {}

    public static boolean contains(int[] array, int value) {
        for (int i : array) {
            if (i == value) return true;
        }
        return false;
    }

    public static SignalZone[] getAvailableSignalZones(StartSpace space) {
        switch (space)
        {
            case A5: return new SignalZone[]{SignalZone.A5_1, SignalZone.A5_2, SignalZone.A5_3};
            case A2: return new SignalZone[]{SignalZone.A2_1, SignalZone.A2_2, SignalZone.A2_3};
            case F5: return new SignalZone[]{SignalZone.F5_1, SignalZone.F5_2, SignalZone.F5_3};
            case F2: return new SignalZone[]{SignalZone.F2_1, SignalZone.F2_2, SignalZone.F2_3};
        }

        return null;
    }

    public static Alliance tileToAlliance(String tile)
    {
        String col = tile.substring(1);

        if (col.equals("A") || col.equals("B") || col.equals("C")) { return Alliance.BLUE; }
        else if (col.equals("D") || col.equals("E") || col.equals("F")) { return Alliance.RED; }
        else { return Alliance.NONE; }
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
        }
        catch (Exception e) { return null; }
    }

    public static void sleep(int ms){
        try {
            Thread.sleep(ms);
        } catch (Exception e){}
    }
}
