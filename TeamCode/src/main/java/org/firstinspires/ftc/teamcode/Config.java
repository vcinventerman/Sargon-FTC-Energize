package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@com.acmerobotics.dashboard.config.Config
public class Config {
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




    public static void nop() {}

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
    static Telemetry getDefaultTelemetry(Telemetry current) {
        return new MultipleTelemetry(current, FtcDashboard.getInstance().getTelemetry());
    }
}
