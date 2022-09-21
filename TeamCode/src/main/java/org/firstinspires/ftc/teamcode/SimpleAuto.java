package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Config.barcodeToSignalZone;

import com.google.mlkit.vision.barcode.common.Barcode;
import com.karrmedia.ftchotpatch.Supervised;
import com.karrmedia.ftchotpatch.SupervisedOpMode;
import com.qualcomm.robotcore.util.Device;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import org.firstinspires.ftc.teamcode.Config.SignalZone;

@Supervised(name="SimpleAuto_?", group="Iterative Opmode", autonomous=false, variations = {"A5", "A2", "F5", "F2"})
public class SimpleAuto extends SupervisedOpMode {


    BarcodeReader barcodeReader;
    SignalZone targetZone;
    Barcode currentBarcode;
    Config.StartSpace startSpace;


    // Code that runs when the INIT button is pressed (mandatory)
    public void init()  {

    }

    public void start() throws Exception {

    }

    // Code that runs repeatedly after the PLAY button is pressed (optional)
    public void loop() throws Exception {




    }

    // Code that runs when the OpMode is stopped (optional)
    public void stop() {

    }

    // Code that runs after this OpMode is dynamically updated
    public void hotpatch() {

    }



}