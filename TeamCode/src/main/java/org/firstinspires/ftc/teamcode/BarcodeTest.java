package org.firstinspires.ftc.teamcode;

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

@Supervised(name="BarcodeTest_?", group="Iterative Opmode", autonomous=false, variations = {"A5", "A2", "F5", "F2"})
public class BarcodeTest extends SupervisedOpMode {

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    BarcodeReader barcodeReader;
    SignalZone targetZone;
    Barcode currentBarcode;


    // Code that runs when the INIT button is pressed (mandatory)
    public void init()  {
        telemetry = Config.getDefaultTelemetry(telemetry);

        initVuforia();
        initTfod();

        barcodeReader = new BarcodeReader(Barcode.FORMAT_UPC_E);
    }

    public void start() throws Exception {
        tfod.activate();
    }

    // Code that runs repeatedly after the PLAY button is pressed (optional)
    public void loop() throws Exception {
        barcodeReader.tryToRead(vuforia.getFrameQueue(), new BarcodeReader.Callback() {
            @Override
            public void onSuccess(Barcode b) {
                if (currentBarcode == null || currentBarcode.getDisplayValue() != b.getDisplayValue()) {
                    currentBarcode = b;
                    telemetry.addData("CurrentBarcodeText", currentBarcode.getDisplayValue());
                    telemetry.update();
                }
            }
        });

        /*if (targetZone == null) {
            barcodeReader.tryToRead(vuforia.getFrameQueue());

            if (barcodeReader.hasResult()) {

            }
        }*/


    }

    // Code that runs when the OpMode is stopped (optional)
    public void stop() {
        vuforia = null;
        tfod = null;
    }

    // Code that runs after this OpMode is dynamically updated
    public void hotpatch() {

    }




    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = Config.VUFORIA_KEY;

        if (Device.isRevControlHub()) {
            // Use USB Webcam
            //parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        }
        else {
            // Use internal back camera
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        }

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        Vuforia.setFrameFormat(PIXEL_FORMAT.YUV, true);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        /*tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);*/
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}