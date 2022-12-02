/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TeamConf.nop;

import android.graphics.Bitmap;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.LuminanceSource;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.Result;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.oned.UPCEReader;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.lang.reflect.Field;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * In this sample, we demonstrate how to use EasyOpenCV in
 * Vuforia passthrough mode. In this mode, EasyOpenCV does not
 * take direct control over the camera. Instead, it pulls frames
 * out of a VuforiaLocalizer's frame queue. This allows you to
 * run both OpenCV and Vuforia simultaneously on the same camera.
 * The downside is that you do not get to choose the resolution
 * of frames delivered to your pipeline, and you do not get any
 * sort of manual control over sensor parameters such as exposure,
 * gain, ISO, or frame rate.
 */
@TeleOp
@Disabled
public class BarcodeTest extends LinearOpMode
{
    VuforiaLocalizer vuforia = null;
    OpenCvCamera vuforiaPassthroughCam;

    ElapsedTime timer = new ElapsedTime();
    double freq = 1;
    long maxAttempts = Long.MAX_VALUE;
    long currentAttempt = 0;
    AtomicBoolean isFrameStored = new AtomicBoolean(false);
    AtomicBoolean isFrameProcessing = new AtomicBoolean(false);
    Executor executor = Executors.newCachedThreadPool();
    //Bitmap frame;
    Frame frame;
    /*Barcode detectedBarcode;
    InputImage image;
    BarcodeScanner scanner;*/

    enum FrameState {
        WAIT,
    }

    @Override
    public void runOpMode()
    {
        try { Field telemetryAccessor = this.getClass().getDeclaredField("telemetry"); telemetryAccessor.setAccessible(true); telemetryAccessor.set(this, TeamConf.getDefaultTelemetry(telemetry)); } catch (Exception e) { RobotLog.e("ROBOT REFLECT: " + e.toString()); }
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCV,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);

        /*
         * Setup Vuforia
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(viewportContainerIds[0]);
        parameters.vuforiaLicenseKey = TeamConf.VUFORIA_KEY;
        //parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        // Uncomment this line below to use a webcam
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforia.enableConvertFrameToBitmap();

        // Create a Vuforia passthrough "virtual camera"
        vuforiaPassthroughCam = OpenCvCameraFactory.getInstance().createVuforiaPassthrough(vuforia, parameters, viewportContainerIds[1]);

        vuforiaPassthroughCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                // Using GPU acceleration can be particularly helpful when using Vuforia passthrough
                // mode, because Vuforia often chooses high resolutions (such as 720p) which can be
                // very CPU-taxing to rotate in software. GPU acceleration has been observed to cause
                // issues on some devices, though, so if you experience issues you may wish to disable it.
                vuforiaPassthroughCam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
                vuforiaPassthroughCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
                vuforiaPassthroughCam.setPipeline(new UselessColorBoxDrawingPipeline(new Scalar(255,0,0,255)));

                // We don't get to choose resolution, unfortunately. The width and height parameters
                // are entirely ignored when using Vuforia passthrough mode. However, they are left
                // in the method signature to provide interface compatibility with the other types
                // of cameras.
                vuforiaPassthroughCam.startStreaming(0,0, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
                nop();
            }
        });

        /*BarcodeScannerOptions options =
                new BarcodeScannerOptions.Builder()
                        .setBarcodeFormats(Barcode.FORMAT_UPC_E)
                        .build();

        scanner = BarcodeScanning.getClient(options);*/

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Passthrough FPS", vuforiaPassthroughCam.getFps());
            telemetry.addData("Frame count", vuforiaPassthroughCam.getFrameCount());
            //if (detectedBarcode != null && detectedBarcode.getDisplayValue() != null) { telemetry.addData("Barcode", detectedBarcode.getDisplayValue()); }
            telemetry.update();



            if (timer.seconds() > freq && currentAttempt < maxAttempts && !isFrameProcessing.get()) {
                isFrameProcessing.set(true);
                timer.reset();
                currentAttempt++;

                vuforia.getFrameOnce(Continuation.create(ThreadPool.getDefault(), new Consumer<Frame>()
                {
                    @Override public void accept(Frame frame)
                    {
                        Bitmap bitmapOrig = vuforia.convertFrameToBitmap(frame);
                        Bitmap bitmap = bitmapOrig.copy(bitmapOrig.getConfig(), false);
                        if (bitmap != null) {

                            int[] intArray = new int[bitmap.getWidth()*bitmap.getHeight()];
                            //copy pixel data from the Bitmap into the 'intArray' array
                            bitmap.getPixels(intArray, 0, bitmap.getWidth(), 0, 0, bitmap.getWidth(), bitmap.getHeight());

                            LuminanceSource source = new RGBLuminanceSource(bitmap.getWidth(), bitmap.getHeight(), intArray);
                            BinaryBitmap bitmapBinary = new BinaryBitmap(new HybridBinarizer(source));

                            UPCEReader reader = new UPCEReader();

                            try {
                                Result res = reader.decode(bitmapBinary);
                                nop();
                            }
                            catch (Exception e) {
                                nop();
                            }

                            //Image img = frame.getImage(0);
                            //InputImage input = InputImage.fromByteBuffer(img.getPixels(), img.getWidth(), img.getHeight(), 0, img.getFormat());
                            //image = InputImage.fromBitmap(bitmap, 0);

                            //Bitmap test = image.getBitmapInternal();

                            /*scanner.process(bitmapOrig, 0)
                            //Task<List<Barcode>> result = scanner.process(image)
                                    .addOnSuccessListener(new OnSuccessListener<List<Barcode>>() {
                                        @Override
                                        public void onSuccess(List<Barcode> barcodes) {
                                            if (barcodes.size() > 0 && detectedBarcode.getBoundingBox() != null) {
                                                detectedBarcode = barcodes.get(0);
                                                int detectedBarcodeSize = detectedBarcode.getBoundingBox().width() * detectedBarcode.getBoundingBox().height();

                                                for (Barcode i : barcodes) {
                                                    Rect box = i.getBoundingBox();
                                                    if (box != null) {
                                                        int size = box.width() * box.height();

                                                        if (size > detectedBarcodeSize) {
                                                            detectedBarcode = i;
                                                            detectedBarcodeSize = size;
                                                        }
                                                    }
                                                }

                                                //callback.onSuccess(detectedBarcode);
                                            } else {
                                                //callback.onFailure(null);
                                            }
                                        }
                                    })
                                    .addOnFailureListener(new OnFailureListener() {
                                        @Override
                                        public void onFailure(@NonNull Exception e) {
                                            //callback.onFailure(e);
                                        }
                                    })
                                    .addOnCompleteListener(new OnCompleteListener<List<Barcode>>() {
                                        @Override
                                        public void onComplete(@NonNull Task<List<Barcode>> task) {
                                            //frame.recycle();

                                            isFrameProcessing.set(false);
                                        }
                                    });*/

                        }
                    }
                }));

                /*
                vuforia.getFrameOnce(Continuation.create(executor, new Consumer<Frame>() {
                    @Override
                    public void accept(Frame value) {
                        //frame = vuforia.convertFrameToBitmap(value);
                        frame = value;

                        isFrameStored.set(true);
                        isFrameProcessing.set(false);
                    }
                }));*/

                /*vuforia.getFrameOnce(Continuation.create(executor, new Consumer<Frame>() {
                    @Override
                    public void accept(Frame value) {
                        continuation.dispatch(new ContinuationResult<Consumer<Bitmap>>() {
                            @Override
                            public void handle(Consumer<Bitmap> consumer) {
                                consumer.accept(vuforia.convertFrameToBitmap(frame));
                            }
                        });
                    }
                }));*/

                /*vuforiaPassthroughCam.getFrameBitmap(Continuation.create(executor, new Consumer<Bitmap>() {
                    @Override
                    public void accept(Bitmap value) {
                        frame = value;
                        isFrameStored.set(true);
                        isFrameProcessing.set(false);
                    }
                }));*/
            }

            /*if (isFrameStored.get() && !isFrameProcessing.get()) {
                if (frame == null || frame.getNumImages() < 1) {
                    isFrameStored.set(false);
                    isFrameProcessing.set(false);
                } else {

                    isFrameProcessing.set(true);

                    Image img = frame.getImage(0);
                    //InputImage input = InputImage.fromByteBuffer(img.getPixels(), img.getWidth(), img.getHeight(), 0, img.getFormat());
                    //InputImage image = InputImage.fromBitmap(frame, 0);

                    //Bitmap test = input.getBitmapInternal();

                    Task<List<Barcode>> result = scanner.process(input)
                            .addOnSuccessListener(new OnSuccessListener<List<Barcode>>() {
                                @Override
                                public void onSuccess(List<Barcode> barcodes) {
                                    if (barcodes.size() > 0 && detectedBarcode.getBoundingBox() != null) {
                                        detectedBarcode = barcodes.get(0);
                                        int detectedBarcodeSize = detectedBarcode.getBoundingBox().width() * detectedBarcode.getBoundingBox().height();

                                        for (Barcode i : barcodes) {
                                            Rect box = i.getBoundingBox();
                                            if (box != null) {
                                                int size = box.width() * box.height();

                                                if (size > detectedBarcodeSize) {
                                                    detectedBarcode = i;
                                                    detectedBarcodeSize = size;
                                                }
                                            }
                                        }

                                        //callback.onSuccess(detectedBarcode);
                                    } else {
                                        //callback.onFailure(null);
                                    }
                                }
                            })
                            .addOnFailureListener(new OnFailureListener() {
                                @Override
                                public void onFailure(@NonNull Exception e) {
                                    //callback.onFailure(e);
                                }
                            })
                            .addOnCompleteListener(new OnCompleteListener<List<Barcode>>() {
                                @Override
                                public void onComplete(@NonNull Task<List<Barcode>> task) {
                                    //frame.recycle();

                                    isFrameProcessing.set(false);
                                    isFrameStored.set(false);
                                }
                            });
                }
            }*/




            idle();
        }
    }

    class UselessColorBoxDrawingPipeline extends OpenCvPipeline
    {
        Scalar color;

        UselessColorBoxDrawingPipeline(Scalar color)
        {
            this.color = color;
        }

        @Override
        public Mat processFrame(Mat input)
        {
            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols()/4.0,
                            input.rows()/4.0),
                    new Point(
                            input.cols()*(3f/4f),
                            input.rows()*(3f/4f)),
                    color, 4);

            return input;
        }
    }
}