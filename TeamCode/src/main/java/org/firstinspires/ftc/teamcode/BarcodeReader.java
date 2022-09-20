package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Config.barcodeToSignalZone;

import android.graphics.Bitmap;
import android.graphics.Rect;

import androidx.annotation.NonNull;

import com.google.android.gms.tasks.OnCompleteListener;
import com.google.android.gms.tasks.OnFailureListener;
import com.google.android.gms.tasks.OnSuccessListener;
import com.google.android.gms.tasks.Task;
import com.google.mlkit.vision.barcode.BarcodeScanner;
import com.google.mlkit.vision.barcode.BarcodeScannerOptions;
import com.google.mlkit.vision.barcode.BarcodeScanning;
import com.google.mlkit.vision.barcode.common.Barcode;
import com.google.mlkit.vision.common.InputImage;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.List;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.Semaphore;

public class BarcodeReader {

    BarcodeScanner scanner;
    Barcode detectedBarcode;
    ElapsedTime timer;
    double freq;
    long maxAttempts;
    long currentAttempt;
    BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue;
    Callback callback;
    Semaphore sem = new Semaphore(1, true);


    public BarcodeReader(BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue, Callback callback) {
        BarcodeScannerOptions options =
                new BarcodeScannerOptions.Builder()
                        .build();

        scanner = BarcodeScanning.getClient(options);

        this.frameQueue = frameQueue;
        this.freq = 0.01;
        this.maxAttempts = Long.MAX_VALUE;
        this.callback = callback;

        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    }

    public BarcodeReader(BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue, Callback callback,  @Barcode.BarcodeFormat int format, @NonNull @Barcode.BarcodeFormat int... moreFormats) {
        BarcodeScannerOptions options =
                new BarcodeScannerOptions.Builder()
                        .setBarcodeFormats(format, moreFormats)
                        .build();

        scanner = BarcodeScanning.getClient(options);

        this.frameQueue = frameQueue;
        this.freq = 0.01;
        this.maxAttempts = Long.MAX_VALUE;
        this.callback = callback;

        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    }
    public BarcodeReader(double frequency, long maxAttempts, BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue, Callback callback, @Barcode.BarcodeFormat int format, @NonNull @Barcode.BarcodeFormat int... moreFormats) {
        BarcodeScannerOptions options =
                new BarcodeScannerOptions.Builder()
                        .setBarcodeFormats(format, moreFormats)
                        .build();

        scanner = BarcodeScanning.getClient(options);

        this.frameQueue = frameQueue;
        this.freq = frequency;
        this.maxAttempts = maxAttempts;
        this.callback = callback;

        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    }

    public boolean tryToRead(BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue) throws Exception {
        return tryToRead(frameQueue, null);
    }

    public boolean tryToRead(BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue, Callback callback) throws Exception {
        VuforiaLocalizer.CloseableFrame frame = frameQueue.peek();
        //frameQueue.clear();

        if (frame == null)
        {
            return false;
        }

        Image rgb = null;
        long numImages = frame.getNumImages();
        for (int i = 0; i < numImages; i++) {
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.YUV) {
                rgb = frame.getImage(i);
                break;
            }
        }

        if (rgb == null)
        {
            return false;
        }

        InputImage image = InputImage.fromByteBuffer(rgb.getPixels(), rgb.getWidth(), rgb.getBufferHeight(), 0, InputImage.IMAGE_FORMAT_YV12);

        //first create a bitmap
        //Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        //then load the image into it
        //bm.copyPixelsFromBuffer(rgb.getPixels());
        //frame.close();

        Task<List<Barcode>> result = scanner.process(image)
                .addOnSuccessListener(new OnSuccessListener<List<Barcode>>() {
                    @Override
                    public void onSuccess(List<Barcode> barcodes) {
                        if (barcodes.size() > 0) {
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

                            callback.onSuccess(detectedBarcode);
                        }
                        else {
                            callback.onFailure(null);
                        }
                    }
                })
                .addOnFailureListener(new OnFailureListener() {
                    @Override
                    public void onFailure(@NonNull Exception e) {
                        callback.onFailure(e);
                    }
                })
                .addOnCompleteListener(new OnCompleteListener<List<Barcode>>() {
                    @Override
                    public void onComplete(@NonNull Task<List<Barcode>> task) {
                        //sem.release();
                    }
                });

        // Didn't crash, there must have been a frame to read
        return true;
    }

    public boolean hasResult() {
        return detectedBarcode != null;
    }

    public String getResult() {
        return detectedBarcode.getDisplayValue();
    }

    public void reset() {
        detectedBarcode = null;
    }



    // Call this in your loop - if you're reading this, Sans took out the queen
    public void tick() throws Exception {
        if (timer.seconds() > freq && currentAttempt < maxAttempts) {
            timer.reset();
            currentAttempt++;

            tryToRead(frameQueue, callback);
        }
    }

    public interface Callback {
        void onSuccess(Barcode b);
        default void onFailure(Exception e) {};
    }
}
