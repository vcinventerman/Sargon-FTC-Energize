package org.firstinspires.ftc.teamcode;

import android.graphics.Rect;

import androidx.annotation.NonNull;

import com.google.android.gms.tasks.OnFailureListener;
import com.google.android.gms.tasks.OnSuccessListener;
import com.google.android.gms.tasks.Task;
import com.google.mlkit.vision.barcode.BarcodeScanner;
import com.google.mlkit.vision.barcode.BarcodeScannerOptions;
import com.google.mlkit.vision.barcode.BarcodeScanning;
import com.google.mlkit.vision.barcode.common.Barcode;
import com.google.mlkit.vision.common.InputImage;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.List;
import java.util.concurrent.BlockingQueue;

public class BarcodeReader {

    BarcodeScanner scanner;
    Barcode detectedBarcode;


    public BarcodeReader(@Barcode.BarcodeFormat int format, @NonNull @Barcode.BarcodeFormat int... moreFormats) {
        BarcodeScannerOptions options =
                new BarcodeScannerOptions.Builder()
                        .setBarcodeFormats(format, moreFormats)
                        .build();

        scanner = BarcodeScanning.getClient(options);
    }

    public BarcodeReader() {
        scanner = BarcodeScanning.getClient();
    }

    public boolean tryToRead(BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue) throws Exception {
        return tryToRead(frameQueue, null);
    }

    public boolean tryToRead(BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue, Callback callback) throws Exception {
        VuforiaLocalizer.CloseableFrame frame = frameQueue.peek();

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

    public interface Callback {
        void onSuccess(Barcode b);
        default void onFailure(Exception e) {};
    }
}
