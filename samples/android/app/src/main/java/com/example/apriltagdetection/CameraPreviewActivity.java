package com.example.apriltagdetection;

import android.Manifest;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.graphics.YuvImage;
import android.graphics.drawable.BitmapDrawable;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import android.os.Bundle;

import android.hardware.Camera;
import android.view.View;
import android.widget.FrameLayout;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;
import android.util.Log;

import com.google.android.material.snackbar.Snackbar;

import java.io.ByteArrayOutputStream;
import java.nio.ByteBuffer;

/**
 * Displays a {@link CameraPreview} of the first {@link Camera}.
 * An error message is displayed if the Camera is not available.
 * <p>
 * This Activity is only used to illustrate that access to the Camera API has been granted (or
 * denied) as part of the runtime permissions model. It is not relevant for the use of the
 * permissions API.
 * <p>
 * Implementation is based directly on the documentation at
 * http://developer.android.com/guide/topics/media/camera.html
 */
public class CameraPreviewActivity extends MainActivity  {

    /**
     * Id of the camera to access. 0 is the first camera.
     */
    private static final int CAMERA_ID = 0;

    private Camera mCamera;
    public static ImageView resultImageView;
    static int w,h;
    static TextView resultInfo;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        // ChatGPT
        // Value? --> https://stackoverflow.com/a/36653669 ?
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.CAMERA}, 100);
        }

        // Open an instance of the first camera and retrieve its info.
        Camera.CameraInfo cameraInfo = new Camera.CameraInfo();
        mCamera = getCameraInstance(CAMERA_ID);
        Camera.getCameraInfo(CAMERA_ID, cameraInfo);

        if (mCamera == null) {
            // Camera is not available, display error message
            Toast.makeText(this, "Camera is not available.", Toast.LENGTH_SHORT).show();
            setContentView(R.layout.camera_unavailable);
        } else {
            setContentView(R.layout.activity_camera_preview);

            resultInfo = findViewById(R.id.resultTV);
//            resultImageView = findViewById(R.id.imageView);

            // init the byte array
            w = mCamera.getParameters().getPreviewSize().width;
            h = mCamera.getParameters().getPreviewSize().height;

            // Get the rotation of the screen to adjust the preview image accordingly.
            final int displayRotation = getWindowManager().getDefaultDisplay()
                    .getRotation();

            // Create the Preview view and set it as the content of this Activity.
            CameraPreview mPreview = new CameraPreview(this, mCamera, cameraInfo, displayRotation);
            FrameLayout preview = findViewById(R.id.camera_preview);
            preview.addView(mPreview);
        }
    }

    public static void updateResult(byte[] Src, int w_, int h_, String s){
        // NOK

//        byte [] Bits = new byte[Src.length*4]; //That's where the RGBA array goes.
//
//        for (int i = 0; i < Src.length; i++){
//            Bits[i*4] = Bits[i*4+1] = Bits[i*4+2] = Src[i]; //Invert the source bits
//            Bits[i*4+3] = -1;//0xff, that's the alpha.
//        }
//
//        //Now put these nice RGBA pixels into a Bitmap object
//        Bitmap bitmap = Bitmap.createBitmap(w_, h_, Bitmap.Config.ARGB_8888);
//        bitmap.copyPixelsFromBuffer(ByteBuffer.wrap(Bits));
//
//        Bitmap modifiedBitmap = drawRedLineOnBitmap(bitmap, 10);
//        resultImageView.setImageBitmap(modifiedBitmap);
//
//
////        Bitmap bitmap = getImageBitmap(resultImageView);
////        if (bitmap != null) {
////            // You can now use the bitmap (e.g., save it, modify it, etc.)
////            Bitmap modifiedBitmap = drawRedLineOnBitmap(bitmap, 10);
////
////            resultImageView.setImageBitmap(modifiedBitmap);
////        }


//        if (resultImageView == null) {
//            Log.e("CameraPreviewActivity", "resultImageView is null");
//        }
//
//        // Convert the byte[] preview frame to a Bitmap
//        Bitmap bitmap = convertNV21ToBitmap(Src, w_, h_, ImageFormat.NV21);
//        // Check if the bitmap is mutable
//        if (!bitmap.isMutable()) {
//            bitmap = bitmap.copy(Bitmap.Config.ARGB_8888, true); // Create a mutable copy of the bitmap
//        }
//
//        if (bitmap != null) {
//            Log.e("CameraPreviewActivity", "Modify the Bitmap");
//            // Modify the Bitmap here (e.g., apply filters or transformations)
//
//            // Create a canvas to draw on the bitmap
//            Canvas canvas = new Canvas(bitmap);
//            canvas.drawColor(0);
//
//            // Set up the paint for drawing the red line
//            Paint paint = new Paint();
//            paint.setColor(Color.GREEN); // Red color
//            int lineWidth = 10;
//            paint.setStrokeWidth(lineWidth); // Set the width of the line
//            paint.setAntiAlias(true); // Smooth out the edges of the line
//
//            // Draw a red line on the canvas (example: from (50, 50) to (500, 500))
//            canvas.drawLine(50, 50, 100, 50, paint);
//
//            // Update the ImageView with the modified Bitmap
//            resultImageView.setImageBitmap(bitmap);
//        }

        resultInfo.setText(s);
    }

    // Function to get Bitmap from ImageView
    private static Bitmap getImageBitmap(ImageView imageView) {
        // Check if the drawable is an instance of BitmapDrawable
        if (imageView.getDrawable() instanceof BitmapDrawable) {
            BitmapDrawable drawable = (BitmapDrawable) imageView.getDrawable();
            return drawable.getBitmap(); // Return the bitmap
        }
        return null; // Return null if the drawable is not a BitmapDrawable
    }

    // Function to draw a red line with a specific width on the image
    private static Bitmap drawRedLineOnBitmap(Bitmap originalBitmap, int lineWidth) {
        // Create a mutable copy of the original bitmap
        Bitmap mutableBitmap = originalBitmap.copy(Bitmap.Config.ARGB_8888, true);

        // Create a canvas to draw on the bitmap
        Canvas canvas = new Canvas(mutableBitmap);

        // Set up the paint for drawing the red line
        Paint paint = new Paint();
        paint.setColor(Color.RED); // Red color
        paint.setStrokeWidth(lineWidth); // Set the width of the line
        paint.setAntiAlias(true); // Smooth out the edges of the line

        // Draw a red line on the canvas (example: from (50, 50) to (500, 500))
        canvas.drawLine(50, 50, 500, 500, paint);

        // Return the modified bitmap
        return mutableBitmap;
    }

    // Convert raw camera frame (NV21) to Bitmap
    private static Bitmap convertNV21ToBitmap(byte[] data, int previewWidth, int previewHeight, int previewFormat) {
//        Camera.Parameters parameters = camera.getParameters();
//        int previewWidth = parameters.getPreviewSize().width;
//        int previewHeight = parameters.getPreviewSize().height;

        // Create a YuvImage from the NV21 byte array
//        YuvImage yuvImage = new YuvImage(data, parameters.getPreviewFormat(), previewWidth, previewHeight, null);
        YuvImage yuvImage = new YuvImage(data, previewFormat, previewWidth, previewHeight, null);

        // Compress the YUV image to a JPEG output stream
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        yuvImage.compressToJpeg(new android.graphics.Rect(0, 0, previewWidth, previewHeight), 100, out);

        // Get the byte array from the JPEG output stream
        byte[] byteArray = out.toByteArray();

        // Decode the byte array into a Bitmap
        return BitmapFactory.decodeByteArray(byteArray, 0, byteArray.length);
    }

    @Override
    public void onPause() {
        super.onPause();
        // Stop camera access
        releaseCamera();
    }

    /** A safe way to get an instance of the Camera object. */
    private Camera getCameraInstance(int cameraId) {
        Camera c = null;
        try {
            c = Camera.open(cameraId); // attempt to get a Camera instance
        } catch (Exception e) {
            // Camera is not available (in use or does not exist)
            Toast.makeText(this, "Camera " + cameraId + " is not available: " + e.getMessage(),
                    Toast.LENGTH_SHORT).show();
        }
        return c; // returns null if camera is unavailable
    }

    private void releaseCamera() {
        if (mCamera != null) {
            mCamera.release();        // release the camera for other applications
            mCamera = null;
        }
    }
}
