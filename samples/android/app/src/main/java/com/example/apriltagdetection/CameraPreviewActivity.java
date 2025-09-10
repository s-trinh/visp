package com.example.apriltagdetection;

import android.Manifest;
import android.content.pm.PackageManager;

import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import android.os.Bundle;

import android.graphics.PixelFormat;
import android.hardware.Camera;
import android.widget.FrameLayout;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import android.util.Log;


import org.visp.core.VpImagePoint;

import java.io.ByteArrayOutputStream;
import java.util.List;

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
    public ImageView resultImageView;
    static int w,h;
    static TextView resultInfo;
    static LineSurfaceView lineSurface;

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
            lineSurface = findViewById(R.id.surfaceView);
            lineSurface.setZOrderOnTop(true);
            lineSurface.getHolder().setFormat(PixelFormat.TRANSLUCENT);

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

    public static void updateResult(List<VpImagePoint> corners, int strokeWidth, String s) {
        int RED = -65536;
        int GREEN = -16711936;
        int YELLOW = -256;
        int BLUE = -16776961;

        lineSurface.clear();

//        lineSurface.drawLine((int) corners.get(0).get_u(), corners.get(0).get_v(), corners.get(1).get_u(), corners.get(1).get_v(), RED, strokeWidth);
//        lineSurface.drawLine((int) corners.get(0).get_u(), corners.get(0).get_v(), corners.get(3).get_u(), corners.get(3).get_v(), GREEN, strokeWidth);
//        lineSurface.drawLine((int) corners.get(1).get_u(), corners.get(1).get_v(), corners.get(2).get_u(), corners.get(2).get_v(), YELLOW, strokeWidth);
//        lineSurface.drawLine((int) corners.get(2).get_u(), corners.get(2).get_v(), corners.get(3).get_u(), corners.get(3).get_v(), BLUE, strokeWidth);

        lineSurface.drawLine((int) corners.get(0).get_v(), corners.get(0).get_u(), corners.get(1).get_v(), corners.get(1).get_u(), RED, strokeWidth);
        lineSurface.drawLine((int) corners.get(0).get_v(), corners.get(0).get_u(), corners.get(3).get_v(), corners.get(3).get_u(), GREEN, strokeWidth);
        lineSurface.drawLine((int) corners.get(1).get_v(), corners.get(1).get_u(), corners.get(2).get_v(), corners.get(2).get_u(), YELLOW, strokeWidth);
        lineSurface.drawLine((int) corners.get(2).get_v(), corners.get(2).get_u(), corners.get(3).get_v(), corners.get(3).get_u(), BLUE, strokeWidth);

        resultInfo.setText(s);
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
