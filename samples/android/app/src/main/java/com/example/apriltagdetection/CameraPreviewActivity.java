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
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Spinner;
import android.view.View;
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
    private Spinner spinner;
    private CameraPreview mPreview;

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

            spinner = findViewById(R.id.spinner);
            // Create an array of data (items to display in the Spinner)
            String[] items = {
                    "TAG_36h11", "TAG_25h9", "TAG_25h7", "TAG_16h5", "TAG_CIRCLE21h7",
                    "TAG_ARUCO_4x4_1000", "TAG_ARUCO_5x5_1000", "TAG_ARUCO_6x6_1000", "TAG_ARUCO_MIP_36h12"
            };

            // Create an ArrayAdapter to populate the Spinner with data
            ArrayAdapter<String> adapter = new ArrayAdapter<>(this, android.R.layout.simple_spinner_item, items);

            // Specify the layout to be used when the dropdown is displayed
            adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);

            // Set the adapter to the Spinner
            spinner.setAdapter(adapter);

            // Set default selection (e.g., select the second item: "Banana")
            spinner.setSelection(0);

            // Set the listener for item selection
            spinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
                // Handle item selection
                @Override
                public void onItemSelected(AdapterView<?> parentView, View selectedItemView, int position, long id) {
                    // Get the selected item
                    String selectedItem = parentView.getItemAtPosition(position).toString();

                    // Show a toast with the selected item
                    Toast.makeText(CameraPreviewActivity.this, "Selected: " + selectedItem, Toast.LENGTH_SHORT).show();

                    mPreview.setAprilTagMethod(position);
                }

                // Handle no item selected
                @Override
                public void onNothingSelected(AdapterView<?> parentView) {
                    // You can choose to do nothing here or show a default message
                    Toast.makeText(CameraPreviewActivity.this, "No item selected", Toast.LENGTH_SHORT).show();
                }
            });

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
            mPreview = new CameraPreview(this, mCamera, cameraInfo, displayRotation);
            FrameLayout preview = findViewById(R.id.camera_preview);
            preview.addView(mPreview);
        }
    }

    public static void updateResults(List<List<VpImagePoint>> cornersList, int strokeWidth, String s) {
        lineSurface.clear();

        for (List<VpImagePoint> corners : cornersList) {
            updateResult(corners, strokeWidth);
        }

        resultInfo.setText(s);
    }

    private static void updateResult(List<VpImagePoint> corners, int strokeWidth) {
        int RED = -65536;
        int GREEN = -16711936;
        int YELLOW = -256;
        int BLUE = -16776961;


//        lineSurface.drawLine((int) corners.get(0).get_u(), corners.get(0).get_v(), corners.get(1).get_u(), corners.get(1).get_v(), RED, strokeWidth);
//        lineSurface.drawLine((int) corners.get(0).get_u(), corners.get(0).get_v(), corners.get(3).get_u(), corners.get(3).get_v(), GREEN, strokeWidth);
//        lineSurface.drawLine((int) corners.get(1).get_u(), corners.get(1).get_v(), corners.get(2).get_u(), corners.get(2).get_v(), YELLOW, strokeWidth);
//        lineSurface.drawLine((int) corners.get(2).get_u(), corners.get(2).get_v(), corners.get(3).get_u(), corners.get(3).get_v(), BLUE, strokeWidth);


        int[] color_ = {RED, GREEN, YELLOW, BLUE};
        int[] strokeWidth_ = {strokeWidth, strokeWidth, strokeWidth, strokeWidth};

        if (!corners.isEmpty()) {
            if (false)
            {
                double[] startX_ = {corners.get(0).get_u(), corners.get(0).get_u(), corners.get(1).get_u(), corners.get(2).get_u()};
                double[] startY_ = {corners.get(0).get_v(), corners.get(0).get_v(), corners.get(1).get_v(), corners.get(2).get_v()};
                double[] stopX_ = {corners.get(1).get_u(), corners.get(3).get_u(), corners.get(2).get_u(), corners.get(3).get_u()};
                double[] stopY_ = {corners.get(1).get_v(), corners.get(3).get_v(), corners.get(2).get_v(), corners.get(3).get_v()};
                lineSurface.drawLine(startX_, startY_, stopX_, stopY_, color_, strokeWidth_);
            }

            if (true)
            {
                double[] startX_ = {corners.get(0).get_v(), corners.get(0).get_v(), corners.get(1).get_v(), corners.get(2).get_v()};
                double[] startY_ = {corners.get(0).get_u(), corners.get(0).get_u(), corners.get(1).get_u(), corners.get(2).get_u()};
                double[] stopX_ = {corners.get(1).get_v(), corners.get(3).get_v(), corners.get(2).get_v(), corners.get(3).get_v()};
                double[] stopY_ = {corners.get(1).get_u(), corners.get(3).get_u(), corners.get(2).get_u(), corners.get(3).get_u()};
                lineSurface.drawLine(startX_, startY_, stopX_, stopY_, color_, strokeWidth_);
            }
        }
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
