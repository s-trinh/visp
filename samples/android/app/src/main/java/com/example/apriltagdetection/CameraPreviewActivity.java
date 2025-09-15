package com.example.apriltagdetection;

import android.Manifest;
import android.content.pm.PackageManager;

import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import android.graphics.Color;
import android.os.Bundle;

import android.graphics.PixelFormat;
import android.hardware.Camera;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;
import android.view.View;

import org.visp.core.VpImagePoint;

import java.util.ArrayList;
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
 * https://developer.android.com/media/camera/camera-deprecated/camera-api
 */
public class CameraPreviewActivity extends MainActivity  {
    /**
     * Id of the camera to access. 0 is the first camera.
     */
    private static final int CAMERA_ID = 0;

    private Camera mCamera;
    private int mW, mH;
    static TextView mResultInfo;
    static LineSurfaceView mLineSurface;
    private Spinner mSpinner;
    private CameraPreview mPreview;
    private Button mBtnAutoFocus;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        // ChatGPT
        // Arbitrary value? --> https://stackoverflow.com/a/36653669 ?
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

            mBtnAutoFocus = findViewById(R.id.btnAutoFocus);
            // Set up the autofocus button
            mBtnAutoFocus.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    if (mCamera != null) {
                        mCamera.autoFocus(new Camera.AutoFocusCallback() {
                            @Override
                            public void onAutoFocus(boolean success, Camera camera) {
                                if (!success) {
                                    Toast.makeText(CameraPreviewActivity.this, "Cannot perform camera autofocus",
                                            Toast.LENGTH_SHORT).show();
                                }
                            }
                        });
                    }
                }
            });

            mSpinner = findViewById(R.id.spinner);
            String[] items = {
                    "TAG_36h11", "TAG_25h9", "TAG_25h7", "TAG_16h5", "TAG_CIRCLE21h7",
                    "TAG_ARUCO_4x4_1000", "TAG_ARUCO_5x5_1000", "TAG_ARUCO_6x6_1000", "TAG_ARUCO_MIP_36h12"
            };

            // Create an ArrayAdapter to populate the Spinner with data
            ArrayAdapter<String> adapter = new ArrayAdapter<>(this, android.R.layout.simple_spinner_item, items);
            adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
            mSpinner.setAdapter(adapter);
            mSpinner.setSelection(0);

            // Set the listener for item selection
            mSpinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
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
                    Toast.makeText(CameraPreviewActivity.this, "No item selected", Toast.LENGTH_SHORT).show();
                }
            });

            mResultInfo = findViewById(R.id.resultTV);
            mLineSurface = findViewById(R.id.surfaceView);
            mLineSurface.setZOrderOnTop(true);
            mLineSurface.getHolder().setFormat(PixelFormat.TRANSLUCENT);

            // init the byte array
            mW = mCamera.getParameters().getPreviewSize().width;
            mH = mCamera.getParameters().getPreviewSize().height;

            // Get the rotation of the screen to adjust the preview image accordingly.
            final int displayRotation = getWindowManager().getDefaultDisplay().getRotation();

            // Create the Preview view and set it as the content of this Activity.
            mPreview = new CameraPreview(this, mCamera, cameraInfo, displayRotation);
            FrameLayout preview = findViewById(R.id.camera_preview);
            preview.addView(mPreview);
        }
    }

    public static void updateResults(List<List<VpImagePoint>> cornersList, int strokeWidth, int[] ids, String s) {
        int RED = Color.RED; // -65536
        int GREEN = Color.GREEN; // -16711936
        int YELLOW = Color.YELLOW; // -256
        int BLUE = Color.BLUE; // -16776961

        int[] color_ = {RED, GREEN, YELLOW, BLUE};
        int[] strokeWidth_ = {strokeWidth, strokeWidth, strokeWidth, strokeWidth};

        List<double[]> list_startX = new ArrayList<>(cornersList.size());
        List<double[]> list_startY = new ArrayList<>(cornersList.size());
        List<double[]> list_stopX = new ArrayList<>(cornersList.size());
        List<double[]> list_stopY = new ArrayList<>(cornersList.size());

        mLineSurface.clear();

        List<Double> centerX = new ArrayList<>(cornersList.size());
        List<Double> centerY = new ArrayList<>(cornersList.size());
        for (List<VpImagePoint> corners : cornersList) {
            double[] startX_ = {corners.get(0).get_v(), corners.get(0).get_v(), corners.get(1).get_v(), corners.get(2).get_v()};
            double[] startY_ = {corners.get(0).get_u(), corners.get(0).get_u(), corners.get(1).get_u(), corners.get(2).get_u()};
            double[] stopX_ = {corners.get(1).get_v(), corners.get(3).get_v(), corners.get(2).get_v(), corners.get(3).get_v()};
            double[] stopY_ = {corners.get(1).get_u(), corners.get(3).get_u(), corners.get(2).get_u(), corners.get(3).get_u()};

            list_startX.add(startX_);
            list_startY.add(startY_);
            list_stopX.add(stopX_);
            list_stopY.add(stopY_);

            centerX.add( (corners.get(0).get_v() + corners.get(1).get_v() + corners.get(2).get_v() + corners.get(3).get_v()) / 4 );
            centerY.add( (corners.get(0).get_u() + corners.get(1).get_u() + corners.get(2).get_u() + corners.get(3).get_u()) / 4 );
        }

        mLineSurface.drawLines(list_startX, list_startY, list_stopX, list_stopY, color_, strokeWidth_, centerX, centerY, ids);

        mResultInfo.setText(s);
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
