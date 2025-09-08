package com.example.apriltagdetection;

import android.Manifest;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

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
    private static int CAMERA_ID = 0;

    private Camera mCamera;
    // public static ImageView resultImageView;
    static int w,h;
    static TextView resultInfo;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);


        Camera.CameraInfo cameraInfo = new Camera.CameraInfo();

        // ChatGPT
        // Value? --> https://stackoverflow.com/a/36653669 ?
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.CAMERA}, 100);
        }

        // Open an instance of the first camera and retrieve its info.
        mCamera = getCameraInstance(CAMERA_ID);
        Camera.getCameraInfo(CAMERA_ID, cameraInfo);

        if (mCamera == null) {
            // Camera is not available, display error message
            Toast.makeText(this, "Camera is not available.", Toast.LENGTH_SHORT).show();
            setContentView(R.layout.camera_unavailable);
        } else {
            setContentView(R.layout.activity_camera_preview);

            resultInfo = findViewById(R.id.resultTV);
            // resultImageView = findViewById(R.id.resultImage);

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

    public static void updateResult(byte[] Src, String s){
        // byte [] Bits = new byte[Src.length*4]; //That's where the RGBA array goes.
        // int i;
        // for(i=0;i<Src.length;i++){
        //    Bits[i*4] = Bits[i*4+1] = Bits[i*4+2] = Src[i]; //Invert the source bits
        //    Bits[i*4+3] = -1;//0xff, that's the alpha.
        // }

        // //Now put these nice RGBA pixels into a Bitmap object
        // Bitmap bm = Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888);
        // bm.copyPixelsFromBuffer(ByteBuffer.wrap(Bits));

        // resultImageView.setImageBitmap(bm);
        resultInfo.setText(s);
    }

//    public static void updateResult(String s){
//        resultInfo.setText(s);
//    }

    @Override
    public void onPause() {
        super.onPause();
        // Stop camera access
        releaseCamera();
    }

    /** A safe way to get an instance of the Camera object. */
    private Camera getCameraInstance(int cameraId) {


        Camera.CameraInfo cameraInfo = new Camera.CameraInfo();
        for (int i = 0; i < Camera.getNumberOfCameras(); i++) {
            Camera.getCameraInfo(i, cameraInfo);

            String facing = (cameraInfo.facing == Camera.CameraInfo.CAMERA_FACING_FRONT) ? "Front" : "Back";
            Log.d("CameraInfo", "Camera " + i + ": Facing = " + facing);
            Log.d("CameraInfo", "Camera " + i + ": Orientation = " + cameraInfo.orientation);
            if (cameraInfo.facing == Camera.CameraInfo.CAMERA_FACING_FRONT) {
                CAMERA_ID = i;
                Toast.makeText(this, "!!!!!!!!!!!!!.", Toast.LENGTH_SHORT).show();
                break;
            }
        }



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







//package com.example.apriltagdetection;
//
//import android.app.Activity;
//import android.graphics.SurfaceTexture;
//import android.hardware.camera2.*;
//import android.os.Bundle;
//import android.util.Log;
//import android.view.Surface;
//import android.view.SurfaceTexture;
//import android.view.TextureView;
//import android.widget.Toast;
//
//import java.util.Arrays;
//
//public class CameraPreviewActivity extends Activity {
//
//    private static final String TAG = "CameraPreviewActivity";
//    private TextureView textureView;
//    private CameraDevice cameraDevice;
//    private CameraCaptureSession cameraCaptureSession;
//    private CameraManager cameraManager;
//    private String cameraId;
//    private Surface previewSurface;
//
//    @Override
//    protected void onCreate(Bundle savedInstanceState) {
//        super.onCreate(savedInstanceState);
//        setContentView(R.layout.activity_camera_preview);
//
//        textureView = findViewById(R.id.textureView);
//
//        // Initialize the TextureView surface texture listener
//        textureView.setSurfaceTextureListener(surfaceTextureListener);
//
//        // Initialize the CameraManager
//        cameraManager = (CameraManager) getSystemService(CAMERA_SERVICE);
//
//        try {
//            // Get the camera ID (default camera)
//            cameraId = cameraManager.getCameraIdList()[0]; // For rear camera
//        } catch (CameraAccessException e) {
//            e.printStackTrace();
//        }
//    }
//
//    private TextureView.SurfaceTextureListener surfaceTextureListener = new TextureView.SurfaceTextureListener() {
//        @Override
//        public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
//            previewSurface = new Surface(surface);
//            try {
//                openCamera();
//            } catch (CameraAccessException e) {
//                e.printStackTrace();
//            }
//        }
//
//        @Override
//        public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {}
//
//        @Override
//        public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
//            return false;
//        }
//
//        @Override
//        public void onSurfaceTextureUpdated(SurfaceTexture surface) {}
//    };
//
//    private void openCamera() throws CameraAccessException {
//        // Request the camera to be opened
//        cameraManager.openCamera(cameraId, stateCallback, null);
//    }
//
//    private CameraDevice.StateCallback stateCallback = new CameraDevice.StateCallback() {
//        @Override
//        public void onOpened(CameraDevice camera) {
//            // Successfully opened camera
//            cameraDevice = camera;
//            try {
//                createCameraPreview();
//            } catch (CameraAccessException e) {
//                e.printStackTrace();
//            }
//        }
//
//        @Override
//        public void onDisconnected(CameraDevice camera) {
//            // Camera was disconnected
//            cameraDevice.close();
//        }
//
//        @Override
//        public void onError(CameraDevice camera, int error) {
//            // Error opening camera
//            cameraDevice.close();
//            cameraDevice = null;
//        }
//    };
//
//    private void createCameraPreview() throws CameraAccessException {
//        // Prepare capture request for the preview
//        CameraCaptureRequest.Builder captureRequestBuilder = cameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
//        captureRequestBuilder.addTarget(previewSurface);
//
//        // Create capture session for the preview
//        cameraDevice.createCaptureSession(
//                Arrays.asList(previewSurface),
//                new CameraCaptureSession.StateCallback() {
//                    @Override
//                    public void onConfigured(CameraCaptureSession session) {
//                        // If the camera is successfully configured, start the preview
//                        if (cameraDevice == null) return;
//
//                        cameraCaptureSession = session;
//
//                        try {
//                            // Start the preview
//                            cameraCaptureSession.setRepeatingRequest(captureRequestBuilder.build(), null, null);
//                        } catch (CameraAccessException e) {
//                            e.printStackTrace();
//                        }
//                    }
//
//                    @Override
//                    public void onConfigureFailed(CameraCaptureSession session) {
//                        // Failed to configure the camera
//                        Toast.makeText(CameraPreviewActivity.this, "Camera configuration failed", Toast.LENGTH_SHORT).show();
//                    }
//                },
//                null
//        );
//    }
//
//    @Override
//    protected void onPause() {
//        super.onPause();
//        if (cameraDevice != null) {
//            cameraDevice.close();
//        }
//    }
//}
