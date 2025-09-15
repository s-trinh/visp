package com.example.apriltagdetection;

import android.content.Context;

import android.hardware.Camera;
import android.util.Log;
import android.view.Surface;
import android.view.SurfaceHolder;
import android.view.SurfaceView;

import org.visp.core.VpCameraParameters;
import org.visp.core.VpHomogeneousMatrix;
import org.visp.core.VpPoint;
import org.visp.core.VpImagePoint;
import org.visp.core.VpImageUChar;
import org.visp.detection.VpDetectorAprilTag;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static com.example.apriltagdetection.CameraPreviewActivity.updateResults;

/**
 * Camera preview that displays a {@link Camera}.
 *
 * Handles basic lifecycle methods to display and stop the preview.
 * <p>
 * Implementation is based directly on the documentation at
 * http://developer.android.com/guide/topics/media/camera.html
 */
public class CameraPreview extends SurfaceView implements SurfaceHolder.Callback, Camera.PreviewCallback {
    private static final String TAG = "CameraPreview";
    private SurfaceHolder mHolder;
    private Camera mCamera;
    private Camera.CameraInfo mCameraInfo;
    private int mDisplayOrientation;
    private long mLastTime;
    private int mW, mH;
    private VpCameraParameters mCameraParameters;
    private double mTagSize;
    private VpDetectorAprilTag mDetectorAprilTag;
    private VpImageUChar mImageUChar;
    private static int[] mMethods = {
        0, // TAG_36h11
        3, // TAG_25h9
        4, // TAG_25h7
        5, // TAG_16h5
        6, // TAG_CIRCLE21h7
        14, // TAG_ARUCO_4x4_1000
        18, // TAG_ARUCO_5x5_1000
        22, // TAG_ARUCO_6x6_1000
        23, // TAG_ARUCO_MIP_36h12
    };
    private int mMethod;
    private static float[] mDetectionMarginThresholds = {
            -1, // TAG_36h11
            -1, // TAG_25h9
            -1, // TAG_25h7
            100, // TAG_16h5
            -1, // TAG_CIRCLE21h7
            100, // TAG_ARUCO_4x4_1000
            100, // TAG_ARUCO_5x5_1000
            100, // TAG_ARUCO_6x6_1000
            -1, // TAG_ARUCO_MIP_36h12
    };

    public CameraPreview(Context context, Camera camera, Camera.CameraInfo cameraInfo,
                         int displayOrientation) {
        super(context);

        // Do not initialize if no camera has been set
        if (camera == null || cameraInfo == null) {
            return;
        }

        mCamera = camera;
        mCameraInfo = cameraInfo;
        mDisplayOrientation = displayOrientation;

        // Install a SurfaceHolder.Callback so we get notified when the
        // underlying surface is created and destroyed.
        mHolder = getHolder();
        mHolder.addCallback(this);

        // init the ViSP tag detection system
        mW = mCamera.getParameters().getPreviewSize().width;
        mH = mCamera.getParameters().getPreviewSize().height;
        mCameraParameters = new VpCameraParameters();
        mCameraParameters.initPersProjWithoutDistortion(615.1674805, 615.1675415, 312.1889954, 243.4373779);
        mTagSize = 0.053;

        mDetectorAprilTag = new VpDetectorAprilTag();
        mMethod = 0;
        mDetectorAprilTag.setAprilTagFamily(mMethods[mMethod]); // TAG_36h11
        mDetectorAprilTag.setAprilTagDecisionMarginThreshold(mDetectionMarginThresholds[mMethod]);
    }

    public void surfaceCreated(SurfaceHolder holder) {
        // The Surface has been created, now tell the camera where to draw the preview.
        try {
            mLastTime = System.currentTimeMillis();
            mCamera.setPreviewDisplay(holder);
            mCamera.startPreview();
            Log.d(TAG, "Camera preview started.");
        } catch (IOException e) {
            Log.d(TAG, "Error setting camera preview: " + e.getMessage());
        }
    }

    public void surfaceDestroyed(SurfaceHolder holder) {
        // empty. Take care of releasing the Camera preview in your activity.
    }

    public void surfaceChanged(SurfaceHolder holder, int format, int w, int h) {
        // If your preview can change or rotate, take care of those events here.
        // Make sure to stop the preview before resizing or reformatting it.

        if (mHolder.getSurface() == null) {
            // preview surface does not exist
            Log.d(TAG, "Preview surface does not exist");
            return;
        }

        // stop preview before making changes
        try {
            mCamera.stopPreview();
            Log.d(TAG, "Preview stopped.");
        } catch (Exception e) {
            // ignore: tried to stop a non-existent preview
            Log.e(TAG, "Error starting camera preview: " + e.getMessage());
        }

        // Now make changes
        int orientation = calculatePreviewOrientation(mCameraInfo, mDisplayOrientation);
        mCamera.setDisplayOrientation(orientation);

        mLastTime = System.currentTimeMillis();

        try {
            mCamera.setPreviewCallback(this);
            mCamera.setPreviewDisplay(mHolder);
            mCamera.startPreview();

            Log.d(TAG, "Camera preview started.");
        } catch (Exception e) {
            Log.e(TAG, "Error starting camera preview: " + e.getMessage());
        }
    }

    /**
     * Calculate the correct orientation for a {@link Camera} preview that is displayed on screen.
     *
     * Implementation is based on the sample code provided in
     * {@link Camera#setDisplayOrientation(int)}.
     */
    public static int calculatePreviewOrientation(Camera.CameraInfo info, int rotation) {
        int degrees = 0;

        switch (rotation) {
            case Surface.ROTATION_0:
                degrees = 0;
                break;
            case Surface.ROTATION_90:
                degrees = 90;
                break;
            case Surface.ROTATION_180:
                degrees = 180;
                break;
            case Surface.ROTATION_270:
                degrees = 270;
                break;
        }

//        degrees = (degrees + 90)%360;

        int result;
        if (info.facing == Camera.CameraInfo.CAMERA_FACING_FRONT) {
            result = (info.orientation + degrees) % 360;
            result = (360 - result) % 360;  // compensate the mirror
        } else {  // back-facing
            result = (info.orientation - degrees + 360) % 360;
        }

        return result;
    }

//    private VpImagePoint project(VpHomogeneousMatrix cMo, VpPoint obj) {
//        obj.changeFrame(cMo);
//        obj.projection();
//        double px = 600, py = 600, u0 = mW/2.0, v0 = mH/2.0;
//        double u = px * obj.get_x() + u0;
//        double v = py * obj.get_y() + v0;
//
//        return new VpImagePoint(v, u);
//    }

    public void onPreviewFrame(byte[] data, Camera camera) {
//        if (System.currentTimeMillis() > 50 + mLastTime)
        {
            mImageUChar = new VpImageUChar(data, mH, mW,true);

            // do the image processing
            // Its working even without grey scale conversion
            // TODO: check the original image color space
            List<VpHomogeneousMatrix> matrices = mDetectorAprilTag.detect(mImageUChar, mTagSize, mCameraParameters);

            int[] tags_id = mDetectorAprilTag.getTagsId();
            Log.d("CameraPreview.java",matrices.size() + " tags detected");
            Log.d("CameraPreview.java", "tags_id=" + Arrays.toString(tags_id));

            int strokeWidth = 8;
            if (!matrices.isEmpty()) {
                List<List<VpImagePoint>> tagsCorners_visp = mDetectorAprilTag.getTagsCorners();
                int[] tag_ids = mDetectorAprilTag.getTagsId();

                updateResults(tagsCorners_visp, strokeWidth, tag_ids,
                        matrices.size() + " tags with id= " + Arrays.toString(tags_id) + " detected within "
                        + (System.currentTimeMillis() - mLastTime) +" ms");
            } else {
                // Display text info
                List<List<VpImagePoint>> tagsCorners = new ArrayList<List<VpImagePoint>>();

                int[] emptyIds = {};
                updateResults(tagsCorners, strokeWidth, emptyIds,
                        matrices.size() + " tags with id= " + Arrays.toString(tags_id) + " detected within "
                        + (System.currentTimeMillis() - mLastTime) +" ms");
            }

            // TODO: read this
            // https://www.geeksforgeeks.org/android/mvc-model-view-controller-architecture-pattern-in-android-with-example/
            // https://stackoverflow.com/questions/57374850/draw-on-androids-surfaceview
            // https://innovationm.com/blog/custom-camera-using-surfaceview/
            // https://stackoverflow.com/questions/11544877/really-confused-with-setpreviewcallback-in-android-need-advice
            // ?
            // https://www.wangxiang.work/2017/10/19/android/exception/java.lang.IllegalArgumentException-Surface.lockCanvas-Surface%20was%20already%20locked/
            // https://abhiandroid.com/ui/framelayout#gsc.tab=0
            // https://stackoverflow.com/questions/31056316/android-surfaceholder-getsurface-results-in-null-pointer-exception
            // http://supertos.free.fr/supertos.php?page=1068
            // https://www.dev2qa.com/android-surfaceview-drawing-example/
            // https://stackoverflow.com/questions/4965724/layered-surfaceviews-in-a-framelayout-in-android
            // https://stackoverflow.com/questions/57742739/drawing-on-surfaceview

            mLastTime = System.currentTimeMillis();
        }
    }

    public void setAprilTagMethod(int selection) {
        mMethod = selection;
        mDetectorAprilTag.setAprilTagFamily(mMethods[mMethod]);
        mDetectorAprilTag.setAprilTagDecisionMarginThreshold(mDetectionMarginThresholds[mMethod]);
    }
}
