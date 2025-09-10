package com.example.apriltagdetection;

import android.content.Context;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Rect;
import android.graphics.YuvImage;

import android.hardware.Camera;
import android.util.Log;
import android.view.Surface;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.widget.ImageView;

import org.visp.core.VpCameraParameters;
import org.visp.core.VpHomogeneousMatrix;
import org.visp.core.VpImageUChar;
import org.visp.detection.VpDetectorAprilTag;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.List;

import static com.example.apriltagdetection.CameraPreviewActivity.updateResult;

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
    private SurfaceView surfaceView;
    private Canvas canvas;
    private ImageView mImageView;
    private Camera mCamera;
    private Camera.CameraInfo mCameraInfo;
    private int mDisplayOrientation;
    private long lastTime;
    private int w, h;
    private VpCameraParameters cameraParameters;
    private double tagSize;

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
//        surfaceView.getHolder().addCallback( this );

//        mImageView = findViewById(R.id.imageView);
//        if (mImageView == null) {
//            Log.e("CameraPreview", "ImageView is null");
//        }

////        /// NOK, return null
////        // https://stackoverflow.com/questions/57742739/drawing-on-surfaceview
//        surfaceView = (SurfaceView) findViewById( R.id.surfaceView );
//        surfaceView.setZOrderOnTop(true);

        // init the ViSP tag detection system
        w = mCamera.getParameters().getPreviewSize().width;
        h = mCamera.getParameters().getPreviewSize().height;
        cameraParameters = new VpCameraParameters();
        cameraParameters.initPersProjWithoutDistortion(615.1674805, 615.1675415, 312.1889954, 243.4373779);
        tagSize = 0.053;
    }

    public void surfaceCreated(SurfaceHolder holder) {
        // The Surface has been created, now tell the camera where to draw the preview.
        try {
            lastTime = System.currentTimeMillis();
            mCamera.setPreviewDisplay(holder);
            mCamera.startPreview();
            Log.d(TAG, "Camera preview started.");


//        /// NOK, return null
//        // https://stackoverflow.com/questions/57742739/drawing-on-surfaceview
            surfaceView = (SurfaceView) findViewById( R.id.surfaceView );
//            surfaceView.setZOrderOnTop(true);
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
            Log.d(TAG, "Error starting camera preview: " + e.getMessage());
        }

        // Now make changes
        int orientation = calculatePreviewOrientation(mCameraInfo, mDisplayOrientation);
        mCamera.setDisplayOrientation(orientation);

        lastTime = System.currentTimeMillis();

        try {
            mCamera.setPreviewCallback(this);
            mCamera.setPreviewDisplay(mHolder);
            mCamera.startPreview();

            Log.d(TAG, "Camera preview started.");
        } catch (Exception e) {
            Log.d(TAG, "Error starting camera preview: " + e.getMessage());
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

    // Getting 24 FPS, 640x480 size images
    public void onPreviewFrame(byte[] data, Camera camera) {
        if (System.currentTimeMillis() > 50 + lastTime) {
            VpImageUChar imageUChar = new VpImageUChar(data,h,w,true);

            // do the image processing
            // Its working even without grey scale conversion
            VpDetectorAprilTag detectorAprilTag = new VpDetectorAprilTag();
            detectorAprilTag.setAprilTagFamily(23); // TAG_ARUCO_MIP_36h12
            List<VpHomogeneousMatrix> matrices = detectorAprilTag.detect(imageUChar,tagSize,cameraParameters);
            int[] tags_id = detectorAprilTag.getTagsId();
            Log.d("CameraPreview.java",matrices.size() + " tags detected");
            for (int tag_id : tags_id) {
                Log.d("CameraPreview.java", "tag_id=" + tag_id);
            }

            Log.d("CameraPreview.java", "tags_id=" + Arrays.toString(tags_id));
            updateResult(data, w, h, matrices.size() + " tags with id= " + Arrays.toString(tags_id) + " detected within "
                    + (System.currentTimeMillis() - lastTime) +" ms");

            Log.e("CameraPreview", "camera.getPreviewFormat()=" + camera.getParameters().getPreviewFormat()); // 17 ; NV21
//            if (mImageView == null) {
//                mImageView = findViewById(R.id.imageView);
//            }
//            Log.e("CameraPreview", "(mImageView != null)? " + (mImageView != null));

            Log.e("CameraPreview", "(mHolder != null)? " + (mHolder != null));

//            if (mHolder.getSurface() != null) {
//                Paint paint = new Paint();
//                paint.setColor(Color.RED); // Red color
//                int lineWidth = 10;
//                paint.setStrokeWidth(lineWidth); // Set the width of the line
//                paint.setAntiAlias(true); // Smooth out the edges of the line
//
//                // Draw a red line on the canvas (example: from (50, 50) to (500, 500))
//                canvas = mHolder.getSurface().lockCanvas(new Rect());
//                canvas.drawLine(50, 50, 500, 500, paint);
//                mHolder.unlockCanvasAndPost(canvas);
//            }
//            Canvas canvas = mHolder.lockCanvas();
//            if (canvas != null) {
//                // Set up the paint for drawing the red line
//                Paint paint = new Paint();
//                paint.setColor(Color.RED); // Red color
//                int lineWidth = 10;
//                paint.setStrokeWidth(lineWidth); // Set the width of the line
//                paint.setAntiAlias(true); // Smooth out the edges of the line
//
//                // Draw a red line on the canvas (example: from (50, 50) to (500, 500))
//                canvas.drawLine(50, 50, 500, 500, paint);
//                mHolder.unlockCanvasAndPost(canvas);
//            }

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

//            // Convert the byte[] preview frame to a Bitmap
////            Bitmap bitmap = getBitmapFromPreviewFrame(data, camera);
//
//            Camera.Parameters parameters = camera.getParameters();
//            int preview_format = parameters.getPreviewFormat();
//
//            // Convert the byte[] preview frame to a Bitmap
//            Bitmap bitmap = convertNV21ToBitmap(data, camera);
//            // Check if the bitmap is mutable
//            if (!bitmap.isMutable()) {
//                bitmap = bitmap.copy(Bitmap.Config.ARGB_8888, true); // Create a mutable copy of the bitmap
//            }
//
//            Log.e("CameraPreview", "(bitmap != null)? " + (bitmap != null));
//            if (bitmap != null) {
//                // Modify the Bitmap here (e.g., apply filters or transformations)
//
//                // Create a canvas to draw on the bitmap
//                Canvas canvas = new Canvas(bitmap);
//
//                // Set up the paint for drawing the red line
//                Paint paint = new Paint();
//                paint.setColor(Color.RED); // Red color
//                int lineWidth = 10;
//                paint.setStrokeWidth(lineWidth); // Set the width of the line
//                paint.setAntiAlias(true); // Smooth out the edges of the line
//
//                // Draw a red line on the canvas (example: from (50, 50) to (500, 500))
//                canvas.drawLine(50, 50, 500, 500, paint);
//
//                // Update the ImageView with the modified Bitmap
//                mImageView.setImageBitmap(bitmap);
//            }

            lastTime = System.currentTimeMillis();
        }
    }

    // Convert raw camera frame (NV21) to Bitmap
    private Bitmap convertNV21ToBitmap(byte[] data, Camera camera) {
        Camera.Parameters parameters = camera.getParameters();
        int previewWidth = parameters.getPreviewSize().width;
        int previewHeight = parameters.getPreviewSize().height;

        // Create a YuvImage from the NV21 byte array
        YuvImage yuvImage = new YuvImage(data, parameters.getPreviewFormat(), previewWidth, previewHeight, null);

        // Compress the YUV image to a JPEG output stream
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        yuvImage.compressToJpeg(new android.graphics.Rect(0, 0, previewWidth, previewHeight), 100, out);

        // Get the byte array from the JPEG output stream
        byte[] byteArray = out.toByteArray();

        // Decode the byte array into a Bitmap
        return BitmapFactory.decodeByteArray(byteArray, 0, byteArray.length);
    }
}
