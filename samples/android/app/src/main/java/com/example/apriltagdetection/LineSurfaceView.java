package com.example.apriltagdetection;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.PorterDuff;
import android.util.AttributeSet;
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.SurfaceView;

public class LineSurfaceView extends SurfaceView implements SurfaceHolder.Callback {

    private static final String TAG = "LineSurfaceView";
    private SurfaceHolder surfaceHolder;
    private Paint paint;

    public LineSurfaceView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    private void init() {
        surfaceHolder = getHolder();
        surfaceHolder.addCallback(this);
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder) {
        Log.d(TAG, "LineSurfaceView::surfaceCreated()");

//        Canvas canvas = surfaceHolder.lockCanvas();
//        Log.d(TAG, "LineSurfaceView::drawLine() ; (canvas != null)=" + (canvas != null));
//        if (canvas != null) {
//            Paint paint = new Paint();
//            int YELLOW = -256;
//            paint.setColor(YELLOW);
//            paint.setStrokeWidth(20);
//            canvas.drawLine((float) 10, (float) 50, (float) 100, (float) 500, paint);
//            surfaceHolder.unlockCanvasAndPost(canvas);
//        }
    }

    public void clear() {
        Log.d(TAG, "LineSurfaceView::clear()");
        Canvas canvas = surfaceHolder.lockCanvas();
        Log.d(TAG, "LineSurfaceView::clear() ; (canvas != null)=" + (canvas != null));
        if (canvas != null) {
            // https://stackoverflow.com/a/9035709
            canvas.drawColor( 0, PorterDuff.Mode.CLEAR );
            surfaceHolder.unlockCanvasAndPost(canvas);
        }
    }

    public void drawLine(double startX, double startY, double stopX, double stopY, int color, int strokeWidth) {
        Log.d(TAG, "LineSurfaceView::drawLine()");
        Canvas canvas = surfaceHolder.lockCanvas();
        Log.d(TAG, "LineSurfaceView::drawLine() ; (canvas != null)=" + (canvas != null));
        if (canvas != null) {
//            canvas.drawColor(Color.WHITE);
            Log.d(TAG, "startX=" + startX + " ; stopX=" + stopX + " ; startY=" + startY + " ; stopY=" + stopY);

            Paint paint = new Paint();
            paint.setColor(color);
            paint.setStrokeWidth(strokeWidth);
            canvas.drawLine((float) startX, (float) startY, (float) stopX, (float) stopY, paint);
            surfaceHolder.unlockCanvasAndPost(canvas);
        }
    }

    public void drawLine(double[] startX_, double[] startY_, double[] stopX_, double[] stopY_, int[] color_, int[] strokeWidth_) {
        Log.d(TAG, "LineSurfaceView::drawLine()");
        Canvas canvas = surfaceHolder.lockCanvas();
        Log.d(TAG, "LineSurfaceView::drawLine() ; (canvas != null)=" + (canvas != null));
        if (canvas != null) {
//            canvas.drawColor(Color.WHITE);
            for (int i = 0; i < startX_.length; i++) {
                float cam_w = 1280;
                float cam_h = 720;
                int view_w = getWidth();
                int view_h = getHeight();
//                float scale_w = view_w / cam_w;
//                float scale_h = view_h / cam_h;
                float scale_w = 1;
                float scale_h = 1;

                double startX = startX_[i];
                double stopX = stopX_[i];
                double startY = startY_[i];
                double stopY = stopY_[i];
                Log.d(TAG, "startX=" + startX + " ; stopX=" + stopX + " ; startY=" + startY + " ; stopY=" + stopY + " ; view_w=" + view_w + " ; view_h=" + view_h);

                int color = color_[i];
                int strokeWidth = strokeWidth_[i];

                Paint paint = new Paint();
                paint.setColor(color);
                paint.setStrokeWidth(strokeWidth);
                canvas.drawLine((float) (scale_w*startX), (float) (scale_h*startY), (float) (scale_w*stopX), (float) (scale_h*stopY), paint);
            }
            surfaceHolder.unlockCanvasAndPost(canvas);
        }
    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
        Log.d(TAG, "LineSurfaceView::surfaceChanged()");
    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {
        Log.d(TAG, "LineSurfaceView::surfaceDestroyed()");
    }
}
