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

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
        Log.d(TAG, "LineSurfaceView::surfaceChanged()");
    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {
        Log.d(TAG, "LineSurfaceView::surfaceDestroyed()");
    }
}
