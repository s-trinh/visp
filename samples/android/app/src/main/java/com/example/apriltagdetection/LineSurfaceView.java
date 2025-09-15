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

import java.util.List;

public class LineSurfaceView extends SurfaceView implements SurfaceHolder.Callback {
    private static final String TAG = "LineSurfaceView";
    private SurfaceHolder mSurfaceHolder;

    public LineSurfaceView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    private void init() {
        mSurfaceHolder = getHolder();
        mSurfaceHolder.addCallback(this);
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder) {
        Log.d(TAG, "LineSurfaceView::surfaceCreated()");
    }

    public void clear() {
        Log.d(TAG, "LineSurfaceView::clear()");
        Canvas canvas = mSurfaceHolder.lockCanvas();
        Log.d(TAG, "LineSurfaceView::clear() ; (canvas != null)=" + (canvas != null));
        if (canvas != null) {
            // https://stackoverflow.com/a/9035709
            canvas.drawColor( 0, PorterDuff.Mode.CLEAR );
            mSurfaceHolder.unlockCanvasAndPost(canvas);
        }
    }

    public void drawLines(List<double[]> list_startX, List<double[]> list_startY, List<double[]> list_stopX, List<double[]> list_stopY,
                          int[] color_, int[] strokeWidth_, List<Double> centerX, List<Double> centerY, int[] ids_) {
        Log.d(TAG, "LineSurfaceView::drawLine()");
        Canvas canvas = mSurfaceHolder.lockCanvas();
        Log.d(TAG, "LineSurfaceView::drawLine() ; (canvas != null)=" + (canvas != null));

        if (canvas != null) {
            int view_w = getWidth();
//            int view_h = getHeight();

            for (int i = 0; i < list_startX.size(); i++) {
                for (int j = 0; j < list_startX.get(i).length; j++) {

                    double startX = list_startX.get(i)[j];
                    double stopX = list_stopX.get(i)[j];
                    double startY = list_startY.get(i)[j];
                    double stopY = list_stopY.get(i)[j];

                    int color = color_[j];
                    int strokeWidth = strokeWidth_[j];

                    Paint paint = new Paint();
                    paint.setColor(color);
                    paint.setStrokeWidth(strokeWidth);
                    // TODO: (view_w - startX)
                    canvas.drawLine((float) (view_w - startX), (float) startY, (float) (view_w - stopX), (float) stopY, paint);
                }

                // Draw id
                Paint paint = new Paint();
                paint.setColor(Color.parseColor("aqua"));
                paint.setStrokeWidth(8);
                paint.setTextSize(40);
                paint.setTextAlign(Paint.Align.CENTER);

                canvas.drawText(String.valueOf(ids_[i]), view_w - centerX.get(i).floatValue(), centerY.get(i).floatValue(), paint);
            }

            mSurfaceHolder.unlockCanvasAndPost(canvas);
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
