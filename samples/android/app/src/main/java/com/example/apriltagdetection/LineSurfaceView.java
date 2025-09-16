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
//        Log.d(TAG, "LineSurfaceView::clear()");
        Canvas canvas = mSurfaceHolder.lockCanvas();
        if (canvas != null) {
            // https://stackoverflow.com/a/9035709
            canvas.drawColor( 0, PorterDuff.Mode.CLEAR );
            mSurfaceHolder.unlockCanvasAndPost(canvas);
        }
    }

    public void drawLines(List<double[]> list_startX, List<double[]> list_startY, List<double[]> list_stopX, List<double[]> list_stopY,
                          int[] color_, int[] strokeWidth_, List<Double> centerX, List<Double> centerY, int[] ids_, int orientation,
                          int width, int height) {
//        Log.d(TAG, "LineSurfaceView::drawLine()");
        Canvas canvas = mSurfaceHolder.lockCanvas();

        if (canvas != null) {
            float view_w = getWidth();
            float view_h = getHeight();

            Log.d(TAG, "LineSurfaceView::drawLines() ; view_w=" + view_w + " ; view_h=" + view_h);

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

                    if (orientation == 0) {
                        double scaleX = view_w / width;
                        double scaleY = view_h / height;

                        canvas.drawLine((float) (scaleX * startX), (float) (scaleY * startY),
                                (float) (scaleX * stopX), (float) (scaleY * stopY), paint);
                    } else if (orientation == 90) {
                        double scaleX = view_w / height;
                        double scaleY = view_h / width;

                        canvas.drawLine((float) (scaleX * (view_w - startY)), (float) (scaleY * startX),
                                (float) (scaleX * (view_w - stopY)), (float) (scaleY * stopX), paint);
                    } else {
                        // 180°
                        double scaleX = view_w / width;
                        double scaleY = view_h / height;

                        canvas.drawLine((float) (scaleX * (width - startX)), (float) (scaleY * (height - startY)),
                                (float) (scaleX * (width - stopX)), (float) (scaleY * (height - stopY)), paint);
                    }
                }

                // Draw id
                Paint paint = new Paint();
                paint.setColor(Color.parseColor("aqua"));
                paint.setStrokeWidth(8);
                paint.setTextSize(40);
                paint.setTextAlign(Paint.Align.CENTER);

                if (orientation == 0) {
                    float scaleX = view_w / width;
                    float scaleY = view_h / height;

                    canvas.drawText(String.valueOf(ids_[i]), scaleX * centerX.get(i).floatValue(), scaleY * centerY.get(i).floatValue(), paint);
                } else if (orientation == 90) {
                    float scaleX = view_w / height;
                    float scaleY = view_h / width;

                    canvas.drawText(String.valueOf(ids_[i]), scaleX * (view_w - centerY.get(i).floatValue()), scaleY * (centerX.get(i).floatValue()), paint);
                } else {
                    // 180°
                    float scaleX = view_w / width;
                    float scaleY = view_h / height;

                    canvas.drawText(String.valueOf(ids_[i]), scaleX * (width - centerX.get(i).floatValue()), scaleY * (height - centerY.get(i).floatValue()), paint);
                }
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
