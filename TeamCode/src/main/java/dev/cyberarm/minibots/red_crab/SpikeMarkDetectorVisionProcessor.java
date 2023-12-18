package dev.cyberarm.minibots.red_crab;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Typeface;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class SpikeMarkDetectorVisionProcessor implements VisionProcessor {
    public enum Selection {
        NONE,
        LINE
    }

    private enum NamedColorPart {
        HUE,
        SATURATION,
        VALUE
    }

    private Selection selection = Selection.NONE;

    /// NOTE: Rect defined with a 640x480 (WxH) frame size assumed
    private final Rect rect = new Rect(1, 1, 639, 479);
    private Mat subMat = new Mat();
    private Mat rotatedMat = new Mat();
    private Mat hsvMat = new Mat();

    private double saturation;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Core.rotate(frame, rotatedMat,Core.ROTATE_180);
        Imgproc.cvtColor(rotatedMat, hsvMat, Imgproc.COLOR_RGB2HSV);

        saturation = averageSaturation(hsvMat, rect);

        // TODO: Tune this value or do more processing
        if (saturation > 0.5) {
            selection = Selection.LINE;
        } else {
            selection = Selection.NONE;
        }

        return selection;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.GREEN);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);
        selectedPaint.setTextSize(scaleCanvasDensity * 28);
        selectedPaint.setTextAlign(Paint.Align.CENTER);
        selectedPaint.setTypeface(Typeface.MONOSPACE);

        Paint notSelectedPaint = new Paint();
        notSelectedPaint.setColor(Color.WHITE);
        notSelectedPaint.setStyle(Paint.Style.STROKE);
        notSelectedPaint.setStrokeWidth(scaleCanvasDensity * 4);
        selectedPaint.setTextSize(scaleCanvasDensity * 28);
        selectedPaint.setTextAlign(Paint.Align.CENTER);
        selectedPaint.setTypeface(Typeface.MONOSPACE);

        android.graphics.Rect drawRect = makeDrawableRect(rect, scaleBmpPxToCanvasPx);

        float textYOffset = selectedPaint.getTextSize() + selectedPaint.getStrokeWidth();

        switch ((Selection) userContext) {
            case LINE:
                canvas.drawRect(drawRect, notSelectedPaint);

                canvas.drawText("LINE!", drawRect.centerX(), drawRect.bottom - textYOffset, selectedPaint);
                break;
            case NONE:
                canvas.drawRect(drawRect, notSelectedPaint);
                break;
        }
    }

    private double averageSaturation(Mat input, Rect rect) {
        // NOTE: Check whether this submat Mat leaks memory
        subMat = input.submat(rect);
        Scalar color = Core.mean(subMat);

        return color.val[NamedColorPart.SATURATION.ordinal()];
    }

    public Selection getSelection() {
        return selection;
    }

    public double getSaturation() {
        return saturation;
    }

    private android.graphics.Rect makeDrawableRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }
}
