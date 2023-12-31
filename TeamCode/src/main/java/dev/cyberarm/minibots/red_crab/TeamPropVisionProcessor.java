package dev.cyberarm.minibots.red_crab;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Typeface;

import com.acmerobotics.dashboard.canvas.Stroke;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class TeamPropVisionProcessor implements VisionProcessor {
    public enum Location {
        NONE,
        LEFT,
        CENTER,
        RIGHT
    }

    private enum NamedColorPart {
        HUE,
        SATURATION,
        VALUE
    }

    private Location location = Location.NONE;

    /// NOTE: Rects are defined with a 640x480 (WxH) frame size assumed
    // 640 / 3 = ~212
    private final Rect rectLeft = new Rect(1, 1, 212, 479);
    private final Rect rectCenter = new Rect(rectLeft.x + rectLeft.width, 1, 212, 479);
    private final Rect rectRight = new Rect(rectCenter.x + rectCenter.width, 1, 212, 479);
    private Mat subMat = new Mat();
    private Mat rotatedMat = new Mat();
    private Mat hsvMat = new Mat();
    private double saturationLeft;
    private double saturationCenter;
    private double saturationRight;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
//        Core.rotate(frame, rotatedMat,Core.ROTATE_180);
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        saturationLeft = averageSaturation(hsvMat, rectLeft);
        saturationCenter = averageSaturation(hsvMat, rectCenter);
        saturationRight = averageSaturation(hsvMat, rectRight);

        if (saturationLeft > saturationCenter && saturationLeft > saturationRight) {
            location = Location.LEFT;
        } else if (saturationCenter > saturationLeft && saturationCenter > saturationRight) {
            location =  Location.CENTER;
        } else {
            location = Location.RIGHT;
        }

        return location;
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
        notSelectedPaint.setAlpha(127);
        notSelectedPaint.setStyle(Paint.Style.STROKE);
        notSelectedPaint.setStrokeWidth(scaleCanvasDensity * 4);
        selectedPaint.setTextSize(scaleCanvasDensity * 28);
        selectedPaint.setTextAlign(Paint.Align.CENTER);
        selectedPaint.setTypeface(Typeface.MONOSPACE);

        Paint whitePaint = new Paint();
        whitePaint.setColor(Color.WHITE);
        whitePaint.setStyle(Paint.Style.FILL_AND_STROKE);
        whitePaint.setStrokeWidth(scaleCanvasDensity * 4);
        whitePaint.setTextSize(scaleCanvasDensity * 28);
        whitePaint.setTextAlign(Paint.Align.CENTER);
        whitePaint.setTypeface(Typeface.MONOSPACE);

        android.graphics.Rect drawRectLeft = makeDrawableRect(rectLeft, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectCenter = makeDrawableRect(rectCenter, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectRight = makeDrawableRect(rectRight, scaleBmpPxToCanvasPx);

        float textYOffset = selectedPaint.getTextSize() + selectedPaint.getStrokeWidth();

        switch ((Location) userContext) {
            case LEFT:
                canvas.drawRect(drawRectLeft, selectedPaint);
                canvas.drawRect(drawRectCenter, notSelectedPaint);
                canvas.drawRect(drawRectRight, notSelectedPaint);

                canvas.drawText("LEFT", drawRectLeft.centerX(), textYOffset, selectedPaint);
                break;
            case CENTER:
                canvas.drawRect(drawRectLeft, notSelectedPaint);
                canvas.drawRect(drawRectCenter, selectedPaint);
                canvas.drawRect(drawRectRight, notSelectedPaint);

                canvas.drawText("CENTER", drawRectCenter.centerX(), textYOffset, selectedPaint);
                break;
            case RIGHT:
                canvas.drawRect(drawRectLeft, notSelectedPaint);
                canvas.drawRect(drawRectCenter, notSelectedPaint);
                canvas.drawRect(drawRectRight, selectedPaint);

                canvas.drawText("RIGHT", drawRectRight.centerX(), textYOffset, selectedPaint);
                break;
            case NONE:
                canvas.drawRect(drawRectLeft, notSelectedPaint);
                canvas.drawRect(drawRectCenter, notSelectedPaint);
                canvas.drawRect(drawRectRight, notSelectedPaint);
                break;
        }
        canvas.drawText(String.format("%.4f", getSaturationLeft()), drawRectLeft.centerX(), textYOffset + whitePaint.getTextSize() + textYOffset, whitePaint);
        canvas.drawText(String.format("%.4f", getSaturationCenter()), drawRectCenter.centerX(), textYOffset + whitePaint.getTextSize() + textYOffset, whitePaint);
        canvas.drawText(String.format("%.4f", getSaturationRight()), drawRectRight.centerX(), textYOffset + whitePaint.getTextSize() + textYOffset, whitePaint);

    }

    private double averageSaturation(Mat input, Rect rect) {
        // NOTE: Check whether this submat Mat leaks memory
        subMat = input.submat(rect);
        Scalar color = Core.mean(subMat);

        return color.val[NamedColorPart.SATURATION.ordinal()];
    }

    public Location getLocation() {
        return location;
    }

    public double getSaturationLeft() {
        return saturationLeft;
    }

    public double getSaturationCenter() {
        return saturationCenter;
    }

    public double getSaturationRight() {
        return saturationRight;
    }

    public double getHighestSaturation() {
        if (getSaturationLeft() > getSaturationCenter() && getSaturationLeft() > getSaturationRight())
            return getSaturationLeft();
        if (getSaturationCenter() > getSaturationLeft() && getSaturationCenter() > getSaturationRight())
            return getSaturationCenter();

        return getSaturationRight();
    }

    private android.graphics.Rect makeDrawableRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }
}
