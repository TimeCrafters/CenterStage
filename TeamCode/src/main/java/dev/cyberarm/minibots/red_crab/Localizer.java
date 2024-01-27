package dev.cyberarm.minibots.red_crab;

import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.slf4j.helpers.Util;

import dev.cyberarm.engine.V2.Utilities;

public class Localizer {
    private final RedCrabMinibot robot;
    private double rawX = 0, rawY = 0, rawR = 0;
    private double trackWidthMM = 365.0, forwardOffsetMM = 140.0, wheelDiameterMM = 90.0;
    private int lastEncoderXLeft = 0, lastEncoderXRight = 0, lastEncoderYCenter = 0;

    public Localizer(RedCrabMinibot robot) {
        this.robot = robot;

        // Preset last encoder to current location to not require resetting encoders, ever. (🤞)
        this.lastEncoderXLeft = robot.deadWheelXLeft.getCurrentPosition();
        this.lastEncoderXRight = robot.deadWheelXRight.getCurrentPosition();
        this.lastEncoderYCenter = robot.deadWheelYCenter.getCurrentPosition();
    }

    public void reset() {
        rawX = 0;
        rawY = 0;
        rawR = 0;
    }

    // FIXME
    public void teleport(double xMM, double yMM) {
        this.rawX = xMM;
        this.rawY = yMM;
    }

    // FIXME
    public void teleport(double xMM, double yMM, double headingDegrees) {
        this.rawX = xMM;
        this.rawY = yMM;
        this.rawR = (AngleUnit.DEGREES).toRadians(AngleUnit.normalizeDegrees(headingDegrees)); // cursed :(
    }

    public void integrate() {
        int leftEncoder = robot.deadWheelXLeft.getCurrentPosition();
        int rightEncoder = robot.deadWheelXRight.getCurrentPosition();
        int centerEncoder = robot.deadWheelYCenter.getCurrentPosition();

        int deltaLeft = leftEncoder - lastEncoderXLeft;
        int deltaRight = rightEncoder - lastEncoderXRight;
        int deltaCenter = centerEncoder - lastEncoderYCenter;

        double phi = (deltaLeft - deltaRight) / trackWidthMM;
        double deltaMiddle = (deltaLeft + deltaRight) / 2.0;
        double deltaPerp = deltaCenter - forwardOffsetMM * phi;

        double heading = rawR + phi;
        double deltaX = deltaMiddle * Math.cos(heading) - deltaPerp * Math.sin(heading);
        double deltaY = deltaMiddle * Math.sin(heading) + deltaPerp * Math.cos(heading);

        rawX += deltaX;
        rawY += deltaY;
        rawR += phi;

        lastEncoderXLeft = leftEncoder;
        lastEncoderXRight = rightEncoder;
        lastEncoderYCenter = centerEncoder;
    }

    public double xMM() {
        return Utilities.ticksToUnit(8192, 1, wheelDiameterMM, DistanceUnit.MM, (int)rawX);
    }

    public double yMM() {
        return Utilities.ticksToUnit(8192, 1, wheelDiameterMM, DistanceUnit.MM, (int)rawY);
    }

    // FIXME
    public double headingDegrees() {
        return rawR;
    }

    // FIXME? (report radians as halves or proper whole?)
    public double headingRadians() {
        return rawR;
    }
}
