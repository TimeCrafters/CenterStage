package dev.cyberarm.minibots.red_crab;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.kinematics.Odometry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.slf4j.helpers.Util;

import dev.cyberarm.engine.V2.Utilities;

public class Localizer {
    private final RedCrabMinibot robot;
    private double rawX = 0, rawY = 0, rawR = 0, offsetX = 0, offsetY = 0;
    private final double trackWidthMM = 365.0, forwardOffsetMM = 140.0, wheelDiameterMM = 90.0;
    private final int encoderTicksPerRevolution = 8192;
    private final double encoderGearRatio = 1;
    private double lastEncoderXLeftMM, lastEncoderXRightMM, lastEncoderYCenterMM;
//    private double xDeltaMultiplier = 0.87012987, yDeltaMultiplier = 0.25;
    private double xDeltaMultiplier = 1, yDeltaMultiplier = 1;
    private HolonomicOdometry odometry;
    public Localizer(RedCrabMinibot robot) {
        this.robot = robot;

        // Preset last encoder to current location to not require resetting encoders, ever. (🤞)
        this.lastEncoderXLeftMM = ticksToMM(robot.deadWheelXLeft.getCurrentPosition());
        this.lastEncoderXRightMM = ticksToMM(robot.deadWheelXRight.getCurrentPosition());
        this.lastEncoderYCenterMM = ticksToMM(robot.deadWheelYCenter.getCurrentPosition());

        this.odometry = new HolonomicOdometry(
                this::leftDistance,
                this::rightDistance,
                this::centerDistance,
                trackWidthMM,
                forwardOffsetMM
        );
    }

    public void reset() {
        robot.resetDeadWheels();

        odometry = new HolonomicOdometry(
                this::leftDistance,
                this::rightDistance,
                this::centerDistance,
                trackWidthMM,
                forwardOffsetMM
        );

        rawX = 0;
        rawY = 0;
        rawR = 0;

        offsetX = 0;
        offsetY = 0;
    }

    // Meant for setting starting location offset
    public void teleport(double xMM, double yMM) {
        this.offsetX = xMM;
        this.offsetY = yMM;
    }

    // FIXME: We need to be able to set rotation to +/- 180 so we can use absolute field coordinates for target location(s)
    //          and use odometry position as "true" field location.
    public void teleport(double xMM, double yMM, double headingDegrees) {
        this.offsetX = xMM;
        this.offsetY = yMM;
        this.rawR = 0; // FIXME HERE // (AngleUnit.DEGREES).toRadians(AngleUnit.normalizeDegrees(headingDegrees)); // cursed :(
    }

    public void integrate() {
        odometry.updatePose();

//        double leftEncoder = ticksToMM(robot.deadWheelXLeft.getCurrentPosition());
//        double rightEncoder = ticksToMM(robot.deadWheelXRight.getCurrentPosition());
//        double centerEncoder = ticksToMM(robot.deadWheelYCenter.getCurrentPosition());
//
//        double deltaLeft = leftEncoder - lastEncoderXLeftMM;
//        double deltaRight = rightEncoder - lastEncoderXRightMM;
//        double deltaCenter = centerEncoder - lastEncoderYCenterMM;
//
//        double phi = (deltaLeft - deltaRight) / trackWidthMM;
//        double deltaMiddle = (deltaLeft + deltaRight) / 2.0;
//        //  double deltaPerp = deltaCenter - forwardOffsetMM * phi;
//        double deltaPerp = deltaCenter - (deltaRight - deltaLeft) * forwardOffsetMM / trackWidthMM;
//
//        double heading = rawR + (phi / 2.0);
//        double deltaX = (deltaMiddle * Math.cos(heading) - deltaPerp * Math.sin(heading)) * xDeltaMultiplier;
//        double deltaY = (deltaMiddle * Math.sin(heading) + deltaPerp * Math.cos(heading)) * yDeltaMultiplier;
//
//        rawX += deltaX;
//        rawY += deltaY;
//        rawR += phi;
//
//        lastEncoderXLeftMM = leftEncoder;
//        lastEncoderXRightMM = rightEncoder;
//        lastEncoderYCenterMM = centerEncoder;
    }

    public double xMM() {
        return odometry.getPose().getX() + offsetX; //rawX;
    }

    public double yMM() {
        return odometry.getPose().getY() + offsetY;  //rawY;
    }

    public double xIn() {
        return xMM() * 0.03937008;
    }
    public double yIn() {
        return yMM() * 0.03937008;
    }

    // Returns true 360 degrees
    public double headingDegrees() {
        double degrees = headingRadians() * 180.0 / Math.PI;
        return ((degrees + 360.0) % 360.0) % 360.0;
    }

    // Returns annoying half-split +/- PI radians
    public double headingRadians() {
        return odometry.getPose().getHeading(); // rawR;
    }

    private double ticksToMM(int ticks) {
        return Utilities.ticksToUnit(encoderTicksPerRevolution, encoderGearRatio, wheelDiameterMM, DistanceUnit.MM, ticks);
    }

    public double leftDistance() {
        return ticksToMM(robot.deadWheelXLeft.getCurrentPosition());
    }

    public double rightDistance() {
        return ticksToMM(robot.deadWheelXRight.getCurrentPosition());
    }

    public double centerDistance() {
        return ticksToMM(robot.deadWheelYCenter.getCurrentPosition());
    }
}
