package dev.cyberarm.engine.V2;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Utilities {
    /***
     * The current heading of the robot as a full 360 degree value, instead of that half radian mess.
     * @param imu IMU of rev hub or similar
     * @param imuAngleOffset optional angle offset added to IMU value
     * @return full range heading of the robot, in DEGREES.
     */
    static public double facing(IMU imu, double imuAngleOffset) {
        double imuDegrees = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        return (((imuDegrees + 360.0) % 360.0) + imuAngleOffset) % 360.0;
    }

    static public double facing(IMU imu) {
        return facing(imu, 0);
    }

    static public double heading(double facing) {
        return AngleUnit.normalizeRadians(-facing * Math.PI / 180.0);
    }

    static public double turnRate(IMU imu) {
        return imu.getRobotAngularVelocity(AngleUnit.DEGREES).yRotationRate;
    }

    /***
     *
     * @param value value to check with
     * @param min minimum value
     * @param max maximum value
     * @return true if value is between min and max. inclusive.
     */
    static public boolean isBetween(double value, double min, double max) {
        return value >= min && value <= max;
    }

    static public boolean isBetween(int value, int min, int max) {
        return value >= min && value <= max;
    }

    // Adapted from: https://github.com/gosu/gosu/blob/980d64de2ce52e4b16fdd5cb9c9e11c8bbb80671/src/Math.cpp#L38

    /***
     * The angular difference between two angles
     * **NOTE** flip flopped from and to values may result in continuous inversion of the angle difference (180 to -180 for example)
     * @param from angle in DEGREES
     * @param to angle in DEGREES
     * @return Angular difference two angles in DEGREES
     */
    static public double angleDiff(double from, double to) {
        double value = (to - from + 180);

        double fmod = (value - 0.0) % (360.0 - 0.0);

        return (fmod < 0 ? fmod + 360.0 : fmod +  0.0) - 180;
    }

    /***
     * Linear interpolation
     * @param min minimum value
     * @param max maximum value
     * @param t factor
     * @return
     */
    static public double lerp(double min, double max, double t)
    {
        return min + (max - min) * t;
    }

    /***
     * Calculates motor angle in ticks
     * @param motorTicksPerRevolution
     * @param gearRatio
     * @param angleInDegrees
     * @return Angle in motor ticks
     */
    static public int motorAngle(int motorTicksPerRevolution, double gearRatio, double angleInDegrees) {
        return (int) (angleInDegrees / (360.0 / (motorTicksPerRevolution * gearRatio)));
    }
}
