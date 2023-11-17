package dev.cyberarm.minibots.yellow;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.timecrafters.TimeCraftersConfigurationTool.library.TimeCraftersConfiguration;

import dev.cyberarm.engine.V2.CyberarmEngine;

public class YellowMinibot {
    private final CyberarmEngine engine;
    public final TimeCraftersConfiguration config;
    public MotorEx leftFront, rightFront, leftBack, rightBack;
    public MotorGroup left, right;
    public IMU imu;
    public CRServo droneLauncher;
    public final double wheelCircumference, distancePerTick, imuAngleOffset, initialFacing;
    public YellowMinibot(CyberarmEngine engine) {
        this.engine = engine;

        this.config = new TimeCraftersConfiguration("Vexy_2023");

        leftFront = new MotorEx(engine.hardwareMap, "leftFront");
        rightFront = new MotorEx(engine.hardwareMap, "rightFront");

        leftBack = new MotorEx(engine.hardwareMap, "leftBack");
        rightBack = new MotorEx(engine.hardwareMap, "rightBack");

        wheelCircumference = Math.PI * (50.0 * 2); //wheelRadius * wheelRadius;
        distancePerTick = 288 / wheelCircumference; // raw motor encoder * gear ratio

        leftFront.setDistancePerPulse(distancePerTick);
        rightFront.setDistancePerPulse(distancePerTick);
        leftBack.setDistancePerPulse(distancePerTick);
        rightBack.setDistancePerPulse(distancePerTick);

        left = new MotorGroup(leftFront, leftBack);
        right = new MotorGroup(rightFront, rightBack);

        imu = engine.hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

        imu.initialize(parameters);

        imuAngleOffset = 0;
        initialFacing = facing();

        droneLauncher = engine.hardwareMap.crservo.get("droneLauncher"); /// Port 5
    }

    public void standardTelemetry() {
        engine.telemetry.addLine();

        engine.telemetry.addLine("Motors");
        engine.telemetry.addData(
                "Left Front",
                "Power: %.2f, Current: %.2f mAmp, Position: %d",
                leftFront.motorEx.getPower(),
                leftFront.motorEx.getCurrent(CurrentUnit.MILLIAMPS),
                leftFront.motorEx.getCurrentPosition());
        engine.telemetry.addData(
                "Right Front",
                "Power: %.2f, Current: %.2f mAmp, Position: %d",
                rightFront.motorEx.getPower(),
                rightFront.motorEx.getCurrent(CurrentUnit.MILLIAMPS),
                rightFront.motorEx.getCurrentPosition());
        engine.telemetry.addData(
                "Left Back",
                "Power: %.2f, Current: %.2f mAmp, Position: %d",
                leftBack.motorEx.getPower(),
                leftBack.motorEx.getCurrent(CurrentUnit.MILLIAMPS),
                leftBack.motorEx.getCurrentPosition());
        engine.telemetry.addData(
                "Right Back",
                "Power: %.2f, Current: %.2f mAmp, Position: %d",
                rightBack.motorEx.getPower(),
                rightBack.motorEx.getCurrent(CurrentUnit.MILLIAMPS),
                rightBack.motorEx.getCurrentPosition());

        engine.telemetry.addLine();
        engine.telemetry.addLine("Servos");
        engine.telemetry.addData("droneLauncher", droneLauncher.getPower());

        engine.telemetry.addLine();
    }

    public void teleopTelemetry() {
        engine.telemetry.addLine();

        engine.telemetry.addLine();
    }

    public double distanceMM(int ticks) {
        return distancePerTick * ticks;
    }


    public double initialFacing() {
        return initialFacing;
    }

    public double facing() {
        double imuDegrees = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        return (((imuDegrees + 360.0) % 360.0) + imuAngleOffset) % 360.0;
    }

    public double heading() {
        return AngleUnit.normalizeRadians(-facing() * Math.PI / 180.0);
//        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public double turnRate() {
        return imu.getRobotAngularVelocity(AngleUnit.DEGREES).yRotationRate; // NOTE: UNTESTED
    }

    public boolean isBetween(double value, double min, double max) {
        return value >= min && value <= max;
    }

    // Adapted from: https://github.com/gosu/gosu/blob/980d64de2ce52e4b16fdd5cb9c9e11c8bbb80671/src/Math.cpp#L38
    public double angleDiff(double from, double to) {
        double value = (to - from + 180);

        double fmod = (value - 0.0) % (360.0 - 0.0);

        return (fmod < 0 ? fmod + 360.0 : fmod +  0.0) - 180;
    }

    public double lerp(double min, double max, double t)
    {
        return min + (max - min) * t;
    }
}
