package dev.cyberarm.minibots.pizza;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.timecrafters.TimeCraftersConfigurationTool.library.TimeCraftersConfiguration;

import dev.cyberarm.engine.V2.CyberarmEngine;

public class PizzaMinibot {
    public static double GRIPPER_CLOSED = 0.333; // ~90 degrees
    public static double GRIPPER_OPEN = 0.75; // ~205 degrees

    public static double ARM_COLLECT = 0.0; // ~? degrees
    public static double ARM_PRECOLLECT = 0.05; // ~? degrees
    public static double ARM_DELIVER = 0.28; // ~? degrees
    public static double ARM_STOW = 0.72; // ~? degrees
    public static double ARM_HOVER_5_STACK = 0.10;
    public static double ARM_HOVER_4_STACK = 0.08;
    public static double ARM_HOVER_3_STACK = 0.07;
    public static double ARM_HOVER_2_STACK = 0.06;
    public static double ARM_HOVER_1_STACK = ARM_PRECOLLECT;
    public MotorEx leftFront, rightFront, leftBack, rightBack;
    public MotorGroup left, right;
    public IMU imu;
    public Servo gripper, arm;
    public int armStackPosition = -1;
    final public TimeCraftersConfiguration config;
    final public double wheelCircumference, distancePerTick;

    public final double imuAngleOffset, initialFacing;

    private CyberarmEngine engine;

    public PizzaMinibot(CyberarmEngine engine) {
        this.engine = engine;

        this.config = new TimeCraftersConfiguration("Pizza_2023");

        leftFront = new MotorEx(engine.hardwareMap, "leftFront");
        rightFront = new MotorEx(engine.hardwareMap, "rightFront");

        leftBack = new MotorEx(engine.hardwareMap, "leftBack");
        rightBack = new MotorEx(engine.hardwareMap, "rightBack");

        rightFront.motorEx.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.motorEx.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        wheelCircumference = Math.PI * (75.0 * 4); //wheelRadius * wheelRadius; //-- times 2 is a hack... --//
        distancePerTick = (28 * 20) / wheelCircumference; // raw motor encoder * gear ratio

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
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

        imu.initialize(parameters);

        imuAngleOffset = 0;
        initialFacing = facing();

        gripper = engine.hardwareMap.servo.get("gripper");
        arm = engine.hardwareMap.servo.get("arm");

        gripper.setPosition(PizzaMinibot.GRIPPER_CLOSED);
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
        engine.telemetry.addData("Gripper", gripper.getPosition());
        engine.telemetry.addData("Arm", gripper.getPosition());
        engine.telemetry.addLine();
        engine.telemetry.addData("Arm Stack Position", armStackPosition);

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
