package dev.cyberarm.minibots.pizza;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

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

    private CyberarmEngine engine;

    public PizzaMinibot(CyberarmEngine engine) {
        this.engine = engine;
        leftFront = new MotorEx(engine.hardwareMap, "leftFront");
        rightFront = new MotorEx(engine.hardwareMap, "rightFront");

        leftBack = new MotorEx(engine.hardwareMap, "leftBack");
        rightBack = new MotorEx(engine.hardwareMap, "rightBack");

        rightFront.motorEx.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.motorEx.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);

        double wheelRadius = 75.0 / 2; // mm
        double distancePerTick = Math.PI * wheelRadius * wheelRadius;
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
}
