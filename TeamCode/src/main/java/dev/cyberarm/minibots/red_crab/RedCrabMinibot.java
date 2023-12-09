package dev.cyberarm.minibots.red_crab;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import dev.cyberarm.engine.V2.CyberarmEngine;
import dev.cyberarm.engine.V2.Utilities;

public class RedCrabMinibot {
    /// CLAW ARM ///
    public static final int ClawArm_INITIAL = -1; // NO OP match starting position
    public static final int ClawArm_STOW = 0;
    public static final int ClawArm_DEPOSIT = 1;
    public static final int ClawArm_COLLECT_FLOAT = 2;
    public static final int ClawArm_COLLECT = 3;

    /// TUNING CONSTANTS ///
    public static final double DRIVETRAIN_MAX_SPEED = 0.5;
    public static final double CLAW_ARM_MAX_SPEED = 0.5;
    public static final double CLAW_ARM_kP = 0.025;
    public static final double CLAW_ARM_POSITION_TOLERANCE = 3.3;
    public static final double WINCH_MAX_SPEED = 0.5;
    public static final double CLAW_ARM_STOW_ANGLE = 45.0; // 45.0
    public static final double CLAW_ARM_DEPOSIT_ANGLE = 130.0; // 110.0
    public static final double CLAW_ARM_COLLECT_FLOAT_ANGLE = 180.0;
    public static final double CLAW_ARM_COLLECT_ANGLE = 200.0;

    public static final double CLAW_WRIST_STOW_POSITION = 0.5;
    public static final double CLAW_WRIST_DEPOSIT_POSITION = 0.64;
    public static final double CLAW_WRIST_COLLECT_FLOAT_POSITION = 0.64;
    public static final double CLAW_WRIST_COLLECT_POSITION = 0.64;

    public static final double CLAW_LEFT_CLOSED_POSITION = 0.2;
    public static final double CLAW_LEFT_OPEN_POSITION = 0.5;
    public static final double CLAW_RIGHT_CLOSED_POSITION = 0.77;
    public static final double CLAW_RIGHT_OPEN_POSITION = 0.5;

    public static final double DRONE_LATCH_INITIAL_POSITION = 0.5;
    public static final double DRONE_LATCH_LAUNCH_POSITION = 0.7;
    public static final int DRONE_LAUNCH_CONFIRMATION_TIME_MS = 1_000;

    public static final double HOOK_ARM_STOW_POSITION = 0.8; // just off of airplane 0.8
    public static final double HOOK_ARM_UP_POSITION = 0.4; // streight up4.0

    /// MOTOR CONSTANTS ///
    public static final int CLAW_ARM_MOTOR_TICKS_PER_REVOLUTION = 4;
    public static final double CLAW_ARM_MOTOR_GEAR_RATIO = 72;

    /// HARDWARE ///
    public final IMU imu;
    public final MotorEx frontLeft, frontRight, backLeft, backRight, winch, clawArm;
    public final Servo leftClaw, rightClaw, clawWrist, droneLatch, hookArm;

    final CyberarmEngine engine;

    public RedCrabMinibot() {
        engine = CyberarmEngine.instance;

        /// IMU ///
        /// ------------------------------------------------------------------------------------ ///
        imu = engine.hardwareMap.get(IMU.class, "imu"); // | Ctrl Hub, I2C Port: 0
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        /// DRIVE TRAIN ///
        /// ------------------------------------------------------------------------------------ ///
        frontLeft = new MotorEx(engine.hardwareMap, "frontLeft"); // | Ctrl|Ex Hub, Port: ??
        frontRight = new MotorEx(engine.hardwareMap, "frontRight"); // | Ctrl|Ex Hub, Port: ??
        backLeft = new MotorEx(engine.hardwareMap, "backLeft"); // | Ctrl|Ex Hub, Port: ??
        backRight = new MotorEx(engine.hardwareMap, "backRight"); // | Ctrl|Ex Hub, Port: ??

        /// --- (SOFT) RESET MOTOR ENCODERS
        frontLeft.resetEncoder();
        frontRight.resetEncoder();
        backLeft.resetEncoder();
        backRight.resetEncoder();

        /// --- MOTOR DIRECTIONS
        frontLeft.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.motorEx.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.motorEx.setDirection(DcMotorSimple.Direction.FORWARD);

        /// --- MOTOR BRAKING MODE
        /// --- NOTE: Having BRAKE mode set for drivetrain helps with consistently of control
        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior((Motor.ZeroPowerBehavior.BRAKE));

        /// WINCH ///
        /// ------------------------------------------------------------------------------------ ///
        winch = new MotorEx(engine.hardwareMap, "winch"); // | Ctrl|Ex Hub, Port: ??

        /// --- (SOFT) MOTOR ENCODER RESET
        winch.resetEncoder();

        /// --- MOTOR DIRECTION
        /// --- NOTE: Unknown if FORWARD or REVERSE is correct
        winch.motorEx.setDirection(DcMotorSimple.Direction.FORWARD);

        /// CLAW and Co. ///
        /// ------------------------------------------------------------------------------------ ///
        clawArm = new MotorEx(engine.hardwareMap, "clawArm"); //  | Ctrl|Ex Hub, Port: ??
        clawWrist = engine.hardwareMap.servo.get("clawWrist");   //  | Ctrl|Ex Hub, Port: ??
        leftClaw = engine.hardwareMap.servo.get("leftClaw");     //  | Ctrl|Ex Hub, Port: ??
        rightClaw = engine.hardwareMap.servo.get("rightClaw");   //  | Ctrl|Ex Hub, Port: ??

        /// --- Claw Arm Motor
        /// --- --- (SOFT) RESET MOTOR ENCODER
        clawArm.resetEncoder();
        /// --- --- DIRECTION
        clawArm.motorEx.setDirection(DcMotorSimple.Direction.FORWARD);
        /// --- --- BRAKING
        /// --- --- NOTE: This won't hold back much, if anything, but its a small help, maybe? 😃
        clawArm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        /// --- --- Run Mode
        clawArm.setRunMode(Motor.RunMode.PositionControl);
        clawArm.setPositionCoefficient(CLAW_ARM_kP);
        clawArm.setPositionTolerance(CLAW_ARM_POSITION_TOLERANCE);
        clawArm.setTargetPosition(0);

        /// --- Claws
        /// --- --- Wrist
        clawWrist.setDirection(Servo.Direction.FORWARD);
        clawWrist.setPosition(CLAW_WRIST_STOW_POSITION);
        /// --- --- Left
        leftClaw.setDirection(Servo.Direction.FORWARD);
        leftClaw.setPosition((CLAW_LEFT_CLOSED_POSITION));
        /// --- --- Right
        rightClaw.setDirection(Servo.Direction.FORWARD);
        rightClaw.setPosition((CLAW_RIGHT_CLOSED_POSITION));

        /// DRONE LATCH ///
        droneLatch = engine.hardwareMap.servo.get("droneLatch");
        droneLatch.setDirection(Servo.Direction.FORWARD);
        droneLatch.setPosition(DRONE_LATCH_INITIAL_POSITION);

        /// HOOK ARM ///
        hookArm = engine.hardwareMap.servo.get("hookArm");
        hookArm.setDirection(Servo.Direction.FORWARD);
//        hookArm.setPosition(HOOK_ARM_STOW_POSITION); // LEAVE OFF:
    }

    public void standardTelemetry() {
        engine.telemetry.addLine();

        engine.telemetry.addLine("Motors");
        engine.telemetry.addData(
                "Front Left",
                "Power: %.2f, Current: %.2f mAmp, Position: %d",
                frontLeft.motorEx.getPower(),
                frontLeft.motorEx.getCurrent(CurrentUnit.MILLIAMPS),
                frontLeft.motorEx.getCurrentPosition());
        engine.telemetry.addData(
                "Front Right",
                "Power: %.2f, Current: %.2f mAmp, Position: %d",
                frontRight.motorEx.getPower(),
                frontRight.motorEx.getCurrent(CurrentUnit.MILLIAMPS),
                frontRight.motorEx.getCurrentPosition());
        engine.telemetry.addData(
                "Back Left",
                "Power: %.2f, Current: %.2f mAmp, Position: %d",
                backLeft.motorEx.getPower(),
                backLeft.motorEx.getCurrent(CurrentUnit.MILLIAMPS),
                backLeft.motorEx.getCurrentPosition());
        engine.telemetry.addData(
                "Back Right",
                "Power: %.2f, Current: %.2f mAmp, Position: %d",
                backRight.motorEx.getPower(),
                backRight.motorEx.getCurrent(CurrentUnit.MILLIAMPS),
                backRight.motorEx.getCurrentPosition());

        engine.telemetry.addLine();
        engine.telemetry.addData(
                "Winch",
                "Power: %.2f, Current: %.2f mAmp, Position: %d",
                winch.motorEx.getPower(),
                winch.motorEx.getCurrent(CurrentUnit.MILLIAMPS),
                winch.motorEx.getCurrentPosition());
        engine.telemetry.addLine();

        engine.telemetry.addData(
                "Claw Arm",
                "Power: %.2f, Current: %.2f mAmp, Position: %d",
                clawArm.motorEx.getPower(),
                clawArm.motorEx.getCurrent(CurrentUnit.MILLIAMPS),
                clawArm.motorEx.getCurrentPosition());

        engine.telemetry.addLine();
        engine.telemetry.addLine("Servos");
        engine.telemetry.addData("Claw Wrist", clawWrist.getPosition());
        engine.telemetry.addData("Left Claw", leftClaw.getPosition());
        engine.telemetry.addData("Right Claw", rightClaw.getPosition());
        engine.telemetry.addLine();
        engine.telemetry.addData("Drone Latch", droneLatch.getPosition());
        engine.telemetry.addLine();
        engine.telemetry.addData("Hook Arm", hookArm.getPosition());
        engine.telemetry.addLine();
        engine.telemetry.addData("IMU RAW Heading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        engine.telemetry.addData("IMU Facing Degree", Utilities.facing(imu));

        engine.telemetry.addLine();
    }
}
