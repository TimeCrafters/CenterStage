package dev.cyberarm.minibots.red_crab;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.timecrafters.TimeCraftersConfigurationTool.library.TimeCraftersConfiguration;
import org.timecrafters.TimeCraftersConfigurationTool.library.backend.config.Action;
import org.timecrafters.TimeCraftersConfigurationTool.library.backend.config.Variable;

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
    public static double DRIVETRAIN_MAX_SPEED = 0.5;
    public static double DRIVETRAIN_VELOCITY_MAX_MM = 610;
    public static double DRIVETRAIN_VELOCITY_SLOW_MM = 250;
    public static double DRIVETRAIN_GEAR_RATIO = 13.0321;
    public static int DRIVETRAIN_MOTOR_TICKS_PER_REVOLUTION = 28;
    public static double DRIVETRAIN_WHEEL_DIAMETER_MM = 90.0;

    public static double CLAW_ARM_MAX_SPEED = 0.5;
    public static double CLAW_ARM_MAX_VELOCITY_DEGREES = 10;
    public static double CLAW_ARM_kP = 0.0;
    public static double CLAW_ARM_kI = 0.0;
    public static double CLAW_ARM_kD = 0.0;
    public static double CLAW_ARM_kF = 0.0;
    public static double CLAW_ARM_kPosP = 0.0;
    public static int CLAW_ARM_POSITION_TOLERANCE = 1;
    public static double CLAW_ARM_STOW_ANGLE = 45.0; // 45.0
    public static double CLAW_ARM_DEPOSIT_ANGLE = 130.0; // 110.0
    public static double CLAW_ARM_COLLECT_FLOAT_ANGLE = 180.0;
    public static double CLAW_ARM_COLLECT_ANGLE = 200.0;

    public static final double WINCH_MAX_SPEED = 0.5;

    public static double CLAW_WRIST_STOW_POSITION = 0.7;
    public static double CLAW_WRIST_DEPOSIT_POSITION = 0.64;
    public static double CLAW_WRIST_COLLECT_FLOAT_POSITION = 0.64;
    public static double CLAW_WRIST_COLLECT_POSITION = 0.64;

    public static double CLAW_LEFT_CLOSED_POSITION = 0.2;
    public static double CLAW_LEFT_OPEN_POSITION = 0.5;
    public static double CLAW_RIGHT_CLOSED_POSITION = 0.77;
    public static double CLAW_RIGHT_OPEN_POSITION = 0.5;

    public static double DRONE_LATCH_INITIAL_POSITION = 0.5;
    public static double DRONE_LATCH_LAUNCH_POSITION = 0.7;
    public static int DRONE_LAUNCH_CONFIRMATION_TIME_MS = 1_000;

    public static double HOOK_ARM_STOW_POSITION = 0.8; // just off of airplane 0.8
    public static double HOOK_ARM_UP_POSITION = 0.4; // streight up4.0

    /// MOTOR CONSTANTS ///
    public static int CLAW_ARM_MOTOR_TICKS_PER_REVOLUTION = 4;
    public static double CLAW_ARM_MOTOR_GEAR_RATIO = 80; // Technically 72, but there is a lot of slop

    /// HARDWARE ///
    public final IMU imu;
    public final MotorEx frontLeft, frontRight, backLeft, backRight, winch;
    public final DcMotorEx clawArm;
    public final Servo leftClaw, rightClaw, clawWrist, droneLatch, hookArm;

    public final MotorGroup left, right;

    final CyberarmEngine engine;

    public TimeCraftersConfiguration config;
    private final PIDFController clawArmPIDFController;
    public final String webcamName = "Webcam 1";

    public enum Path {
        LEFT,
        CENTER,
        RIGHT
    }

    /* --- VisionProcessors --- */
    /// Tensorflow Lite Pixel detector.
    /// NOTE: detects april tags as pixels, use with caution!
    public TfodProcessor tfPixel = null;
    /// TeamProp detector: using OpenCV for subframe saturation threshold detection.
    public TeamPropVisionProcessor teamProp = null;
    /// Spike Mark detector: using OpenCV for full frame saturation threshold detection.
    public SpikeMarkDetectorVisionProcessor spikeMark = null;
    /// April Tag detector: Untested
    public AprilTagProcessor aprilTag = null;
    /// Doohickey
    public VisionPortal visionPortal = null;

    public RedCrabMinibot(boolean autonomous) {
        engine = CyberarmEngine.instance;

        config = new TimeCraftersConfiguration("cyberarm_RedCrab");
        loadConstants();

        /// IMU ///
        /// ------------------------------------------------------------------------------------ ///
        imu = engine.hardwareMap.get(IMU.class, "imu"); // | Ctrl Hub, I2C Port: 0
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        if (autonomous) {
            imu.resetYaw();
        }

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

        /// --- MOTOR GROUPS
        left = new MotorGroup(frontLeft, backLeft);
        right = new MotorGroup(frontRight, backRight);

        /// --- MOTOR DISTANCE PER TICK
        double distancePerTick = Utilities.ticksToUnit(
                DRIVETRAIN_MOTOR_TICKS_PER_REVOLUTION,
                DRIVETRAIN_GEAR_RATIO,
                DRIVETRAIN_WHEEL_DIAMETER_MM,
                DistanceUnit.MM,
                1);

        frontLeft.setDistancePerPulse(distancePerTick);
        frontRight.setDistancePerPulse(distancePerTick);
        backLeft.setDistancePerPulse(distancePerTick);
        backRight.setDistancePerPulse(distancePerTick);

        /// --- RUN MODE
        frontLeft.setRunMode(Motor.RunMode.VelocityControl);
        frontRight.setRunMode(Motor.RunMode.VelocityControl);
        backLeft.setRunMode(Motor.RunMode.VelocityControl);
        backRight.setRunMode(Motor.RunMode.VelocityControl);

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
        clawArmPIDFController = new PIDFController(0.4, 0.01, 0.1, 0.0);
        clawArm = (DcMotorEx) engine.hardwareMap.dcMotor.get("clawArm"); //  | Ctrl|Ex Hub, Port: ??
        clawWrist = engine.hardwareMap.servo.get("clawWrist");   //  | Ctrl|Ex Hub, Port: ??
        leftClaw = engine.hardwareMap.servo.get("leftClaw");     //  | Ctrl|Ex Hub, Port: ??
        rightClaw = engine.hardwareMap.servo.get("rightClaw");   //  | Ctrl|Ex Hub, Port: ??

        /// --- Claw Arm Motor
        /// --- --- (SOFT) RESET MOTOR ENCODER
        clawArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        /// --- --- DIRECTION
        clawArm.setDirection(DcMotorSimple.Direction.FORWARD);
        /// --- --- BRAKING
        /// --- --- NOTE: This won't hold back much, if anything, but its a small help, maybe? 😃
        clawArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /// --- --- Run Mode
        clawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        clawArm.setTargetPositionTolerance(CLAW_ARM_POSITION_TOLERANCE);
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
        droneLatch = engine.hardwareMap.servo.get("droneLatch"); //  | Ctrl|Ex Hub, Port: ??
        droneLatch.setDirection(Servo.Direction.FORWARD);
        droneLatch.setPosition(DRONE_LATCH_INITIAL_POSITION);

        /// HOOK ARM ///
        hookArm = engine.hardwareMap.servo.get("hookArm"); //  | Ctrl|Ex Hub, Port: ??
        hookArm.setDirection(Servo.Direction.FORWARD);
//        hookArm.setPosition(HOOK_ARM_STOW_POSITION); // LEAVE OFF:

        // Bulk read from hubs
        Utilities.hubsBulkReadMode(engine.hardwareMap, LynxModule.BulkCachingMode.MANUAL);
    }

    public void reloadConfig() {
        config = new TimeCraftersConfiguration("cyberarm_RedCrab");

        loadConstants();
    }

    private void loadConstants() {
        /// Drivetrain
        RedCrabMinibot.DRIVETRAIN_MAX_SPEED = config.variable("Robot", "Drivetrain_Tuning", "max_speed").value();
        RedCrabMinibot.DRIVETRAIN_VELOCITY_MAX_MM = config.variable("Robot", "Drivetrain_Tuning", "velocity_max_in_mm").value();
        RedCrabMinibot.DRIVETRAIN_VELOCITY_SLOW_MM = config.variable("Robot", "Drivetrain_Tuning", "velocity_slow_in_mm").value();
        RedCrabMinibot.DRIVETRAIN_GEAR_RATIO = config.variable("Robot", "Drivetrain_Tuning", "gear_ratio").value();
        RedCrabMinibot.DRIVETRAIN_MOTOR_TICKS_PER_REVOLUTION = config.variable("Robot", "Drivetrain_Tuning", "motor_ticks").value();
        RedCrabMinibot.DRIVETRAIN_WHEEL_DIAMETER_MM = config.variable("Robot", "Drivetrain_Tuning", "wheel_diameter_mm").value();

        /// CLAW ARM
        RedCrabMinibot.CLAW_ARM_MAX_SPEED = config.variable("Robot", "ClawArm_Tuning", "max_speed").value();
        RedCrabMinibot.CLAW_ARM_MAX_VELOCITY_DEGREES = config.variable("Robot", "ClawArm_Tuning", "max_velocityDEGREES").value();
        RedCrabMinibot.CLAW_ARM_POSITION_TOLERANCE = config.variable("Robot", "ClawArm_Tuning", "tolerance").value();
        RedCrabMinibot.CLAW_ARM_STOW_ANGLE = config.variable("Robot", "ClawArm_Tuning", "stow_angle").value();
        RedCrabMinibot.CLAW_ARM_DEPOSIT_ANGLE = config.variable("Robot", "ClawArm_Tuning", "collect_float_angle").value();
        RedCrabMinibot.CLAW_ARM_COLLECT_FLOAT_ANGLE = config.variable("Robot", "ClawArm_Tuning", "collect_angle").value();
        RedCrabMinibot.CLAW_ARM_COLLECT_ANGLE = config.variable("Robot", "ClawArm_Tuning", "collect_angle").value();

        RedCrabMinibot.CLAW_ARM_MOTOR_GEAR_RATIO = config.variable("Robot", "ClawArm_Tuning", "gear_ratio").value();
        RedCrabMinibot.CLAW_ARM_MOTOR_TICKS_PER_REVOLUTION = config.variable("Robot", "ClawArm_Tuning", "motor_ticks").value();

        /// CLAW WRIST
        RedCrabMinibot.CLAW_WRIST_STOW_POSITION = config.variable("Robot", "ClawWrist_Tuning", "stow_position").value();
        RedCrabMinibot.CLAW_WRIST_DEPOSIT_POSITION = config.variable("Robot", "ClawWrist_Tuning", "deposit_position").value();
        RedCrabMinibot.CLAW_WRIST_COLLECT_FLOAT_POSITION = config.variable("Robot", "ClawWrist_Tuning", "collect_float_position").value();
        RedCrabMinibot.CLAW_WRIST_COLLECT_POSITION = config.variable("Robot", "ClawWrist_Tuning", "collect_position").value();

        /// CLAWS
        RedCrabMinibot.CLAW_LEFT_CLOSED_POSITION = config.variable("Robot", "Claw_Tuning", "claw_left_closed_position").value();
        RedCrabMinibot.CLAW_LEFT_OPEN_POSITION = config.variable("Robot", "Claw_Tuning", "claw_left_open_position").value();
        RedCrabMinibot.CLAW_RIGHT_CLOSED_POSITION = config.variable("Robot", "Claw_Tuning", "claw_right_closed_position").value();
        RedCrabMinibot.CLAW_RIGHT_OPEN_POSITION = config.variable("Robot", "Claw_Tuning", "claw_right_open_position").value();

        /// HOOK ARM
        RedCrabMinibot.HOOK_ARM_STOW_POSITION = config.variable("Robot", "HookArm_Tuning", "stow_position").value();
        RedCrabMinibot.HOOK_ARM_UP_POSITION = config.variable("Robot", "HookArm_Tuning", "up_position").value();

        /// DRONE LATCH
        RedCrabMinibot.DRONE_LATCH_LAUNCH_POSITION = config.variable("Robot", "DroneLauncher_Tuning", "initial_position").value();
        RedCrabMinibot.DRONE_LATCH_INITIAL_POSITION = config.variable("Robot", "DroneLauncher_Tuning", "launch_position").value();
        RedCrabMinibot.DRONE_LAUNCH_CONFIRMATION_TIME_MS = config.variable("Robot", "DroneLauncher_Tuning", "launch_confirmation_time_ms").value();
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
                "Power: %.2f, Current: %.2f mAmp, Position: %d, Velocity: %.2f",
                clawArm.getPower(),
                clawArm.getCurrent(CurrentUnit.MILLIAMPS),
                clawArm.getCurrentPosition(),
                clawArm.getVelocity());
        engine.telemetry.addData(
                "   PIDF", "P: %.4f, I: %.4f, D: %.4f, F: %.4f",
                clawArmPIDFController.getP(),
                clawArmPIDFController.getI(),
                clawArmPIDFController.getD(),
                clawArmPIDFController.getF());
        engine.telemetry.addData(
                "   PIDF+", "PosError: %.4f, velError: %.4f, Period: %.4f",
                clawArmPIDFController.getPositionError(),
                clawArmPIDFController.getVelocityError(),
                clawArmPIDFController.getPeriod());

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

    public void controlClawArm() {
        Action action = config.action("Robot", "ClawArm_Tuning");

        double ticksInDegree = Utilities.motorAngleToTicks(CLAW_ARM_MOTOR_TICKS_PER_REVOLUTION, CLAW_ARM_MOTOR_GEAR_RATIO, 1);

        for (Variable v : action.getVariables()) {
            switch (v.name.trim()) {
                case "kP": // Proportional
                    CLAW_ARM_kP = v.value();
                    break;
                case "kI": // Integral
                    CLAW_ARM_kI = v.value();
                    break;
                case "kD": // Derivative (Dampener)
                    CLAW_ARM_kD = v.value();
                    break;
                case "kF": // Feedforward
                    CLAW_ARM_kF = v.value();
                    break;
                case "KPosP": // Positional Proportional
                    CLAW_ARM_kPosP = v.value();
                    break;
            }
        }

//        int armPos = clawArm.getCurrentPosition();
//        clawArmPIDFController.setTolerance(CLAW_ARM_POSITION_TOLERANCE);
//        clawArmPIDFController.setPIDF(CLAW_ARM_kP, CLAW_ARM_kI, CLAW_ARM_kD, CLAW_ARM_kF);
//        double pidf = clawArmPIDFController.calculate(armPos, clawArm.getTargetPosition());
//        double ff = Math.cos(Math.toRadians(clawArm.getTargetPosition() / ticksInDegree)) * f;

        // Limit pidf's value to max power range
//        double power = Range.clip(pidf, -CLAW_ARM_MAX_SPEED, CLAW_ARM_MAX_SPEED);

        clawArm.setVelocityPIDFCoefficients(CLAW_ARM_kP, CLAW_ARM_kI, CLAW_ARM_kD, CLAW_ARM_kF);
        clawArm.setPositionPIDFCoefficients(CLAW_ARM_kPosP);

        double velocity = Utilities.motorAngleToTicks(CLAW_ARM_MOTOR_TICKS_PER_REVOLUTION, CLAW_ARM_MOTOR_GEAR_RATIO, CLAW_ARM_MAX_VELOCITY_DEGREES);

        clawArm.setVelocity(velocity);
    }
}
