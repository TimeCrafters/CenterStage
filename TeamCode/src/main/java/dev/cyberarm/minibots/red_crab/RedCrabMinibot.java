package dev.cyberarm.minibots.red_crab;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.timecrafters.TimeCraftersConfigurationTool.library.TimeCraftersConfiguration;
import org.timecrafters.TimeCraftersConfigurationTool.library.backend.config.Action;
import org.timecrafters.TimeCraftersConfigurationTool.library.backend.config.Variable;

import dev.cyberarm.drivers.EncoderCustomKB2040;
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
    private static double CLAW_ARM_MOTOR_MAX_CURRENT_MILLIAMPS = 1588.0;
    private static long CLAW_ARM_WARN_OVERCURRENT_AFTER_MS = 5000;
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

    public static double WINCH_MAX_SPEED = 0.5;

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
    public final DcMotorEx frontLeft, frontRight, backLeft, backRight, winch;
    public final DcMotorEx clawArm;
    public final Servo leftClaw, rightClaw, clawWrist, droneLatch, hookArm;
    public final DcMotorEx deadWheelXLeft, deadWheelXRight;
    public final EncoderCustomKB2040 deadWheelYCenter;

    final CyberarmEngine engine;

    public TimeCraftersConfiguration config;
    private final PIDFController clawArmPIDFController;
    public final String webcamName = "Webcam 1";

    private long lastClawArmOverCurrentAnnounced = 0;
    private boolean clawArmOverCurrent = false;
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

    public static Localizer localizer;

    public RedCrabMinibot(boolean autonomous) {
        engine = CyberarmEngine.instance;

        config = new TimeCraftersConfiguration("cyberarm_RedCrab");
        loadConstants();

        /// IMU ///
        /// ------------------------------------------------------------------------------------ ///
        imu = engine.hardwareMap.get(IMU.class, "imu"); // | Control Hub, I2C Port: 0
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
        frontLeft = (DcMotorEx) engine.hardwareMap.dcMotor.get("frontLeft");   // | Expansion Hub, Motor Port: 2
        frontRight = (DcMotorEx) engine.hardwareMap.dcMotor.get("frontRight"); // | Expansion Hub, Motor Port: 3
        backLeft = (DcMotorEx) engine.hardwareMap.dcMotor.get("backLeft");     // | Expansion Hub, Motor Port: 3
        backRight = (DcMotorEx) engine.hardwareMap.dcMotor.get("backRight");   // | Expansion Hub, Motor Port: 2

        /// --- RESET MOTOR ENCODERS
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /// --- MOTOR DIRECTIONS
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        /// --- MOTOR BRAKING MODE
        /// --- NOTE: Having BRAKE mode set for drivetrain helps with consistently of control
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /// --- MOTOR DISTANCE PER TICK
        double distancePerTick = Utilities.ticksToUnit(
                DRIVETRAIN_MOTOR_TICKS_PER_REVOLUTION,
                DRIVETRAIN_GEAR_RATIO,
                DRIVETRAIN_WHEEL_DIAMETER_MM,
                DistanceUnit.MM,
                1);

        /// --- RUN MODE
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /// WINCH ///
        /// ------------------------------------------------------------------------------------ ///
        winch = (DcMotorEx) engine.hardwareMap.dcMotor.get("winch"); // | Expansion Hub, Motor Port: 0

        /// --- MOTOR ENCODER RESET
        winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /// --- MOTOR DIRECTION
        /// --- NOTE: Unknown if FORWARD or REVERSE is correct
        winch.setDirection(DcMotorSimple.Direction.FORWARD);

        /// --- RUN MODE
        winch.setTargetPosition(winch.getCurrentPosition());
        winch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /// CLAW and Co. ///
        /// ------------------------------------------------------------------------------------ ///
        clawArmPIDFController = new PIDFController(0.4, 0.01, 0.1, 0.0);
        clawArm = (DcMotorEx) engine.hardwareMap.dcMotor.get("clawArm"); //  | Expansion Hub, Motor Port: 1
        clawWrist = engine.hardwareMap.servo.get("clawWrist");           //  | Control Hub, Servo Port: 0
        leftClaw = engine.hardwareMap.servo.get("leftClaw");             //  | Control Hub, Servo Port: 2
        rightClaw = engine.hardwareMap.servo.get("rightClaw");           //  | Control Hub, Servo Port: 1

        /// --- Claw Arm Motor
        /// --- --- (SOFT) RESET MOTOR ENCODER
        // ONLY RESET ENCODER IN AUTONOMOUS
        if (autonomous)
            clawArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        /// --- --- DIRECTION
        clawArm.setDirection(DcMotorSimple.Direction.REVERSE);
        /// --- --- BRAKING
        /// --- --- NOTE: This won't hold back much, if anything, but its a small help, maybe? 😃
        clawArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /// --- --- Run Mode
        clawArm.setTargetPosition(0);
        clawArm.setTargetPositionTolerance(CLAW_ARM_POSITION_TOLERANCE);
        clawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
        droneLatch = engine.hardwareMap.servo.get("droneLatch"); //  | Expansion Hub, Servo Port: 0
        droneLatch.setDirection(Servo.Direction.FORWARD);
        droneLatch.setPosition(DRONE_LATCH_INITIAL_POSITION);

        /// HOOK ARM ///
        hookArm = engine.hardwareMap.servo.get("hookArm"); //  | Control Hub, Servo Port: 3
        hookArm.setDirection(Servo.Direction.FORWARD);
//        hookArm.setPosition(HOOK_ARM_STOW_POSITION); // LEAVE OFF:

        /// DEAD WHEELS ///
        deadWheelXLeft = (DcMotorEx) engine.hardwareMap.dcMotor.get("deadwheel_x_left");
        deadWheelXRight = (DcMotorEx) engine.hardwareMap.dcMotor.get("deadwheel_x_right");
        deadWheelYCenter = engine.hardwareMap.get(EncoderCustomKB2040.class, "deadwheel_y_center");

        deadWheelXLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        deadWheelXRight.setDirection(DcMotorSimple.Direction.FORWARD);
//        deadWheelYCenter.setDirection(DcMotorSimple.Direction.FORWARD);

        // Bulk read from hubs
        Utilities.hubsBulkReadMode(engine.hardwareMap, LynxModule.BulkCachingMode.MANUAL);

        // Initialize localizer
        if (autonomous || RedCrabMinibot.localizer == null) {
            RedCrabMinibot.localizer = new Localizer(this);
        }

        if (autonomous)
            RedCrabMinibot.localizer.reset();
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
        RedCrabMinibot.CLAW_ARM_DEPOSIT_ANGLE = config.variable("Robot", "ClawArm_Tuning", "deposit_angle").value();
        RedCrabMinibot.CLAW_ARM_COLLECT_FLOAT_ANGLE = config.variable("Robot", "ClawArm_Tuning", "collect_float_angle").value();
        RedCrabMinibot.CLAW_ARM_COLLECT_ANGLE = config.variable("Robot", "ClawArm_Tuning", "collect_angle").value();

        RedCrabMinibot.CLAW_ARM_MOTOR_GEAR_RATIO = config.variable("Robot", "ClawArm_Tuning", "gear_ratio").value();
        RedCrabMinibot.CLAW_ARM_MOTOR_TICKS_PER_REVOLUTION = config.variable("Robot", "ClawArm_Tuning", "motor_ticks").value();
        RedCrabMinibot.CLAW_ARM_MOTOR_MAX_CURRENT_MILLIAMPS = config.variable("Robot", "ClawArm_Tuning", "max_current_milliamps").value();
        RedCrabMinibot.CLAW_ARM_WARN_OVERCURRENT_AFTER_MS = config.variable("Robot", "ClawArm_Tuning", "warn_overcurrent_after_ms").value();

        /// WINCH

        RedCrabMinibot.WINCH_MAX_SPEED = config.variable("Robot", "Winch_Tuning", "max_speed").value();

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
        RedCrabMinibot.DRONE_LATCH_INITIAL_POSITION = config.variable("Robot", "DroneLauncher_Tuning", "initial_position").value();
        RedCrabMinibot.DRONE_LATCH_LAUNCH_POSITION = config.variable("Robot", "DroneLauncher_Tuning", "launch_position").value();
        RedCrabMinibot.DRONE_LAUNCH_CONFIRMATION_TIME_MS = config.variable("Robot", "DroneLauncher_Tuning", "launch_confirmation_time_ms").value();
    }

    public void standardTelemetry() {
        engine.telemetry.addLine();

        if (RedCrabMinibot.localizer != null) {
            engine.telemetry.addLine("Localizer");
            engine.telemetry.addData("X (MM)", "%.2fmm", RedCrabMinibot.localizer.xMM());
            engine.telemetry.addData("Y (MM)", "%.2fmm", RedCrabMinibot.localizer.yMM());
            engine.telemetry.addData("R (De)", "%.2fdeg", RedCrabMinibot.localizer.headingDegrees());
            engine.telemetry.addLine();
        }

        engine.telemetry.addLine("Deadwheels");
        engine.telemetry.addData("X Left", deadWheelXLeft.getCurrentPosition());
        engine.telemetry.addData("X Right", deadWheelXRight.getCurrentPosition());
        // Use .getLastPosition instead of .getCurrentPosition here to not make an additional round trip just for telemetry
        engine.telemetry.addData("Y Center", deadWheelYCenter.getLastPosition());
        engine.telemetry.addLine();

        engine.telemetry.addLine("Motors");
        engine.telemetry.addData(
                "Front Left",
                "Power: %.2f, Current: %.2f mAmp, Position: %d, Velocity: %.2f (%.2f mm/s)",
                frontLeft.getPower(),
                frontLeft.getCurrent(CurrentUnit.MILLIAMPS),
                frontLeft.getCurrentPosition(),
                frontLeft.getVelocity(),
                Utilities.ticksToUnit(
                        DRIVETRAIN_MOTOR_TICKS_PER_REVOLUTION,
                        DRIVETRAIN_GEAR_RATIO,
                        DRIVETRAIN_WHEEL_DIAMETER_MM,
                        DistanceUnit.MM,
                        (int)frontLeft.getVelocity()));
        engine.telemetry.addLine();
        engine.telemetry.addData(
                "Front Right",
                "Power: %.2f, Current: %.2f mAmp, Position: %d, Velocity: %.2f (%.2f mm/s)",
                frontRight.getPower(),
                frontRight.getCurrent(CurrentUnit.MILLIAMPS),
                frontRight.getCurrentPosition(),
                frontRight.getVelocity(),
                Utilities.ticksToUnit(
                        DRIVETRAIN_MOTOR_TICKS_PER_REVOLUTION,
                        DRIVETRAIN_GEAR_RATIO,
                        DRIVETRAIN_WHEEL_DIAMETER_MM,
                        DistanceUnit.MM,
                        (int)frontLeft.getVelocity()));
        engine.telemetry.addLine();
        engine.telemetry.addData(
                "Back Left",
                "Power: %.2f, Current: %.2f mAmp, Position: %d, Velocity: %.2f (%.2f mm/s)",
                backLeft.getPower(),
                backLeft.getCurrent(CurrentUnit.MILLIAMPS),
                backLeft.getCurrentPosition(),
                backLeft.getVelocity(),
                Utilities.ticksToUnit(
                        DRIVETRAIN_MOTOR_TICKS_PER_REVOLUTION,
                        DRIVETRAIN_GEAR_RATIO,
                        DRIVETRAIN_WHEEL_DIAMETER_MM,
                        DistanceUnit.MM,
                        (int)backLeft.getVelocity()));
        engine.telemetry.addLine();
        engine.telemetry.addData(
                "Back Right",
                "Power: %.2f, Current: %.2f mAmp, Position: %d, Velocity: %.2f (%.2f mm/s)",
                backRight.getPower(),
                backRight.getCurrent(CurrentUnit.MILLIAMPS),
                backRight.getCurrentPosition(),
                backRight.getVelocity(),
                Utilities.ticksToUnit(
                        DRIVETRAIN_MOTOR_TICKS_PER_REVOLUTION,
                        DRIVETRAIN_GEAR_RATIO,
                        DRIVETRAIN_WHEEL_DIAMETER_MM,
                        DistanceUnit.MM,
                        (int)backRight.getVelocity()));

        engine.telemetry.addLine();
        engine.telemetry.addLine();

        engine.telemetry.addData(
                "Winch",
                "Power: %.2f, Current: %.2f mAmp, Position: %d, Velocity: %.2f (%.2f mm/s)",
                winch.getPower(),
                winch.getCurrent(CurrentUnit.MILLIAMPS),
                winch.getCurrentPosition(),
                winch.getVelocity(),
                Utilities.ticksToUnit(
                        DRIVETRAIN_MOTOR_TICKS_PER_REVOLUTION,
                        DRIVETRAIN_GEAR_RATIO,
                        DRIVETRAIN_WHEEL_DIAMETER_MM,
                        DistanceUnit.MM,
                        (int)winch.getVelocity()));
        engine.telemetry.addLine();

        PIDFCoefficients clawArmPIDFPosition = clawArm.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        PIDFCoefficients clawArmPIDFEncoder = clawArm.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        int clawArmError = clawArm.getCurrentPosition() - clawArm.getTargetPosition();
        engine.telemetry.addData(
                "Claw Arm",
                "Power: %.2f, Current: %.2f mAmp, Position: %d, Angle: %.2f, Velocity: %.2f (%.2f degrees/s)",
                clawArm.getPower(),
                clawArm.getCurrent(CurrentUnit.MILLIAMPS),
                clawArm.getCurrentPosition(),
                Utilities.motorTicksToAngle(CLAW_ARM_MOTOR_TICKS_PER_REVOLUTION, CLAW_ARM_MOTOR_GEAR_RATIO, clawArm.getCurrentPosition()),
                clawArm.getVelocity(),
                Utilities.motorTicksToAngle(CLAW_ARM_MOTOR_TICKS_PER_REVOLUTION, CLAW_ARM_MOTOR_GEAR_RATIO, (int)clawArm.getVelocity()));
        engine.telemetry.addData(
                "Claw Arm",
                "TPos: %d, TAngle: %.2f, ErrPos: %d ErrAngle: %.2f",
                clawArm.getTargetPosition(),
                Utilities.motorTicksToAngle(CLAW_ARM_MOTOR_TICKS_PER_REVOLUTION, CLAW_ARM_MOTOR_GEAR_RATIO, clawArm.getTargetPosition()),
                clawArmError,
                Utilities.motorTicksToAngle(CLAW_ARM_MOTOR_TICKS_PER_REVOLUTION, CLAW_ARM_MOTOR_GEAR_RATIO, clawArmError),
                clawArm.getVelocity(),
                Utilities.motorTicksToAngle(CLAW_ARM_MOTOR_TICKS_PER_REVOLUTION, CLAW_ARM_MOTOR_GEAR_RATIO, (int)clawArm.getVelocity()));
        engine.telemetry.addData("Current Alarm?", clawArm.isOverCurrent());
        engine.telemetry.addData(
                "   PIDF", "P: %.4f, I: %.4f, D: %.4f, F: %.4f, PosP: %.4f",
                clawArmPIDFEncoder.p,
                clawArmPIDFEncoder.i,
                clawArmPIDFEncoder.d,
                clawArmPIDFEncoder.f,
                clawArmPIDFPosition.p);

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

        long milliseconds = System.currentTimeMillis();
        if (clawArm.isOverCurrent())
        {
            if (milliseconds - lastClawArmOverCurrentAnnounced >= CLAW_ARM_WARN_OVERCURRENT_AFTER_MS) {
                lastClawArmOverCurrentAnnounced = System.currentTimeMillis();

                engine.telemetry.speak("WARNING. ARM. OVER. CURRENT.");
            }
        } else {
            lastClawArmOverCurrentAnnounced = milliseconds;
        }

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

        clawArm.setCurrentAlert(CLAW_ARM_MOTOR_MAX_CURRENT_MILLIAMPS, CurrentUnit.MILLIAMPS);

        double velocity = Utilities.motorAngleToTicks(CLAW_ARM_MOTOR_TICKS_PER_REVOLUTION, CLAW_ARM_MOTOR_GEAR_RATIO, CLAW_ARM_MAX_VELOCITY_DEGREES);

        double currentAngle = Utilities.motorTicksToAngle(CLAW_ARM_MOTOR_TICKS_PER_REVOLUTION, CLAW_ARM_MOTOR_GEAR_RATIO, clawArm.getCurrentPosition());
        double targetAngle = Utilities.motorTicksToAngle(CLAW_ARM_MOTOR_TICKS_PER_REVOLUTION, CLAW_ARM_MOTOR_GEAR_RATIO, clawArm.getTargetPosition());
        double angleDiff = Math.abs(Utilities.angleDiff(currentAngle, targetAngle));

        // Turn off motor if it is stowed or all the way down
        if (targetAngle <= CLAW_ARM_STOW_ANGLE + 5.0 && angleDiff <= 5.0) {
            velocity = 0.0;
        } else if (targetAngle >= CLAW_ARM_COLLECT_ANGLE - 5.0 && angleDiff <= 5.0)
        {
            velocity = 0.0;
        }

        clawArm.setVelocity(velocity);
    }

    public double distanceMM(DcMotorEx motor) {
        return Utilities.ticksToUnit(
                RedCrabMinibot.DRIVETRAIN_MOTOR_TICKS_PER_REVOLUTION,
                RedCrabMinibot.DRIVETRAIN_GEAR_RATIO,
                RedCrabMinibot.DRIVETRAIN_WHEEL_DIAMETER_MM,
                DistanceUnit.MM,
                motor.getCurrentPosition()
        );
    }

    public boolean atTargetPosition(DcMotorEx motor, double travelledDistanceMM, double toleranceMM) {
        double distanceMM = distanceMM(motor);

        return Utilities.isBetween(distanceMM, travelledDistanceMM - toleranceMM, travelledDistanceMM + toleranceMM);
    }
}
