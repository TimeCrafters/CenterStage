package dev.cyberarm.minibots.yellow;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.timecrafters.TimeCraftersConfigurationTool.library.TimeCraftersConfiguration;

import dev.cyberarm.engine.V2.CyberarmEngine;

public class YellowMinibot {
    private final CyberarmEngine engine;
    public TimeCraftersConfiguration config;
    public final DcMotorEx backLeft, frontLeft, frontRight, backRight;
    public final IMU imu;
    public final Servo leftClaw, rightClaw, droneLatch;
    public final CRServo arm;
    public final TouchSensor armEndStopLeft, armEndStopRight;

    public double DRIVETRAIN_WHEEL_DIAMETER_MM = 90.0;
    public int    DRIVETRAIN_MOTOR_TICKS = 4;
    public double DRIVETRAIN_MOTOR_GEAR_RATIO = 72.0;
    public double DRIVETRAIN_MAX_VELOCITY_MM = 305.0;
    public double LEFT_CLAW_CLOSED_POSITION = 0.0;
    public double LEFT_CLAW_OPEN_POSITION = 0.0;
    public double RIGHT_CLAW_CLOSED_POSITION = 0.0;
    public double RIGHT_CLAW_OPEN_POSITION = 0.0;
    public double DRONE_LATCH_INITIAL_POSITION = 0.0;
    public double DRONE_LATCH_LAUNCH_POSITION = 0.0;

    public YellowMinibot(CyberarmEngine engine) {
        this.engine = engine;

        reloadConfig();

        backLeft = (DcMotorEx) engine.hardwareMap.dcMotor.get("backLeft");
        frontLeft = (DcMotorEx) engine.hardwareMap.dcMotor.get("frontLeft");
        frontRight = (DcMotorEx) engine.hardwareMap.dcMotor.get("frontRight");
        backRight = (DcMotorEx) engine.hardwareMap.dcMotor.get("backRight");

        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftClaw = engine.hardwareMap.servo.get("leftClaw"); /// Port 0
        rightClaw = engine.hardwareMap.servo.get("rightClaw"); /// Port 1
        arm = engine.hardwareMap.crservo.get("arm"); /// Port 2
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        droneLatch = engine.hardwareMap.servo.get("droneLatch"); /// Port 3

        armEndStopLeft = engine.hardwareMap.touchSensor.get("armEndStopLeft");
        armEndStopRight = engine.hardwareMap.touchSensor.get("armEndStopRight");

        imu = engine.hardwareMap.get(IMU.class, "imu"); // Embedded | IC2 port 0
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

        imu.initialize(parameters);
    }

    public void reloadConfig() {
        this.config = new TimeCraftersConfiguration("cyberarm_Vexy_Yellow");

        loadConstants();
    }

    public void loadConstants() {
        DRIVETRAIN_WHEEL_DIAMETER_MM = config.variable("Robot", "Drivetrain_Tuning", "wheel_diameter_mm").value();
        DRIVETRAIN_MOTOR_TICKS = config.variable("Robot", "Drivetrain_Tuning", "motor_ticks").value();
        DRIVETRAIN_MOTOR_GEAR_RATIO = config.variable("Robot", "Drivetrain_Tuning", "motor_gear_ratio").value();
        DRIVETRAIN_MAX_VELOCITY_MM = config.variable("Robot", "Drivetrain_Tuning", "max_velocity_mm").value();

        LEFT_CLAW_CLOSED_POSITION = config.variable("Robot", "LeftClaw_Tuning", "open_position").value();
        LEFT_CLAW_OPEN_POSITION = config.variable("Robot", "LeftClaw_Tuning", "closed_position").value();


        RIGHT_CLAW_OPEN_POSITION = config.variable("Robot", "RightClaw_Tuning", "open_position").value();
        RIGHT_CLAW_CLOSED_POSITION = config.variable("Robot", "RightClaw_Tuning", "closed_position").value();

        DRONE_LATCH_INITIAL_POSITION = config.variable("Robot", "DroneLatch_Tuning", "initial_position").value();
        DRONE_LATCH_LAUNCH_POSITION = config.variable("Robot", "DroneLatch_Tuning", "launch_position").value();
    }

    public void standardTelemetry() {
        engine.telemetry.addLine();

        engine.telemetry.addLine("Motors");
        engine.telemetry.addData(
                "Front Left",
                "Power: %.2f, Current: %.2f mAmp, Position: %d",
                frontLeft.getPower(),
                frontLeft.getCurrent(CurrentUnit.MILLIAMPS),
                frontLeft.getCurrentPosition());
        engine.telemetry.addData(
                "Front Right",
                "Power: %.2f, Current: %.2f mAmp, Position: %d",
                frontRight.getPower(),
                frontRight.getCurrent(CurrentUnit.MILLIAMPS),
                frontRight.getCurrentPosition());
        engine.telemetry.addData(
                "Back Left",
                "Power: %.2f, Current: %.2f mAmp, Position: %d",
                backLeft.getPower(),
                backLeft.getCurrent(CurrentUnit.MILLIAMPS),
                backLeft.getCurrentPosition());
        engine.telemetry.addData(
                "Back Right",
                "Power: %.2f, Current: %.2f mAmp, Position: %d",
                backRight.getPower(),
                backRight.getCurrent(CurrentUnit.MILLIAMPS),
                backRight.getCurrentPosition());

        engine.telemetry.addLine();

        engine.telemetry.addLine("Servos");
        engine.telemetry.addData("left claw", leftClaw.getPosition());
        engine.telemetry.addData("right claw", rightClaw.getPosition());
        engine.telemetry.addData("arm", arm.getPower());
        engine.telemetry.addData("droneLatch", droneLatch.getPosition());

        engine.telemetry.addLine();

        engine.telemetry.addData("arm endstop left", armEndStopLeft.isPressed());
        engine.telemetry.addData("arm endstop right", armEndStopRight.isPressed());

        engine.telemetry.addLine();

    }

    public void teleopTelemetry() {
        engine.telemetry.addLine();

        engine.telemetry.addLine();
    }
}