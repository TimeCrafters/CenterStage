package org.timecrafters.CenterStage.Common;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.timecrafters.Library.Robot;
import org.timecrafters.TimeCraftersConfigurationTool.library.TimeCraftersConfiguration;

import dev.cyberarm.engine.V2.CyberarmEngine;

@Config
public class CompetitionRobotV1 extends Robot {
    // --------------------------------------------------------------------------------------------------- engine and state setup variables:
    private String string;
    private CyberarmEngine engine;

    public TimeCraftersConfiguration configuration;

    // ------------------------------------------------------------------------------------------------------------------ HardwareMap setup:

    public DcMotor frontLeft, frontRight, backLeft, backRight, lift;
    public DcMotor odometerR, odometerL, odometerA;

    public IMU imu;
    public Servo shoulder, elbow, leftClaw, rightClaw;

    // ----------------------------------------------------------------------------------------------------------------- odometry variables:
    public static double Hp = 0, Hi = 0, Hd = 0;
    public static double Xp = 0, Xi = 0, Xd = 0;
    public static double Yp = 0, Yi = 0, Yd = 0;
    private double drivePower = 1;
    public double rx;
    public double xMultiplier = 1;
    public double yMultiplier = 1;
    public double positionX = 0;
    public double positionY = 0;
    public double positionH = 0;
    public double xTarget;
    public double yTarget;
    public double hTarget;

    public int currentRightPosition = 0;
    public int currentLeftPosition = 0;
    public int currentAuxPosition = 0;
    public int oldRightPosition = 0;
    public int oldLeftPosition = 0;
    public int oldAuxPosition = 0;
    public final static double L = 22.5; // distance between left and right encoder in cm
    final static double B = 11.5; // distance between the midpoint of the left and right encoder from the auxillary encoder in cm
    public final static double R = 3; // wheel radius in cm
    final static double N = 8192; // encoder ticks per revolution (REV encoder)

    // heading math variables for pid with imu
    public double headingIntegralSum = 0;
    public double xIntegralSum = 0;
    public double yIntegralSum = 0;
    public double targetHeading;
    public final double cm_per_tick = (2 * Math.PI * R) / N;
    private double headingLastError = 0;
    private double xLastError = 0;
    private double yLastError = 0;
    ElapsedTime headingTimer = new ElapsedTime();
    ElapsedTime xTimer = new ElapsedTime();
    ElapsedTime yTimer = new ElapsedTime();
    public double frontLeftPower;
    public double backLeftPower;
    public double frontRightPower;
    public double backRightPower;

    //-------------------------------------------------------------------------------------------------------------- arm sequence variables:
    public static double shoulderCollect = 0.25;
    public static double shoulderDeposit = 0.55;
    public static double shoulderPassive = 1;
    public static double elbowCollect = 0.7;
    public static double elbowDeposit = 0.8;
    public static double elbowPassive = 0.28;

    private HardwareMap hardwareMap;



    public CompetitionRobotV1(String string) {
        this.engine = engine;
        setup();
        this.string = string;
    }

    public void initConstants() {

    }

    @Override
    public void setup() {
        System.out.println("Bacon: " + this.string);
        this.hardwareMap = CyberarmEngine.instance.hardwareMap;
        this.engine = CyberarmEngine.instance;

        imu = engine.hardwareMap.get(IMU.class, "imu");

        //-------------------------------------------------------------------------------------------------------------------------- MOTORS:
        frontRight = engine.hardwareMap.dcMotor.get("frontRight");
        frontLeft = engine.hardwareMap.dcMotor.get("frontLeft");
        backRight = engine.hardwareMap.dcMotor.get("backRight");
        backLeft = engine.hardwareMap.dcMotor.get("backLeft");
        lift = engine.hardwareMap.dcMotor.get("Lift");

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);

        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //IMU
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.initialize(parameters);

        imu.resetYaw();

        configuration = new TimeCraftersConfiguration("Blue Crab");

        initConstants();


        //--------------------------------------------------------------------------------------------------------------------------- SERVO:
        shoulder = hardwareMap.servo.get("shoulder");
        elbow = hardwareMap.servo.get("elbow");
        leftClaw = hardwareMap.servo.get("leftClaw");
        rightClaw = hardwareMap.servo.get("leftClaw");

        shoulder.setPosition(shoulderPassive);
        elbow.setPosition(elbowPassive);



    }

    // -------------------------------------------------------------------------------------------------------------------------- Functions:

    public void OdometryLocalizer() { // ------------------------------------------------------------------------------- Odometry Localizer:
        // update positions
        oldRightPosition = currentRightPosition;
        oldLeftPosition = currentLeftPosition;
        oldAuxPosition = currentAuxPosition;

        currentRightPosition = -frontLeft.getCurrentPosition();
        currentLeftPosition = -backRight.getCurrentPosition();
        currentAuxPosition = backLeft.getCurrentPosition();

        int dnl1 = currentLeftPosition - oldLeftPosition;
        int dnr2 = currentRightPosition - oldRightPosition;
        int dna3 = currentAuxPosition - oldAuxPosition;

        // the robot has turned and moved a tiny bit between two measurements
        double dtheta = cm_per_tick * (dnr2 - dnl1) / L;
        double dx = cm_per_tick * (dnl1 + dnr2) / 2.0;
        double dy = cm_per_tick * (dna3 - (dnr2 - dnl1) * B / L);

        // the small movement of the bot gets added to the field coordinates
        double theta = positionH + (dtheta / 2.0);
        positionX += (dx * Math.cos(theta) - dy * Math.sin(theta)) * xMultiplier;
        positionY += (dx * Math.sin(theta) + dy * Math.cos(theta)) * yMultiplier;
        positionH += dtheta;

    }
    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }
    public double HeadingPIDControl(double reference, double current){
        double error = angleWrap(current - reference);
        headingIntegralSum += error * headingTimer.seconds();
        double derivative = (headingLastError - error) / headingTimer.seconds();

        headingTimer.reset();

        double output = (error * Hp) + (derivative * Hd) + (headingIntegralSum * Hi);
        return output;
    }

    public double XPIDControl ( double reference, double current){
        double error = (current - reference);
        xIntegralSum += error * xTimer.seconds();
        double derivative = (error - xLastError) / xTimer.seconds();

        xTimer.reset();

        double output = (error * Xp) + (derivative * Xd) + (xIntegralSum * Xi);
        return output;
    }

    public double YPIDControl ( double reference, double current){
        double error = (current - reference);
        yIntegralSum += error * yTimer.seconds();
        double derivative = (error - yLastError) / yTimer.seconds();

        yTimer.reset();

        double output = (error * Yp) + (derivative * Yd) + (yIntegralSum * Yi);
        return output;
    }

    public void DriveToCoordinates () {

        // determine the velocities needed for each direction
        // this uses PID to adjust needed Power for robot to move to target
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rx = HeadingPIDControl(Math.toRadians(hTarget), heading);
        double pidX = XPIDControl(xTarget, positionX);
        double pidY = YPIDControl(yTarget, positionY);

        double denominator = Math.max(Math.abs(pidX) + Math.abs(pidY) + Math.abs(rx), 1);

        // field oriented math, (rotating the global field to the relative field)
        double rotY = pidY * Math.cos(heading) - pidX * Math.sin(heading);
        double rotX = pidY * Math.sin(heading) + pidX * Math.cos(heading);


        // finding approximate power for each wheel.
        frontLeftPower = (rotY + rotX + rx) / denominator;
        backLeftPower = (rotY - rotX + rx) / denominator;
        frontRightPower = (rotY - rotX - rx) / denominator;
        backRightPower = (rotY - rotX + rx) / denominator;

        // apply my powers
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
        frontRight.setPower(frontRightPower);
    }
}