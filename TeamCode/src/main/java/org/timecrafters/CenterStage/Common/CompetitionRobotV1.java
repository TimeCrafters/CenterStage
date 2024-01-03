package org.timecrafters.CenterStage.Common;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.timecrafters.Library.Robot;
import org.timecrafters.TimeCraftersConfigurationTool.library.TimeCraftersConfiguration;

import java.util.Objects;

import dev.cyberarm.engine.V2.CyberarmEngine;

@Config
public class CompetitionRobotV1 extends Robot {
    // --------------------------------------------------------------------------------------------------- engine and state setup variables:
    private String string;
    private CyberarmEngine engine;

    public TimeCraftersConfiguration configuration;
    public int objectPos;

    // ------------------------------------------------------------------------------------------------------------------ HardwareMap setup:
    public double power;
    public OpenCvWebcam webcam1 = null;
    public WebcamName webCamName;
    public DcMotor frontLeft, frontRight, backLeft, backRight, lift, clawArm, chinUp;

    public IMU imu;
    public Servo shoulder, elbow, leftClaw, rightClaw, chinUpServo;
    public DistanceSensor customObject;
    public TouchSensor touchLeftArm, touchRightArm;
    public int loopCheck = 0;

    // ----------------------------------------------------------------------------------------------------------------- odometry variables:
    public static double Hp = 0.8, Hi = 0, Hd = 0;
    public static double Xp = -0.035, Xi = 0, Xd = 0;
    public static double Yp = 0.035, Yi = 0, Yd = 0.0013;
    public double rx;
    public double xMultiplier = 1;
    public double yMultiplier = 1;
    public double positionX = 1000;
    public double positionY = 1000;
    public double positionH = 0;
    public double xTarget = 1000;
    public double yTarget = 1000;
    public double hTarget = 0;

    public int currentRightPosition = 0;
    public int currentLeftPosition = 0;
    public int currentAuxPosition = 0;
    public int oldRightPosition = 0;
    public int oldLeftPosition = 0;
    public int oldAuxPosition = 0;
    public final static double L = 22.5; // distance between left and right encoder in cm
    final static double B = 15; // distance between the midpoint of the left and right encoder from the auxillary encoder in cm
    public final static double R = 3; // wheel radius in cm
    final static double N = 8192; // encoder ticks per revolution (REV encoder)

    // heading math variables for pid with imu
    public double headingIntegralSum = 0;
    public double xIntegralSum = 0;
    public double yIntegralSum = 0;
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

    public double yMaxPower = 1;
    public double xMaxPower = 1;
    public double pidX;
    public double pidY;

    //-------------------------------------------------------------------------------------------------------------- arm sequence variables:
    PIDController pidController;

    public String armPos;

    public int target;
    public  double p = 0.007, i = 0,  d = 0.0001, f = 0;
    public  double shoulderCollect = 0.38;
    public  double shoulderDeposit = 0.36;
    public  double shoulderPassive = 0.8;
    public  double elbowCollect = 0.02;
    public  double elbowDeposit = 0;
    public  double elbowPassive = 0;


    private HardwareMap hardwareMap;



    public CompetitionRobotV1(String string) {
        this.engine = engine;
        setup();
        this.string = string;
        pidController = new PIDController(p, i, d);

    }

    public void initConstants() {

    }

    @Override
    public void setup() {
        System.out.println("Bacon: " + this.string);
        this.hardwareMap = CyberarmEngine.instance.hardwareMap;
        this.engine = CyberarmEngine.instance;

        webCamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        imu = engine.hardwareMap.get(IMU.class, "imu");

        //-------------------------------------------------------------------------------------------------------------------------- MOTORS:
        frontRight = engine.hardwareMap.dcMotor.get("frontRight");
        frontLeft = engine.hardwareMap.dcMotor.get("frontLeft");
        backRight = engine.hardwareMap.dcMotor.get("backRight");
        backLeft = engine.hardwareMap.dcMotor.get("backLeft");
        chinUp = engine.hardwareMap.dcMotor.get("chinUp");
        lift = engine.hardwareMap.dcMotor.get("Lift");
        clawArm = engine.hardwareMap.dcMotor.get("clawArm");

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        clawArm.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        chinUp.setDirection(DcMotorSimple.Direction.FORWARD);

        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        chinUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        clawArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        chinUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ----------------------------------------------------------------------------------------------------------------------------- IMU
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.initialize(parameters);

        configuration = new TimeCraftersConfiguration("Blue Crab");

        initConstants();

        pidController = new PIDController(p, i, d);

        // ------------------------------------------------------------------------------------------------------------------------- SENSOR:
        customObject = engine.hardwareMap.get(Rev2mDistanceSensor.class, "customObject");
        touchLeftArm = engine.hardwareMap.get(TouchSensor.class, "left touch");
        touchRightArm = engine.hardwareMap.get(TouchSensor.class, "right touch");

        //--------------------------------------------------------------------------------------------------------------------------- SERVO:
        shoulder = hardwareMap.servo.get("shoulder");
        elbow = hardwareMap.servo.get("elbow");
        leftClaw = hardwareMap.servo.get("leftClaw");
        rightClaw = hardwareMap.servo.get("rightClaw");
        chinUpServo = hardwareMap.servo.get("chinUpServo");


        elbow.setDirection(Servo.Direction.REVERSE);



    }

    // -------------------------------------------------------------------------------------------------------------------------- Functions:


    public void OdometryLocalizer() { // ------------------------------------------------------------------------------- Odometry Localizer:
        // update positions

        oldRightPosition = currentRightPosition;
        oldLeftPosition = currentLeftPosition;
        oldAuxPosition = currentAuxPosition;

        currentRightPosition = frontLeft.getCurrentPosition();
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

    /**
     * A Controller taking error of the heading position and converting to a power in the direction of a target.
     * @param reference reference is the target position
     * @param current  current is the measured sensor value.
     * @return A power to the target the position
     *
     */
    public double HeadingPIDControl(double reference, double current){
        double error = angleWrap(current - reference);
        headingIntegralSum += error * headingTimer.seconds();
        double derivative = (headingLastError - error) / headingTimer.seconds();

        headingTimer.reset();

        double output = (error * Hp) + (derivative * Hd) + (headingIntegralSum * Hi);
        return output;
    }

    public double XPIDControl ( double reference, double current){
        double error = (reference - current);
        xIntegralSum += error * xTimer.seconds();
        double derivative = (error - xLastError) / xTimer.seconds();

        xTimer.reset();

        double output = (error * Xp) + (derivative * Xd) + (xIntegralSum * Xi);
        return output;
    }

    public double YPIDControl ( double reference, double current){
        double error = (reference - current);
        yIntegralSum += error * yTimer.seconds();
        double derivative = (error - yLastError) / yTimer.seconds();

        yTimer.reset();

        double output = (error * Yp) + (derivative * Yd) + (yIntegralSum * Yi);
        return output;
    }

    public void DriveToCoordinates () {
        // determine the velocities needed for each direction
        // this uses PID to adjust needed Power for robot to move to target

//        if (yMaxPower < XPIDControl(xTarget, positionX)){
//            pidX = yMaxPower;
//        } else {
//            pidX = YPIDControl(yTarget, positionY);
//        }
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rx = HeadingPIDControl(Math.toRadians(hTarget), heading);

//        if (xMaxPower < YPIDControl(yTarget, positionY)){
//            pidY = xMaxPower;
//        } else {
//            pidY = XPIDControl(xTarget, positionX);
//        }

            pidY = XPIDControl(xTarget, positionX);
            pidX = YPIDControl(yTarget, positionY);


        double denominator = Math.max(Math.abs(pidX) + Math.abs(pidY) + Math.abs(rx), 1);

        // field oriented math, (rotating the global field to the relative field)
        double rotY = pidY * Math.cos(heading) - pidX * Math.sin(heading);
        double rotX = pidY * Math.sin(heading) + pidX * Math.cos(heading);


        // finding approximate power for each wheel.
        frontLeftPower = (rotY + rotX + rx) / denominator;
        backLeftPower = (rotY - rotX + rx) / denominator;
        frontRightPower = (rotY - rotX - rx) / denominator;
        backRightPower = (rotY + rotX - rx) / denominator;

        // apply my powers
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(-backRightPower);
        frontRight.setPower(frontRightPower);
    }

    public void clawArmControl(){
        if (armPos.equals("collect")) {
            if (lift.getCurrentPosition() >= 20) {
                lift.setPower(-0.6);
            } else {
                lift.setPower(0);
                shoulder.setPosition(shoulderCollect);
                elbow.setPosition(elbowCollect);
                target = 30;
            }
        }
        if (Objects.equals(armPos, "deposit")) {
            shoulder.setPosition(shoulderDeposit);
            elbow.setPosition(elbowDeposit);
            target = 400;


        }
        if (Objects.equals(armPos, "hover")) {
            if (lift.getCurrentPosition() >= 20) {
                lift.setPower(-0.6);
            } else {
                shoulder.setPosition(0.38);
                target = 100;
            }

        }
        if (armPos.equals("search")) {
            shoulder.setPosition(0.15);
            target = 570;

        }

        pidController.setPID(p, i, d);
        int armPos = clawArm.getCurrentPosition();
        double pid = pidController.calculate(armPos, target);

        power = pid;

        clawArm.setPower(power);

    }
}
