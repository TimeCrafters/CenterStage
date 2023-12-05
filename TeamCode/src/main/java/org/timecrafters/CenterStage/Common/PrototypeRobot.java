package org.timecrafters.CenterStage.Common;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.timecrafters.Library.Robot;
import org.timecrafters.TimeCraftersConfigurationTool.library.TimeCraftersConfiguration;

import dev.cyberarm.engine.V2.CyberarmEngine;

@Config
public class PrototypeRobot extends Robot {
    public boolean headingLock = false;
    public double collectLock = Math.toRadians(-90);
    public double backDropLock = Math.toRadians(90);
    public int armPosition = 0;
    public long startOfSequencerTime;
    public int oldArmPosition = 0;
    public float DEPOSITOR_SHOULDER_IN;
    public float DEPOSITOR_SHOULDER_OUT;
    public float DEPOSITOR_ELBOW_IN;
    public float DEPOSITOR_ELBOW_OUT;
    public float COLLECTOR_SHOULDER_IN;
    public float COLLECTOR_SHOULDER_PASSIVE;
    public float COLLECTOR_SHOULDER_OUT;
    public float COLLECTOR_ELBOW_IN;
    public float COLLECTOR_ELBOW_PASSIVE;
    public float COLLECTOR_ELBOW_OUT;

    // hardware map
    private HardwareMap hardwareMap;
    public DcMotor frontLeft, frontRight, backLeft, backRight, lift;
    public DcMotor odometerR, odometerL, odometerA;
    public IMU imu;
    public Servo depositorShoulder, depositorElbow, collectorShoulder, collectorElbow, depositor, collector;
    private String string;

    // robot geometry constants for odometry -----------------------------------------------------------------------------------------------
    public static double Hp = 0, Hi = 0, Hd = 0;
    public static double Xp = 0, Xi = 0, Xd = 0;
    public static double Yp = 0, Yi = 0, Yd = 0;
    private double drivePower = 1;

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
    public double rx;
    public final static double L = 23.425; // distance between left and right encoder in cm
    final static double B = 10; // distance between the midpoint of the left and right encoder from the auxillary encoder in cm
    public final static double R = 4.5; // wheel radius in cm
    final static double N = 8192; // encoder ticks per revolution (REV encoder)

    // heading math variables for pid with imu
    public double integralSum = 0;
    public double targetHeading;

    private double lastError = 0;
    ElapsedTime timer = new ElapsedTime();


    // collector / depositor variables
    private boolean lbsVar1;
    private boolean lbsVar2;
    private boolean rbsVar2;
    public float depositorPos;
    public float collectorPos;

    public final double cm_per_tick = (2 * Math.PI * R) / N;
    private CyberarmEngine engine;

    public TimeCraftersConfiguration configuration;

    public PrototypeRobot(String string) {
        this.engine = engine;
        setup();
        this.string = string;
    }

    public void initConstants() {
        DEPOSITOR_SHOULDER_IN = configuration.variable("Robot", "Tuning", "DEPOSITOR_SHOULDER_IN").value();
        DEPOSITOR_SHOULDER_OUT = configuration.variable("Robot", "Tuning", "DEPOSITOR_SHOULDER_OUT").value();
        DEPOSITOR_ELBOW_IN = configuration.variable("Robot", "Tuning", "DEPOSITOR_ELBOW_IN").value();
        DEPOSITOR_ELBOW_OUT = configuration.variable("Robot", "Tuning", "DEPOSITOR_ELBOW_OUT").value();
        COLLECTOR_SHOULDER_IN = configuration.variable("Robot", "Tuning", "COLLECTOR_SHOULDER_IN").value();
        COLLECTOR_SHOULDER_PASSIVE = configuration.variable("Robot", "Tuning", "COLLECTOR_SHOULDER_PASSIVE").value();
        COLLECTOR_SHOULDER_OUT = configuration.variable("Robot", "Tuning", "COLLECTOR_SHOULDER_OUT").value();
        COLLECTOR_ELBOW_IN = configuration.variable("Robot", "Tuning", "COLLECTOR_ELBOW_IN").value();
        COLLECTOR_ELBOW_PASSIVE = configuration.variable("Robot", "Tuning", "COLLECTOR_ELBOW_PASSIVE").value();
        COLLECTOR_ELBOW_OUT = configuration.variable("Robot", "Tuning", "COLLECTOR_ELBOW_OUT").value();
    }

    @Override
    public void setup() {
        System.out.println("Bacon: " + this.string);
        this.hardwareMap = CyberarmEngine.instance.hardwareMap;
        this.engine = CyberarmEngine.instance;

        imu = engine.hardwareMap.get(IMU.class, "imu");

        //MOTORS
        frontRight = engine.hardwareMap.dcMotor.get("frontRight");
        frontLeft = engine.hardwareMap.dcMotor.get("frontLeft");
        backRight = engine.hardwareMap.dcMotor.get("backRight");
        backLeft = engine.hardwareMap.dcMotor.get("backLeft");
        lift = engine.hardwareMap.dcMotor.get("Lift");
        odometerL = engine.hardwareMap.dcMotor.get("leftodo");
        odometerR = engine.hardwareMap.dcMotor.get("rightodo");
        odometerA = engine.hardwareMap.dcMotor.get("auxodo");

        configuration = new TimeCraftersConfiguration("Blue Crab");

        initConstants();

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        odometerR.setDirection(DcMotorSimple.Direction.FORWARD);
        odometerL.setDirection(DcMotorSimple.Direction.REVERSE);
        odometerA.setDirection(DcMotorSimple.Direction.FORWARD);


        odometerR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometerL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometerA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


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

        //SERVO
        depositorShoulder = hardwareMap.servo.get("depositor_shoulder");
        depositorElbow = hardwareMap.servo.get("depositor_elbow");
        depositor = hardwareMap.servo.get("depositor");
        collector = hardwareMap.servo.get("collector");

        depositorShoulder.setPosition(DEPOSITOR_SHOULDER_IN);
        depositorElbow.setPosition(DEPOSITOR_ELBOW_IN);
        collectorElbow.setPosition(COLLECTOR_ELBOW_IN);
        collectorShoulder.setPosition(COLLECTOR_SHOULDER_IN);
        collector.setPosition(0F);
    }

        public void driveTrainTeleOp () {
            boolean lbs1 = engine.gamepad1.left_stick_button;
            if (lbs1 && !lbsVar1) {
                if (drivePower == 1) {
                    drivePower = 0.5;
                } else {
                    drivePower = 1;
                }
            }
            lbsVar1 = lbs1;

            double y = -engine.gamepad1.left_stick_y;
            double x = engine.gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            // angle math to make things field oriented
            double heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(heading) - y * Math.sin(heading);
            double rotY = x * Math.sin(heading) + y * Math.cos(heading);

            // joystick math to find the approximate power across each wheel for a movement
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY + rotX - rx) / denominator;
            double backRightPower = (rotY - rotX - rx) / denominator;

            // setting each power determined previously from the math above
            // as well as multiplying it by a drive power that can be changed.
            backLeft.setPower(backLeftPower * drivePower);
            backRight.setPower(backRightPower * drivePower);
            frontLeft.setPower(frontLeftPower * drivePower);
            frontRight.setPower(frontRightPower * drivePower);
        }

        public void OdometryLocalizer () {

            // update positions
            oldRightPosition = currentRightPosition;
            oldLeftPosition = currentLeftPosition;
            oldAuxPosition = currentAuxPosition;

            currentRightPosition = -odometerR.getCurrentPosition();
            currentLeftPosition = -odometerL.getCurrentPosition();
            currentAuxPosition = odometerA.getCurrentPosition();

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

        public void CollectorToggle () {
            boolean lbs2 = engine.gamepad2.left_stick_button;
            if (lbs2 && !lbsVar2) {
                if (collectorPos == 1F) {
                    collectorPos = 0F;
                } else {
                    collectorPos = 1F;
                }
            }
            lbsVar2 = lbs2;
        }

        public void DepositorToggle () {
            boolean rbs2 = engine.gamepad2.right_stick_button;
            if (rbs2 && !rbsVar2) {
                if (depositorPos == 0.6F) {
                    depositorPos = 0F;
                } else {
                    depositorPos = 0.6F;
                }
            }
            rbsVar2 = rbs2;
        }

        public double angleWrap ( double radians){
            while (radians > Math.PI) {
                radians -= 2 * Math.PI;
            }
            while (radians < -Math.PI) {
                radians += 2 * Math.PI;
            }
            return radians;
        }

        public double HeadingPIDControl ( double reference, double current){
            double error = angleWrap(current - reference);
            integralSum += error * timer.seconds();
            double derivative = (error - lastError) / timer.seconds();

            timer.reset();

            double output = (error * Hp) + (derivative * Hd) + (integralSum * Hi);
            return output;
        }

        public double XPIDControl ( double reference, double current){
            double error = (current - reference);
            integralSum += error * timer.seconds();
            double derivative = (error - lastError) / timer.seconds();

            timer.reset();

            double output = (error * Xp) + (derivative * Xd) + (integralSum * Xi);
            return output;
        }

        public double YPIDControl ( double reference, double current){
            double error = (current - reference);
            integralSum += error * timer.seconds();
            double derivative = (error - lastError) / timer.seconds();

            timer.reset();

            double output = (error * Yp) + (derivative * Yd) + (integralSum * Yi);
            return output;
        }

        public void DriveToCoordinates () {

            // determine the velocities needed for each direction
            // this uses PID to adjust needed Power for robot to move to target
            double heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rx = HeadingPIDControl(Math.toRadians(hTarget), heading);
            double pidX = XPIDControl(xTarget, positionX);
            double pidY = YPIDControl(yTarget, positionY);

            double denominator = Math.max(Math.abs(pidX) + Math.abs(pidY) + Math.abs(rx), 1);

            // field oriented math, (rotating the global field to the relative field)
            double rotY = pidY * Math.cos(heading) - pidX * Math.sin(heading);
            double rotX = pidY * Math.sin(heading) + pidX * Math.cos(heading);


            // finding approximate power for each wheel.
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY + rotX - rx) / denominator;
            double backRightPower = (rotY - rotX - rx) / denominator;

            // apply my powers
            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);
            frontRight.setPower(frontRightPower);
        }
    }


