package org.timecrafters.CenterStage.Common;

import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.timecrafters.Library.Robot;
import org.timecrafters.TimeCraftersConfigurationTool.library.TimeCraftersConfiguration;

import dev.cyberarm.engine.V2.CyberarmEngine;

public class PrototypeRobot extends Robot {

    public int armPosition = 0;

    public int oldArmPosititon;
    public long waitTime;
    public double servoWaitTime;
    public double servoSecPerDeg = 0.14/60;
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
    public float lastSetPosShoulder;
    public float lastSetPosElbow;
    public float currentSetPosShoulder;
    public float currentSetPosElbow;
    private HardwareMap hardwareMap;
    public MotorEx frontLeft, frontRight, backLeft, backRight, lift;
    public DcMotor odometerR, odometerL, odometerA;
    public IMU imu;
    public Servo depositorShoulder, depositorElbow, collectorShoulder, collectorElbow, depositor;
    private HDrive xDrive;
    private String string;
    public double xMultiplier = 1;
    public double yMultiplier = 1;
    public double positionX;
    public double positionY;
    public double positionH;

    // robot geometry constants for odometry -----------------------------------------------------------------------------------------------
    public int currentRightPosition = 0;
    public int currentLeftPosition = 0;
    public int currentAuxPosition = 0;
    public int oldRightPosition = 0;
    public int oldLeftPosition = 0;
    public int oldAuxPosition = 0;
    public double globalPositionX;
    public double globalPositionY;
    public double globalPositionH;
    public double localPositionX;
    public double localPositionY;
    public double localPositionH;
    public final static double L = 23.425; // distance between left and right encoder in cm
    final static double B = 10; // distance between the midpoint of the left and right encoder from the auxillary encoder in cm
    public final static double R = 4.5; // wheel radius in cm
    final static double N = 8192; // encoder ticks per revolution (REV encoder)

    public final double MaxVelocityForward = 40;
    public final double MaxStrafeVelocity = 34;
    public final double MaxRotationalVelocity = 20;



    public final double cm_per_tick = (2 * Math.PI * R) / N;
    private CyberarmEngine engine;

    public TimeCraftersConfiguration configuration;

    public PrototypeRobot(String string) {
        this.engine = engine;
        setup();
        this.string = string;
    }

    public void initConstants(){
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
        frontRight = new MotorEx(hardwareMap, "frontRight");
        frontLeft = new MotorEx(hardwareMap, "frontLeft");
        backRight = new MotorEx(hardwareMap, "backRight");
        backLeft = new MotorEx(hardwareMap, "backLeft");
        lift = new MotorEx(hardwareMap, "Lift");

        configuration = new TimeCraftersConfiguration("Blue Crab");

        initConstants();

        frontRight.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.motor.setDirection(DcMotorSimple.Direction.FORWARD);


        frontRight.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //IMU
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.initialize(parameters);

        //SERVO
        depositorShoulder = hardwareMap.servo.get("depositor_shoulder");
        depositorElbow = hardwareMap.servo.get("depositor_elbow");
        collectorShoulder = hardwareMap.servo.get("collector_shoulder");
        collectorElbow = hardwareMap.servo.get("collector_elbow");
        depositor = hardwareMap.servo.get("depositor");

        // input motors exactly as shown below
        xDrive = new HDrive(frontLeft, frontRight,
                            backLeft, backRight);

//        depositorShoulder.setPosition(COLLECTOR_SHOULDER_IN);
//        depositorElbow.setPosition(COLLECTOR_ELBOW_IN);

    }

    public void driveTrainTeleOp() {
        xDrive.driveRobotCentric(-engine.gamepad1.left_stick_x, engine.gamepad1.left_stick_y, -engine.gamepad1.right_stick_x);
    }

    public void ShoulderServoWaitTime(){

        servoWaitTime = servoSecPerDeg * (Math.abs(lastSetPosShoulder - currentSetPosShoulder));

    }

    public void ElbowServoWaitTime(){

        servoWaitTime = 1000 * (servoSecPerDeg * (Math.abs(lastSetPosElbow - currentSetPosElbow)));

    }
    public void OdometryLocalizer(){

        if (Math.toDegrees(positionH) > 360){
            positionH -= 360;
        }

        globalPositionX = localPositionX;
        globalPositionY = localPositionY;
        globalPositionH = localPositionH;


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
}