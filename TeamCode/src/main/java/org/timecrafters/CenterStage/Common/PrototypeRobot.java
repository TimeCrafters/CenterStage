package org.timecrafters.CenterStage.Common;

import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
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
    public double servoWaitTime;
    public double servoSecPerDeg = 0.14/60;
    public float ELBOW_COLLECT;
    public float ELBOW_DEPOSIT;
    public float SHOULDER_COLLECT;
    public float SHOULDER_DEPOSIT;
    public float lastSetPosShoulder = SHOULDER_COLLECT;
    public float lastSetPosElbow = ELBOW_COLLECT;
    public float currentSetPosShoulder;
    public float currentSetPosElbow;
    private HardwareMap hardwareMap;
    public MotorEx frontLeft, frontRight, backLeft, backRight, lift;
    public IMU imu;
    public Servo depositorShoulder, depositorElbow, depositor;
    private HDrive xDrive;
    private String string;
    private CyberarmEngine engine;

    public TimeCraftersConfiguration configuration;

    public PrototypeRobot(String string) {
        this.engine = engine;
        setup();
        this.string = string;
    }

    public void initConstants(){
            ELBOW_DEPOSIT = configuration.variable("Robot", "Tuning", "ELBOW_DEPOSIT").value();
            ELBOW_COLLECT = configuration.variable("Robot", "Tuning", "ELBOW_COLLECT").value();
            SHOULDER_DEPOSIT = configuration.variable("Robot", "Tuning", "SHOULDER_DEPOSIT").value();
            SHOULDER_COLLECT = configuration.variable("Robot", "Tuning", "SHOULDER_COLLECT").value();
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
        lift = new MotorEx(hardwareMap, "lift");

        configuration = new TimeCraftersConfiguration("Robbie");

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
        depositor = hardwareMap.servo.get("depositor");

        // input motors exactly as shown below
        xDrive = new HDrive(frontLeft, frontRight,
                            backLeft, backRight);

        depositorShoulder.setPosition(SHOULDER_COLLECT);
        depositorElbow.setPosition(ELBOW_COLLECT);

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
}