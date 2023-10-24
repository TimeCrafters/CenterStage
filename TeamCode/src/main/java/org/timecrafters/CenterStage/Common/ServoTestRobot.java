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

public class ServoTestRobot extends Robot {
    public int oldArmPosititon;
    public long servoWaitTime;
    public double servoSecPerDeg = 0.14/60;
    public long waitTime;

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
    public float currentSetPosShoulder;
    public float currentSetPosElbow;
    private HardwareMap hardwareMap;
    public IMU imu;
    public Servo depositorShoulder, depositorElbow, collectorShoulder, collectorElbow;
    private String string;



    private CyberarmEngine engine;

    public TimeCraftersConfiguration configuration;

    public ServoTestRobot(String string) {
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

        configuration = new TimeCraftersConfiguration("Blue Crab");

        initConstants();

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

        depositorShoulder.setPosition(DEPOSITOR_SHOULDER_IN);
        depositorElbow.setPosition(DEPOSITOR_ELBOW_IN);
        collectorShoulder.setPosition(COLLECTOR_SHOULDER_IN);
        collectorElbow.setPosition(COLLECTOR_ELBOW_IN);

    }

    public void ServoWaitTime(Float lastSetPos, Float currentSetPos){

        servoWaitTime = (long) (servoSecPerDeg * (Math.abs(lastSetPos - currentSetPosShoulder)));

    }


}