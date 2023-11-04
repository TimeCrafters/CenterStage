package org.timecrafters.CenterStage.Common;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;

import org.timecrafters.Library.Robot;
import org.timecrafters.TimeCraftersConfigurationTool.library.TimeCraftersConfiguration;

import dev.cyberarm.engine.V2.CyberarmEngine;

public class MiniBTeleOPBot extends Robot {

public HardwareMap hardwareMap;
public MotorEx leftDrive, rightDrive;
public Servo servLowLeft, servLowRight, servTop;
public IMU imu;

public TimeCraftersConfiguration configuration;

public MiniBTeleOPBot() {
}

    @Override
    public void setup() {
        this.hardwareMap = CyberarmEngine.instance.hardwareMap;
        CyberarmEngine engine = CyberarmEngine.instance;

        imu = engine.hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.initialize(parameters);

        //Motors
        leftDrive = new MotorEx(hardwareMap, "LeftDrive");
        rightDrive = new MotorEx(hardwareMap, "RightDrive");

        leftDrive.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Servos
        servLowLeft = hardwareMap.servo.get("ServoLowLeft");
        servLowRight = hardwareMap.servo.get("ServoLowRight");
        servTop = hardwareMap.servo.get("ServoTop");
    }
}
