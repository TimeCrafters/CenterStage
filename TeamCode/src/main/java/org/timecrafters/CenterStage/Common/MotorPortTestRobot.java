package org.timecrafters.CenterStage.Common;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.timecrafters.Library.Robot;
import org.timecrafters.TimeCraftersConfigurationTool.library.TimeCraftersConfiguration;

import dev.cyberarm.engine.V2.CyberarmEngine;

public class MotorPortTestRobot extends Robot {
    private HardwareMap hardwareMap;
    private String string;
    private CyberarmEngine engine;

    public MotorEx eh0, eh1, eh2, eh3, ch0, ch1, ch2, ch3;

    public MotorPortTestRobot(String string) {
        this.engine = engine;
        this.string = string;
    }

    @Override
    public void setup() {
        eh0 = new MotorEx(hardwareMap, "eh0");
        eh1 = new MotorEx(hardwareMap, "eh1");
        eh2 = new MotorEx(hardwareMap, "eh2");
        eh3 = new MotorEx(hardwareMap, "eh3");
        ch0 = new MotorEx(hardwareMap, "ch0");
        ch1 = new MotorEx(hardwareMap, "ch1");
        ch2 = new MotorEx(hardwareMap, "ch2");
        ch3 = new MotorEx(hardwareMap, "ch3");
    }
}