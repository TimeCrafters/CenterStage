package org.timecrafters.CenterStage.Common;

import org.timecrafters.Library.Robot;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.timecrafters.Library.Robot;
import org.timecrafters.TimeCraftersConfigurationTool.library.TimeCraftersConfiguration;

import dev.cyberarm.engine.V2.CyberarmEngine;

public class ProtoBotSodi extends Robot {

    public HardwareMap hardwareMap;
    public MotorEx flDrive, frDrive, blDrive, brDrive, bloodWorm;
    public CRServo fang;
    public Servo jaw;
    @Override
    public void setup() {

        //Motors
        flDrive = new MotorEx(hardwareMap, "FrontLeftDrive");
        frDrive = new MotorEx(hardwareMap, "FrontRightDrive");
        blDrive = new MotorEx(hardwareMap, "BackLeftDrive");
        brDrive = new MotorEx(hardwareMap, "BackRightDrive");
        bloodWorm = new MotorEx(hardwareMap, "Proboscis");
        //The motor bloodWorm is the one that raises the deliverer.
        //The reason it's called bloodworm is because of what bloodworms are. Better left unsaid.

    }
}
