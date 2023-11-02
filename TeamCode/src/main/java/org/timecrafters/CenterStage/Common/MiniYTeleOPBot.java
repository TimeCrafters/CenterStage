package org.timecrafters.CenterStage.Common;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.timecrafters.Library.Robot;
import org.timecrafters.TimeCraftersConfigurationTool.library.TimeCraftersConfiguration;

import dev.cyberarm.engine.V2.CyberarmEngine;

public class MiniYTeleOPBot extends Robot {

    public HardwareMap hardwareMap;
    public MotorEx flDrive, frDrive, blDrive, brDrive;
    public IMU imu;
    private String string;

    public TimeCraftersConfiguration configuration;

    public MiniYTeleOPBot() {
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
        frDrive = new MotorEx(hardwareMap, "FrontRight");
        flDrive = new MotorEx(hardwareMap, "FrontLeft");
        brDrive = new MotorEx(hardwareMap, "BackRight");
        blDrive = new MotorEx(hardwareMap, "BackLeft");

        flDrive.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        frDrive.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        blDrive.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        brDrive.motor.setDirection(DcMotorSimple.Direction.FORWARD);

        flDrive.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frDrive.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blDrive.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brDrive.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flDrive.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frDrive.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blDrive.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brDrive.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}
