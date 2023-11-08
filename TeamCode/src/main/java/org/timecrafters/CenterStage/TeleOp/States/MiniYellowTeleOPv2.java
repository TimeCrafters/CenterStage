package org.timecrafters.CenterStage.TeleOp.States;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.timecrafters.CenterStage.Common.MiniYTeleOPBot;
import org.timecrafters.Library.Robot;
import org.timecrafters.TimeCraftersConfigurationTool.library.TimeCraftersConfiguration;

import dev.cyberarm.engine.V2.CyberarmEngine;
import dev.cyberarm.engine.V2.CyberarmState;

public class MiniYellowTeleOPv2 extends CyberarmState {

    public HardwareMap hardwareMap;
    public MotorEx flDrive, frDrive, blDrive, brDrive;
    public IMU imu;
    private double flPower, frPower, blPower, brPower;
    private float lStickY, transitPercent, rotPercent;

    public TimeCraftersConfiguration configuration;
        @Override
    public void init() {
        this.hardwareMap = CyberarmEngine.instance.hardwareMap;
        CyberarmEngine engine = CyberarmEngine.instance;

//        configuration = new TimeCraftersConfiguration("Minibot Yellow");


        imu = engine.hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.initialize(parameters);
        imu.resetYaw();

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

    @Override
    public void telemetry() {
        engine.telemetry.addData("FLPower", flPower);
        engine.telemetry.addData("Transit %", transitPercent);
        engine.telemetry.addData("L Stick Y", lStickY);
        engine.telemetry.update();
    }

    @Override
    public void exec () {
        transitPercent = engine.gamepad1.left_stick_y * 100;
        rotPercent = engine.gamepad1.right_stick_x * -100;
        flPower = ((transitPercent + rotPercent) / Math.abs(transitPercent + rotPercent));
        flDrive.motor.setPower(flPower);

    }
}
