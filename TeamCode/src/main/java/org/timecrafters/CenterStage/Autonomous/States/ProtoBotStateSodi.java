package org.timecrafters.CenterStage.Autonomous.States;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import dev.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.CenterStage.Common.ProtoBotSodi;


public class ProtoBotStateSodi extends CyberarmState {
    ProtoBotSodi robot;
    private double avgVelocity, avgDrivePower;
    private long lastTimeChecked;
    public ProtoBotStateSodi(ProtoBotSodi robot) {
        this.robot = robot;
    }
    public void telemetry() {
//    engine.telemetry.addData("Avg Drive Velocity", avgVelocity);
//    engine.telemetry.addData("Avg Drive Power", avgDrivePower);
    engine.telemetry.addData("Front Left Velocity", robot.flDrive.getVelocity());
    engine.telemetry.addData("Front Right Velocity", robot.frDrive.getVelocity());
    engine.telemetry.addData("Back Left Velocity", robot.blDrive.getVelocity());
    engine.telemetry.addData("Back Right Velocity", robot.brDrive.getVelocity());
    engine.telemetry.addData("Front Left Power", robot.flDrive.motor.getPower());
    engine.telemetry.addData("Front Right Power", robot.frDrive.motor.getPower());
    engine.telemetry.addData("Back Left Power", robot.blDrive.motor.getPower());
    engine.telemetry.addData("Back Right Power", robot.brDrive.motor.getPower());

    }

//    public double getAvgDrivePower() {
//        avgDrivePower = (robot.flDrive.motor.getPower() + robot.frDrive.motor.getPower() + robot.blDrive.motor.getPower() + robot.brDrive.motor.getPower())/4;
//        return avgDrivePower;
//    }

//    public double getAvgVelocity() {
//        avgVelocity = (robot.flDrive.getVelocity() + robot.frDrive.getVelocity() + robot.blDrive.getVelocity() + robot.brDrive.getVelocity())/4;
//        return avgVelocity;
//    }

    @Override
    public void init() {
        robot.flDrive.motor.setPower(0);
        robot.frDrive.motor.setPower(0);
        robot.blDrive.motor.setPower(0);
        robot.brDrive.motor.setPower(0);
        robot.liftMotor.motor.setPower(0);

        robot.grabJaw.setPosition(0);
        robot.grabElbow.setPosition(0);
        robot.grabShoulder.setPosition(0);
        robot.dropShoulder.setPosition(0);
        robot.dropElbow.setPosition(0);
        robot.dropJaw.setPosition(0);

        lastTimeChecked = System.currentTimeMillis();


    }

    @Override
    public void exec() {

        if (System.currentTimeMillis() - lastTimeChecked >= 500 && System.currentTimeMillis() - lastTimeChecked < 2500) {
            robot.flDrive.motor.setPower(0.5);
            robot.frDrive.motor.setPower(0.5);
            robot.blDrive.motor.setPower(0.5);
            robot.brDrive.motor.setPower(0.5);
            robot.liftMotor.motor.setPower(0.5);
        } else if (System.currentTimeMillis() - lastTimeChecked >= 2500 && System.currentTimeMillis() - lastTimeChecked < 4500) {
            robot.flDrive.motor.setPower(-0.5);
            robot.frDrive.motor.setPower(-0.5);
            robot.blDrive.motor.setPower(-0.5);
            robot.brDrive.motor.setPower(-0.5);
            robot.liftMotor.motor.setPower(-0.5);
        } else if (System.currentTimeMillis() - lastTimeChecked >= 4500 && System.currentTimeMillis() - lastTimeChecked < 6500) {
            robot.flDrive.motor.setPower(0.5);
            robot.frDrive.motor.setPower(0.5);
            robot.blDrive.motor.setPower(-0.5);
            robot.brDrive.motor.setPower(-0.5);
            robot.liftMotor.motor.setPower(0);
        } else if (System.currentTimeMillis() - lastTimeChecked >= 6500 && System.currentTimeMillis() - lastTimeChecked < 8500) {
            robot.flDrive.motor.setPower(-0.5);
            robot.frDrive.motor.setPower(-0.5);
            robot.blDrive.motor.setPower(0.5);
            robot.brDrive.motor.setPower(0.5);
            robot.liftMotor.motor.setPower(0);
        } else if (System.currentTimeMillis() - lastTimeChecked >= 8600){
            robot.flDrive.motor.setPower(0);
            robot.frDrive.motor.setPower(0);
            robot.blDrive.motor.setPower(0);
            robot.brDrive.motor.setPower(0);
            robot.liftMotor.motor.setPower(0);
            setHasFinished(true);
        }
    }
}
