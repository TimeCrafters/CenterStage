package org.timecrafters.CenterStage.Autonomous.States;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import dev.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.CenterStage.Common.ProtoBotSodi;


public class ProtoBotStateSodi extends CyberarmState {
    ProtoBotSodi robot;
    private double avgVelocity, avgDrivePower;
    public ProtoBotStateSodi() {

    }
    public void telemetry() {
    engine.telemetry.addData("Avg Drive Velocity", avgVelocity);
    engine.telemetry.addData("Avg Drive Power", avgDrivePower);
    engine.telemetry.addData("Front Left Velocity", robot.flDrive.getVelocity());
    engine.telemetry.addData("Front Right Velocity", robot.frDrive.getVelocity());
    engine.telemetry.addData("Back Left Velocity", robot.blDrive.getVelocity());
    engine.telemetry.addData("Back Right Velocity", robot.brDrive.getVelocity());
    engine.telemetry.addData("Front Left Power", robot.flDrive.motor.getPower());
    engine.telemetry.addData("Front Right Power", robot.frDrive.motor.getPower());
    engine.telemetry.addData("Back Left Power", robot.blDrive.motor.getPower());
    engine.telemetry.addData("Back Right Power", robot.brDrive.motor.getPower());

    }

    public double getAvgDrivePower() {
        avgDrivePower = (robot.flDrive.motor.getPower() + robot.frDrive.motor.getPower() + robot.blDrive.motor.getPower() + robot.brDrive.motor.getPower())/4;
        return avgDrivePower;
    }

    public double getAvgVelocity() {
        avgVelocity = (robot.flDrive.getVelocity() + robot.frDrive.getVelocity() + robot.blDrive.getVelocity() + robot.brDrive.getVelocity())/4;
        return avgVelocity;
    }

    @Override
    public void init() {
        robot.flDrive.motor.setPower(0);
        robot.frDrive.motor.setPower(0);
        robot.blDrive.motor.setPower(0);
        robot.brDrive.motor.setPower(0);
        robot.liftMotor.motor.setPower(0);

//        robot.fang.setPosition(0);
//        robot.jaw.setPosition(0);
//        robot.neck.setPosition(0);
//        robot.shoulder.setPosition(0);
//        robot.wrist.setPosition(0);
//        robot.hand.setPosition(0);

    }

    @Override
    public void exec() {

        if (System.currentTimeMillis() >= 500 && System.currentTimeMillis() < 2500) {
            robot.flDrive.motor.setPower(0.5);
            robot.frDrive.motor.setPower(0.5);
            robot.blDrive.motor.setPower(0.5);
            robot.brDrive.motor.setPower(0.5);
            robot.liftMotor.motor.setPower(0.5);
        } else if (System.currentTimeMillis() >= 2500 && System.currentTimeMillis() < 3500) {
            robot.flDrive.motor.setPower(-0.5);
            robot.frDrive.motor.setPower(-0.5);
            robot.blDrive.motor.setPower(-0.5);
            robot.brDrive.motor.setPower(-0.5);
            robot.liftMotor.motor.setPower(-0.5);
        } else if (System.currentTimeMillis() >= 3500 && System.currentTimeMillis() < 4500) {
            robot.flDrive.motor.setPower(0.5);
            robot.frDrive.motor.setPower(0.5);
            robot.blDrive.motor.setPower(-0.5);
            robot.brDrive.motor.setPower(-0.5);
        } else if (System.currentTimeMillis() >= 4500 && System.currentTimeMillis() < 5500) {
            robot.flDrive.motor.setPower(-0.5);
            robot.frDrive.motor.setPower(-0.5);
            robot.blDrive.motor.setPower(0.5);
            robot.brDrive.motor.setPower(0.5);
        } else {
            robot.flDrive.motor.setPower(0);
            robot.frDrive.motor.setPower(0);
            robot.blDrive.motor.setPower(0);
            robot.brDrive.motor.setPower(0);
            robot.liftMotor.motor.setPower(0);
        }
    }
}
