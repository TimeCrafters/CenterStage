package org.timecrafters.CenterStage.TeleOp.States;

import dev.cyberarm.engine.V2.CyberarmState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import dev.cyberarm.engine.V2.GamepadChecker;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.timecrafters.CenterStage.Common.MiniYTeleOPBot;

public class YellowMinibotTeleOP extends CyberarmState {
    private final MiniYTeleOPBot robot;
    public float angleDelta, drivePower;
    YawPitchRollAngles imuInitAngle;

    public YellowMinibotTeleOP(MiniYTeleOPBot robot) {
        this.robot = robot;
    }


    public float getAngleDelta() {
        robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        return angleDelta;
    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("Front Left Velocity", robot.flDrive.getVelocity());
        engine.telemetry.addData("Front Right Velocity", robot.frDrive.getVelocity());
        engine.telemetry.addData("Back Left Velocity", robot.blDrive.getVelocity());
        engine.telemetry.addData("Back Right Velocity", robot.brDrive.getVelocity());
        engine.telemetry.addData("Front Left Power", robot.flDrive.motor.getPower());
        engine.telemetry.addData("Front Right Power", robot.frDrive.motor.getPower());
        engine.telemetry.addData("Back Left Power", robot.blDrive.motor.getPower());
        engine.telemetry.addData("Back Right Power", robot.brDrive.motor.getPower());
        engine.telemetry.addData("FL Position",robot.flDrive.motor.getCurrentPosition());
        engine.telemetry.addData("IMU Angles", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

    }

    @Override
    public void init() {
        drivePower = 0;
        robot.flDrive.motor.setPower(0);
        robot.frDrive.motor.setPower(0);
        robot.blDrive.motor.setPower(0);
        robot.brDrive.motor.setPower(0);

        robot.imu.resetYaw();
        imuInitAngle = robot.imu.getRobotYawPitchRollAngles();

        GamepadChecker gamepad1Checker = new GamepadChecker(engine, engine.gamepad1);
        GamepadChecker gamepad2Checker = new GamepadChecker(engine, engine.gamepad2);
    }

    @Override
    public void exec() {

        if (Math.abs(engine.gamepad1.right_stick_y) < 0.1 && Math.abs(engine.gamepad1.right_stick_x) < 0.1 && Math.abs(engine.gamepad1.left_stick_x) < 0.1) {
            drivePower = 0;
            robot.flDrive.motor.setPower(0);
            robot.frDrive.motor.setPower(0);
            robot.blDrive.motor.setPower(0);
            robot.brDrive.motor.setPower(0);
        }

        if (engine.gamepad1.start && !engine.gamepad1.a) {
            robot.flDrive.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frDrive.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.blDrive.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.brDrive.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.imu.resetYaw();
        }

        if (Math.abs(engine.gamepad1.right_stick_y) > 0.1 && Math.abs(engine.gamepad1.right_stick_x) < 0.1) {
            drivePower = engine.gamepad1.right_stick_y;
            robot.flDrive.motor.setPower(drivePower);
            robot.frDrive.motor.setPower(drivePower);
            robot.blDrive.motor.setPower(drivePower);
            robot.brDrive.motor.setPower(drivePower);
        }

        if (Math.abs(engine.gamepad1.right_stick_x) > 0.1) {
            drivePower = engine.gamepad1.right_stick_x;
            robot.flDrive.motor.setPower(-drivePower);
            robot.frDrive.motor.setPower(drivePower);
            robot.blDrive.motor.setPower(drivePower);
            robot.brDrive.motor.setPower(-drivePower);
        }

        if (Math.abs(engine.gamepad1.left_stick_x) > 0.1){
            drivePower = engine.gamepad1.left_stick_x;
            robot.flDrive.motor.setPower(-drivePower);
            robot.frDrive.motor.setPower(drivePower);
            robot.blDrive.motor.setPower(-drivePower);
            robot.brDrive.motor.setPower(drivePower);
        }
        
    }
}
