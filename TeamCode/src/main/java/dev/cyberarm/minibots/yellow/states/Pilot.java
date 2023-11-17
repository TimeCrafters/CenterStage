package dev.cyberarm.minibots.yellow.states;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.minibots.yellow.YellowMinibot;

public class Pilot extends CyberarmState {
    final YellowMinibot robot;

    public Pilot(YellowMinibot robot) {
        this.robot = robot;
    }
    @Override
    public void exec() {
        /// --- IMU Reset --- ///
        if (engine.gamepad1.guide) {
            robot.imu.resetYaw();
        }

        /// --- DRONE --- ///
        if (engine.gamepad1.y) {
            robot.droneLauncher.setPower(1.0);
        } else if (engine.gamepad1.a) {
            robot.droneLauncher.setPower(0.0);
        }

        /// --- DRIVE --- ///
//        robot.left.set(engine.gamepad1.left_stick_y);
//        robot.right.set(engine.gamepad1.right_stick_y);


        // https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html#field-centric

        double y = engine.gamepad1.left_stick_y; // Remember, Y stick value is reversed;
        double x = -engine.gamepad1.left_stick_x;
        double rx = -engine.gamepad1.right_stick_x;

        double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

//        frontLeftPower *= 24.0;
//        backLeftPower *= 24.0;
//        frontRightPower *= 24.0;
//        backRightPower *= 24.0;

        double maxPower = 1.0;

        robot.leftFront.motorEx.setPower(frontLeftPower * maxPower);
        robot.leftBack.motorEx.setPower(backLeftPower * maxPower);
        robot.rightFront.motorEx.setPower(frontRightPower * maxPower);
        robot.rightBack.motorEx.setPower(backRightPower * maxPower);
    }
}
