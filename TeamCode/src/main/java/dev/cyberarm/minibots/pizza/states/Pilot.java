package dev.cyberarm.minibots.pizza.states;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.minibots.pizza.PizzaMinibot;

public class Pilot extends CyberarmState {
    final PizzaMinibot robot;

    public Pilot(PizzaMinibot robot) {
        this.robot = robot;
    }
    @Override
    public void exec() {
        /// --- IMU Reset --- ///
        if (engine.gamepad1.guide) {
            robot.imu.resetYaw();
        }

        /// --- GRIPPER --- ///
        if (engine.gamepad1.x) {
            robot.gripper.setPosition(PizzaMinibot.GRIPPER_CLOSED);
        } else if (engine.gamepad1.b) {
            robot.gripper.setPosition(PizzaMinibot.GRIPPER_OPEN);
        }

        /// --- ARM --- ///
        if (engine.gamepad1.dpad_down) {
            robot.arm.setPosition(PizzaMinibot.ARM_STOW);
            robot.armStackPosition = -1;
        } else if (engine.gamepad1.dpad_up) {
            robot.arm.setPosition(PizzaMinibot.ARM_COLLECT);
            robot.armStackPosition = -1;
        } else if (engine.gamepad1.dpad_right) {
            robot.arm.setPosition(PizzaMinibot.ARM_PRECOLLECT);
            robot.armStackPosition = -1;
        } else if (engine.gamepad1.dpad_left) {
            robot.arm.setPosition(PizzaMinibot.ARM_DELIVER);
            robot.armStackPosition = -1;
        }

        /// --- DRIVE --- ///
//        robot.left.set(engine.gamepad1.left_stick_y);
//        robot.right.set(engine.gamepad1.right_stick_y);


        // https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html#field-centric

        double y = -engine.gamepad1.left_stick_y; // Remember, Y stick value is reversed;
        double x = engine.gamepad1.left_stick_x;
        double rx = engine.gamepad1.right_stick_x;

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

        double maxPower = 0.5;

        robot.leftFront.motorEx.setPower(frontLeftPower * maxPower);
        robot.leftBack.motorEx.setPower(backLeftPower * maxPower);
        robot.rightFront.motorEx.setPower(frontRightPower * maxPower);
        robot.rightBack.motorEx.setPower(backRightPower * maxPower);
    }

    @Override
    public void buttonDown(Gamepad gamepad, String button) {
        if (gamepad == engine.gamepad1) {
            if (button.equals("left_bumper")) {
                switch (robot.armStackPosition) {
                    case -1:
                        robot.arm.setPosition(PizzaMinibot.ARM_HOVER_5_STACK);
                        robot.armStackPosition = 5;
                        break;
                    case 5:
                        robot.arm.setPosition(PizzaMinibot.ARM_HOVER_5_STACK);
                        robot.armStackPosition = 5;
                        break;
                    case 4:
                        robot.arm.setPosition(PizzaMinibot.ARM_HOVER_5_STACK);
                        robot.armStackPosition = 5;
                        break;
                    case 3:
                        robot.arm.setPosition(PizzaMinibot.ARM_HOVER_4_STACK);
                        robot.armStackPosition = 4;
                        break;
                    case 2:
                        robot.arm.setPosition(PizzaMinibot.ARM_HOVER_3_STACK);
                        robot.armStackPosition = 3;
                        break;
                    case 1:
                        robot.arm.setPosition(PizzaMinibot.ARM_HOVER_2_STACK);
                        robot.armStackPosition = 2;
                        break;
                    default:
                        break;
                }
            }
            if (button.equals("right_bumper")) {
                switch (robot.armStackPosition) {
                    case -1:
                        robot.arm.setPosition(PizzaMinibot.ARM_HOVER_5_STACK);
                        robot.armStackPosition = 5;
                        break;
                    case 5:
                        robot.arm.setPosition(PizzaMinibot.ARM_HOVER_4_STACK);
                        robot.armStackPosition = 4;
                        break;
                    case 4:
                        robot.arm.setPosition(PizzaMinibot.ARM_HOVER_3_STACK);
                        robot.armStackPosition = 3;
                        break;
                    case 3:
                        robot.arm.setPosition(PizzaMinibot.ARM_HOVER_2_STACK);
                        robot.armStackPosition = 2;
                        break;
                    case 2:
                        robot.arm.setPosition(PizzaMinibot.ARM_HOVER_1_STACK);
                        robot.armStackPosition = 1;
                        break;
                    default:
                        break;
                }
            }
        }
    }
}

