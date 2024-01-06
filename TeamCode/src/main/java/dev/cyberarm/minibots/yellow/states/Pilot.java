package dev.cyberarm.minibots.yellow.states;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.engine.V2.Utilities;
import dev.cyberarm.minibots.yellow.YellowMinibot;

public class Pilot extends CyberarmState {
    final YellowMinibot robot;
    private boolean leftClawOpen, rightClawOpen;

    public Pilot(YellowMinibot robot) {
        this.robot = robot;

        this.leftClawOpen = false;
        this.rightClawOpen = false;
    }
    @Override
    public void exec() {
        drivetrain();
        armController();
        clawControllers();
        droneLatchController();
    }

    private void drivetrain()
    {
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

        double velocity = Utilities.unitToTicks(
                robot.DRIVETRAIN_MOTOR_TICKS,
                robot.DRIVETRAIN_MOTOR_GEAR_RATIO,
                robot.DRIVETRAIN_WHEEL_DIAMETER_MM,
                DistanceUnit.MM,
                robot.DRIVETRAIN_MAX_VELOCITY_MM);

        robot.backLeft.setVelocity(frontLeftPower * velocity);
        robot.frontLeft.setVelocity(backLeftPower * velocity);
        robot.frontRight.setVelocity(frontRightPower * velocity);
        robot.backRight.setVelocity(backRightPower * velocity);
    }

    private void armController()
    {
        double armPower = engine.gamepad1.right_trigger - engine.gamepad1.left_trigger;
        if (armPower > 0) {
            if (robot.armEndStopLeft.isPressed() || robot.armEndStopRight.isPressed())
                armPower = 0;
        }

        robot.arm.setPower(armPower);
    }

    private void clawControllers()
    {
        robot.leftClaw.setPosition(leftClawOpen ? robot.LEFT_CLAW_OPEN_POSITION : robot.LEFT_CLAW_CLOSED_POSITION);
        robot.rightClaw.setPosition(rightClawOpen ? robot.RIGHT_CLAW_OPEN_POSITION : robot.RIGHT_CLAW_CLOSED_POSITION);
    }

    private void droneLatchController()
    {}

    @Override
    public void telemetry() {
        robot.standardTelemetry();
        robot.teleopTelemetry();
    }

    @Override
    public void buttonDown(Gamepad gamepad, String button) {
        if (gamepad == engine.gamepad1)
        {
            switch (button) {
                case "guide":
                    /// --- IMU Reset --- ///
                    robot.imu.resetYaw();
                    break;
                case "start":
                    robot.reloadConfig();
                    break;
                case "left_bumper":
                    leftClawOpen = !leftClawOpen;
                    break;
                case "right_bumper":
                    rightClawOpen = !rightClawOpen;
                    break;
                case "y":
                    /// DRONE LATCH ///
                    robot.droneLatch.setPosition(robot.DRONE_LATCH_LAUNCH_POSITION);
                    break;
                case "a":
                    /// DRONE LATCH ///
                    robot.droneLatch.setPosition(robot.DRONE_LATCH_INITIAL_POSITION);
                    break;
            }
        }
    }
}
