package dev.cyberarm.minibots.red_crab.states;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.engine.V2.Utilities;
import dev.cyberarm.minibots.red_crab.RedCrabMinibot;

public class Pilot extends CyberarmState {
    private final RedCrabMinibot robot;

    private boolean leftClawOpen = false;
    private boolean rightClawOpen = false;
    private int clawArmPosition = RedCrabMinibot.ClawArm_INITIAL;
    private boolean hookArmUp = false;
    private boolean droneLaunchAuthorized = false;
    private boolean droneLaunchRequested = false;
    private double droneLastLaunchRequestStartMS = 0;
    private double odometryResetRequestStartMS = 0;
    private boolean odometryResetRequested = false;
    private boolean odometryResetRequestLeftStick = false;
    private boolean odometryResetRequestRightStick = false;
    private boolean robotSlowMode = false;
    private boolean triggersControlClawArm = false;
    private double deltaTime = 0;
    private double lastLoopTimeMS = 0;

    public Pilot(RedCrabMinibot robot) {
        this.robot = robot;
    }

    @Override
    public void exec() {
        deltaTime = runTime() - lastLoopTimeMS;

        drivetrain();

        clawArmAndWristController();
        clawController();
        droneLatchController();
        hookArmController(); // disabled for swrist debug
        winchController();
        odometryDebugController();

        lastLoopTimeMS = runTime();
    }

    @Override
    public void buttonDown(Gamepad gamepad, String button) {
        if (gamepad == engine.gamepad1) {
            switch (button) {
                case "y":
                    droneLaunchRequested = true;
                    droneLastLaunchRequestStartMS = runTime();
                    break;
                case "x":
                    hookArmUp = true;
                    break;
                case "b":
                    hookArmUp = false;
                    break;
                case "guide":
                    robot.imu.resetYaw();
                    robot.ams.addNotice("IMU", "IMU RESET");
                    engine.telemetry.speak("IMU. RESET.");
                    break;
                case "left_bumper":
                    leftClawOpen = !leftClawOpen;
                    break;
                case "right_bumper":
                    rightClawOpen = !rightClawOpen;
                    break;
                case "dpad_up":
                    clawArmPosition = RedCrabMinibot.ClawArm_COLLECT;
                    break;
                case "dpad_down":
                    clawArmPosition = RedCrabMinibot.ClawArm_STOW;
                    break;
                case "dpad_left":
                    clawArmPosition = RedCrabMinibot.ClawArm_DEPOSIT;
                    break;
                case "dpad_right":
                    clawArmPosition = RedCrabMinibot.ClawArm_COLLECT_FLOAT;
                    break;
                case "back":
                    // FIXME: trigger controls don't yet work well.
                    triggersControlClawArm = false; //!triggersControlClawArm;

                    if (triggersControlClawArm)
                    {
                        robot.ams.addNotice("CLAW ARM", "Claw Arm Manual");
                        engine.telemetry.speak("Claw. Arm. Manual.");
                    } else {
                        robot.ams.addNotice("WINCH", "Winch Manual");
                        engine.telemetry.speak("Winch. Manual.");
                    }
                    break;
                case "start":
                    robot.reloadConfig();
                    break;
                case "left_stick_button":
                    odometryResetRequestLeftStick = true;
                    break;
                case "right_stick_button":
                    odometryResetRequestRightStick = true;
                    break;
            }
        }
    }

    @Override
    public void buttonUp(Gamepad gamepad, String button) {
        if (gamepad == engine.gamepad1) {
            switch (button) {
                case "y":
                    droneLaunchRequested = false;
                    droneLastLaunchRequestStartMS = runTime();
                    break;
                case "left_stick_button":
                    odometryResetRequestLeftStick = false;
                    break;
                case "right_stick_button":
                    odometryResetRequestRightStick = false;
                    break;
            }
        }
    }

    @Override
    public void telemetry() {
        robot.standardTelemetry();
    }

    private void drivetrain() {
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

        double maxVelocity = Utilities.unitToTicks(
                RedCrabMinibot.DRIVETRAIN_MOTOR_TICKS_PER_REVOLUTION,
                RedCrabMinibot.DRIVETRAIN_GEAR_RATIO,
                RedCrabMinibot.DRIVETRAIN_WHEEL_DIAMETER_MM,
                DistanceUnit.MM,
                RedCrabMinibot.DRIVETRAIN_VELOCITY_MAX_MM);
        double slowVelocity = Utilities.unitToTicks(
                RedCrabMinibot.DRIVETRAIN_MOTOR_TICKS_PER_REVOLUTION,
                RedCrabMinibot.DRIVETRAIN_GEAR_RATIO,
                RedCrabMinibot.DRIVETRAIN_WHEEL_DIAMETER_MM,
                DistanceUnit.MM,
                RedCrabMinibot.DRIVETRAIN_VELOCITY_SLOW_MM);
        double velocity = robotSlowMode ? slowVelocity : maxVelocity;

        robot.frontLeft.setVelocity(frontLeftPower * velocity);
        robot.backLeft.setVelocity(backLeftPower * velocity);
        robot.frontRight.setVelocity(frontRightPower * velocity);
        robot.backRight.setVelocity(backRightPower * velocity);
    }

    private void clawArmAndWristController() {
        if (triggersControlClawArm)
        {
            double triggerPower = engine.gamepad1.right_trigger - engine.gamepad1.left_trigger;

            robot.clawArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            double maxVelocity = Utilities.motorAngleToTicks(
                    RedCrabMinibot.CLAW_ARM_MOTOR_TICKS_PER_REVOLUTION,
                    RedCrabMinibot.CLAW_ARM_MOTOR_GEAR_RATIO,
                    RedCrabMinibot.CLAW_ARM_MAX_VELOCITY_DEGREES);
            robot.clawArm.setTargetPosition((int)(deltaTime * maxVelocity));

            robot.clawArm.setVelocity(maxVelocity * triggerPower);
            return;
        } else {
            robot.clawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        switch (clawArmPosition) {
            case RedCrabMinibot.ClawArm_STOW:
                robot.clawArm.setTargetPosition(Utilities.motorAngleToTicks(
                        RedCrabMinibot.CLAW_ARM_MOTOR_TICKS_PER_REVOLUTION,
                        RedCrabMinibot.CLAW_ARM_MOTOR_GEAR_RATIO,
                        RedCrabMinibot.CLAW_ARM_STOW_ANGLE));

                robot.clawWrist.setPosition(RedCrabMinibot.CLAW_WRIST_STOW_POSITION);
                break;
            case RedCrabMinibot.ClawArm_DEPOSIT:
                robot.clawArm.setTargetPosition(Utilities.motorAngleToTicks(
                        RedCrabMinibot.CLAW_ARM_MOTOR_TICKS_PER_REVOLUTION,
                        RedCrabMinibot.CLAW_ARM_MOTOR_GEAR_RATIO,
                        RedCrabMinibot.CLAW_ARM_DEPOSIT_ANGLE));

                robot.clawWrist.setPosition(RedCrabMinibot.CLAW_WRIST_DEPOSIT_POSITION);
                break;
            case RedCrabMinibot.ClawArm_COLLECT_FLOAT:
                robot.clawArm.setTargetPosition(Utilities.motorAngleToTicks(
                        RedCrabMinibot.CLAW_ARM_MOTOR_TICKS_PER_REVOLUTION,
                        RedCrabMinibot.CLAW_ARM_MOTOR_GEAR_RATIO,
                        RedCrabMinibot.CLAW_ARM_COLLECT_FLOAT_ANGLE));

                robot.clawWrist.setPosition(RedCrabMinibot.CLAW_WRIST_COLLECT_FLOAT_POSITION);
                break;
            case RedCrabMinibot.ClawArm_COLLECT:
                robot.clawArm.setTargetPosition(Utilities.motorAngleToTicks(
                        RedCrabMinibot.CLAW_ARM_MOTOR_TICKS_PER_REVOLUTION,
                        RedCrabMinibot.CLAW_ARM_MOTOR_GEAR_RATIO,
                        RedCrabMinibot.CLAW_ARM_COLLECT_ANGLE));

                robot.clawWrist.setPosition(RedCrabMinibot.CLAW_WRIST_COLLECT_POSITION);
                break;
        }
    }

    private void clawController() {
        robot.leftClaw.setPosition((leftClawOpen ? RedCrabMinibot.CLAW_LEFT_OPEN_POSITION : RedCrabMinibot.CLAW_LEFT_CLOSED_POSITION));
        robot.rightClaw.setPosition((rightClawOpen ? RedCrabMinibot.CLAW_RIGHT_OPEN_POSITION : RedCrabMinibot.CLAW_RIGHT_CLOSED_POSITION));
    }

    private void droneLatchController() {
        if (droneLaunchRequested && runTime() - droneLastLaunchRequestStartMS >= RedCrabMinibot.DRONE_LAUNCH_CONFIRMATION_TIME_MS)
            droneLaunchAuthorized = true;

        if (droneLaunchAuthorized) {
            robot.droneLatch.setPosition(RedCrabMinibot.DRONE_LATCH_LAUNCH_POSITION);
        }

        // Auto reset drone latch after DRONE_LAUNCH_CONFIRMATION_TIME_MS + 1 second
        if (!droneLaunchRequested && runTime() - droneLastLaunchRequestStartMS >= RedCrabMinibot.DRONE_LAUNCH_CONFIRMATION_TIME_MS + 1_000) {
            droneLaunchAuthorized = false;

            robot.droneLatch.setPosition(RedCrabMinibot.DRONE_LATCH_INITIAL_POSITION);
        }
    }

    private void hookArmController() {
        robot.hookArm.setPosition((hookArmUp ? RedCrabMinibot.HOOK_ARM_UP_POSITION : RedCrabMinibot.HOOK_ARM_STOW_POSITION));
    }

    private void winchController() {
        if (triggersControlClawArm)
        {
            return;
        }

        if (engine.gamepad1.right_trigger > 0) {
            robot.winch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.winch.setPower(engine.gamepad1.right_trigger * RedCrabMinibot.WINCH_MAX_SPEED);
            robot.winch.setTargetPosition(robot.winch.getCurrentPosition());
        } else if (engine.gamepad1.left_trigger > 0) {
            robot.winch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.winch.setPower(-engine.gamepad1.left_trigger * RedCrabMinibot.WINCH_MAX_SPEED);
            robot.winch.setTargetPosition(robot.winch.getCurrentPosition());
        } else {
            robot.winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.winch.setPower(RedCrabMinibot.WINCH_MAX_SPEED);
        }
    }

    private void odometryDebugController() {
        if (odometryResetRequestLeftStick && odometryResetRequestRightStick) {
            if (!odometryResetRequested) {
                odometryResetRequested = true;
                odometryResetRequestStartMS = runTime();
            } else if (runTime() - odometryResetRequestStartMS >= 1_000) {
                RedCrabMinibot.localizer.reset();
                odometryResetRequestStartMS = runTime();
                odometryResetRequested = false;
                robot.ams.addNotice("ODOMETRY", "Odometry Reset");
                engine.telemetry.speak("ODOMETRY. RESET.");
            }
        } else {
            odometryResetRequested = false;
            odometryResetRequestStartMS = runTime();
        }
    }
}
