package org.timecrafters.CenterStage.TeleOp.States;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.timecrafters.CenterStage.Common.PrototypeRobot;

import dev.cyberarm.engine.V2.CyberarmState;


@Config
public class headingLockTeleOp extends CyberarmState {
    private PrototypeRobot robot;
    private PIDController HeadingPidController;
    public double integralSum = 0;
    public static double Kp = 1;
    public static double Ki = 0;
    public static double Kd = 0;
    private double lastError = 0;
    ElapsedTime timer = new ElapsedTime();



    public headingLockTeleOp(PrototypeRobot robot) {
        this.robot = robot;

    }

    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }

    @Override
    public void init() {
        robot.headingLock = false;
    }

    @Override
    public void exec() {
        if (robot.headingLock) {
            double currentHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double error = angleWrap(robot.targetHeading - currentHeading);
            integralSum += error * timer.seconds();
            double derivative = (error - lastError) / timer.seconds();

            timer.reset();

            double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
            robot.PIDrx = output;
        }

        // drivetrain
        robot.driveTrainTeleOp();




        if (engine.gamepad1.right_stick_button) {
            robot.imu.resetYaw();
        }

            if (engine.gamepad1.b){
                robot.headingLock = true;
                robot.targetHeading = robot.backDropLock;
            }

            if (engine.gamepad1.right_stick_x != 0){
                robot.headingLock = false;
            }

    }

        @Override
        public void telemetry () {
            engine.telemetry.addData("imu", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            engine.telemetry.addData("rx", robot.rx);
            engine.telemetry.addData("PIDrx", robot.PIDrx);
        }
    }

