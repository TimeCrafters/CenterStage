package org.timecrafters.CenterStage.TeleOp.States;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.timecrafters.CenterStage.Common.PrototypeRobot;
import org.timecrafters.CenterStage.Common.XDrivetrainBot;

import dev.cyberarm.engine.V2.CyberarmState;
@Config
public class XDrivetrainRobotState extends CyberarmState {
    private XDrivetrainBot robot;
    private PIDController ArmPidController;
    public static double ArmP, ArmI, ArmD;
    private double targetHeading;
    private double currentHeading;
    public double integralSum = 0;
    public static double Kp = 1;
    public static double Ki = 0;
    public static double Kd = 0;
    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private boolean headingLock = false;

    private long lastCheckedTime;

    public XDrivetrainRobotState(XDrivetrainBot robot) {
        this.robot = robot;
        ArmPidController = new PIDController(ArmP, ArmI, ArmD);


    }

    //    }
    // --------------------------------------------------------------------------------------------------------- Slider control function
    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }

    public void ArmControl(){

    }

    public double HeadingPIDControl(double reference, double current){
        double error = angleWrap(current - reference);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }

    @Override
    public void init() {
    }

    @Override
    public void exec() {
        currentHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if (engine.gamepad1.right_stick_button) {
            robot.imu.resetYaw();
        }

        if (headingLock){
            robot.rx = HeadingPIDControl(targetHeading, currentHeading);
        } else {
            robot.rx = engine.gamepad1.right_stick_x;

        }
            // drivetrain
            robot.driveTrainTeleOp();

            if (engine.gamepad1.b){
                headingLock = true;
                targetHeading = robot.backDropLock;
            }
            if (engine.gamepad1.x){
                headingLock = true;
                targetHeading = robot.collectLock;
            }
            if (engine.gamepad1.a){
                headingLock = true;
                targetHeading = currentHeading;
            }
            if (engine.gamepad1.right_stick_x != 0){
                headingLock = false;
            }

    }

        @Override
        public void telemetry () {
            engine.telemetry.addData("imu", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            engine.telemetry.addData("arm Pos", robot.armPosition);
            engine.telemetry.addData("old arm pos", robot.oldArmPosition);
            engine.telemetry.addData("depositor pos", robot.depositorPos);
            engine.telemetry.addData("collector pos", robot.collectorPos);
            engine.telemetry.addData("time", System.currentTimeMillis() - robot.startOfSequencerTime);
            engine.telemetry.addData("heading lock?", headingLock);
            engine.telemetry.addData("pid heading power", HeadingPIDControl(targetHeading, currentHeading));
        }
    }

