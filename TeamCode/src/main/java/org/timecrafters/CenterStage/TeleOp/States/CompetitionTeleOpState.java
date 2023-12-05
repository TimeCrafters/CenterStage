package org.timecrafters.CenterStage.TeleOp.States;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.timecrafters.CenterStage.Common.PrototypeRobot;

import dev.cyberarm.engine.V2.CyberarmState;

@Config

public class CompetitionTeleOpState extends CyberarmState {
    private PrototypeRobot robot;
    private int maxExtension = 2000;
    private int minExtension = 0;
    public double integralSum = 0;
    private double targetHeading;

    public double power;
    private double currentHeading;
    private boolean headingLock = false;

    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    private double lastError = 0;
    ElapsedTime timer = new ElapsedTime();


    private long lastCheckedTime;

    public CompetitionTeleOpState(PrototypeRobot robot) {
        this.robot = robot;

    }

    //    }
    // --------------------------------------------------------------------------------------------------------- Slider control function
    private void SliderTeleOp() {
        if (engine.gamepad2.right_trigger != 0) {
            if (robot.lift.getCurrentPosition() >= maxExtension) {
                robot.lift.setPower(0);
            } else if (robot.lift.getCurrentPosition() >= maxExtension - 200) {
                robot.lift.setPower(0.35);
            } else {
                robot.lift.setPower(engine.gamepad2.right_trigger);
            }
        } else if (engine.gamepad2.left_trigger != 0) {

            if (robot.lift.getCurrentPosition() <= minExtension) {
                robot.lift.setPower(0);
            } else if (robot.lift.getCurrentPosition() < 350) {
                robot.lift.setPower(-0.3);
            } else {
                robot.lift.setPower(-engine.gamepad2.left_trigger);
            }
        } else {
            robot.lift.setPower(0);
        }
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

//        if (engine.gamepad2.a) {
//            robot.armPosition = 0;
//            robot.startOfSequencerTime = System.currentTimeMillis();
//        } else if (engine.gamepad2.x) {
//            robot.armPosition = 1;
//            robot.startOfSequencerTime = System.currentTimeMillis();
//        } else if (engine.gamepad2.b) {
//            robot.armPosition = 2;
//            robot.startOfSequencerTime = System.currentTimeMillis();
//        } else if (engine.gamepad2.y) {
//            robot.armPosition = 3;
//            robot.startOfSequencerTime = System.currentTimeMillis();
//        }
//
//        robot.depositor.setPosition(robot.depositorPos);
//        robot.collector.setPosition(robot.collectorPos);
//
//
//            // drivetrain
//            robot.driveTrainTeleOp();
            // lift
//            SliderTeleOp();
            // collector depositor
//            robot.CollectorToggle();
            // depositor toggle
//            robot.DepositorToggle();


    }

        @Override
        public void telemetry () {
            engine.telemetry.addData("Lift Encoder Pos", robot.lift.getCurrentPosition());
            engine.telemetry.addData("imu", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            engine.telemetry.addData("arm Pos", robot.armPosition);
            engine.telemetry.addData("old arm pos", robot.oldArmPosition);
            engine.telemetry.addData("depositor pos", robot.depositorPos);
            engine.telemetry.addData("collector pos", robot.collectorPos);
            engine.telemetry.addData("time", System.currentTimeMillis() - robot.startOfSequencerTime);
            engine.telemetry.addData("pid power", power);
            engine.telemetry.addData("heading Lock?", headingLock);
            engine.telemetry.addData("Kp", Kp);
            engine.telemetry.addData("Ki", Ki);
            engine.telemetry.addData("Kd", Kd);
        }
    }

