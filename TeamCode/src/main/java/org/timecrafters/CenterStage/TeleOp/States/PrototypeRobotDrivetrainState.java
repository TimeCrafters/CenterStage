package org.timecrafters.CenterStage.TeleOp.States;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.timecrafters.CenterStage.Common.PrototypeRobot;

import dev.cyberarm.engine.V2.CyberarmState;

public class PrototypeRobotDrivetrainState extends CyberarmState {
    private PrototypeRobot robot;
    private int maxExtension = 2000;
    private int minExtension = 0;
    private PIDController HeadingPidController;
    private double targetHeading;
    public double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    private double lastError = 0;
    ElapsedTime timer = new ElapsedTime();


    private long lastCheckedTime;

    public PrototypeRobotDrivetrainState(PrototypeRobot robot) {
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

    private void HeadingLock(){
        double currentHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

    }
    public double HeadingPIDControl(double reference, double current){
        double error = angleWrap(reference - current);
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

        if (engine.gamepad1.right_stick_button) {
            robot.imu.resetYaw();
        }

        if (engine.gamepad2.a) {
            robot.armPosition = 0;
            robot.startOfSequencerTime = System.currentTimeMillis();
        } else if (engine.gamepad2.x) {
            robot.armPosition = 1;
            robot.startOfSequencerTime = System.currentTimeMillis();
        } else if (engine.gamepad2.b) {
            robot.armPosition = 2;
            robot.startOfSequencerTime = System.currentTimeMillis();
        } else if (engine.gamepad2.y) {
            robot.armPosition = 3;
            robot.startOfSequencerTime = System.currentTimeMillis();
        }

        robot.depositor.setPosition(robot.depositorPos);
        robot.collector.setPosition(robot.collectorPos);


        // arm sequencer
//        robot.ArmSequences();
//        switch (robot.armPosition) {
//            case 0: // ----------------------------------------------------------------------------------------------- drive to transfer pos
//                switch (robot.oldArmPosition) {
//                    case 0:
//                        // transfer
//                        robot.oldArmPosition = 0;
//                        break;
//                    case 1:
//                        // driving
//                        robot.collectorShoulder.setPosition(robot.COLLECTOR_SHOULDER_IN); // drive the shoulder to the transfer position
//                        if (System.currentTimeMillis() - robot.startOfSequencerTime >= 800) { // wait to move till time is met
//                            robot.collectorElbow.setPosition(robot.COLLECTOR_ELBOW_IN);
//                            robot.oldArmPosition = 0;
//                        }
//                        break;
//                    case 2:
//                        // collect
//                        robot.collectorShoulder.setPosition(robot.COLLECTOR_SHOULDER_IN); // drive the shoulder to the transfer position
//                        if (System.currentTimeMillis() - robot.startOfSequencerTime >= 750) { // wait to move till time is met
//                            robot.collectorElbow.setPosition(robot.COLLECTOR_ELBOW_IN);
//                            if (System.currentTimeMillis() - robot.startOfSequencerTime >= 1500) {
//                                robot.oldArmPosition = 0;
//                            }
//                        }
//                        break;
//                    case 3:
//                        // deposit
//                        robot.collectorShoulder.setPosition(robot.COLLECTOR_SHOULDER_OUT); // drive the shoulder to the out position
//                        robot.depositorElbow.setPosition(robot.DEPOSITOR_ELBOW_IN);
//                        if (System.currentTimeMillis() - robot.startOfSequencerTime >= 800) { // wait to move till time is met
//                            robot.depositorShoulder.setPosition(robot.DEPOSITOR_SHOULDER_IN); // drive the shoulder to the transfer position
//                            if (System.currentTimeMillis() - robot.startOfSequencerTime >= 1100) {
//                                robot.collectorShoulder.setPosition(robot.COLLECTOR_SHOULDER_IN); // drive the shoulder to the transfer position
//                                if (System.currentTimeMillis() - robot.startOfSequencerTime >= 1500) { // wait to move till time is met
//                                    robot.collectorElbow.setPosition(robot.COLLECTOR_ELBOW_IN);
//                                    if (System.currentTimeMillis() - robot.startOfSequencerTime >= 2000) {
//                                        robot.oldArmPosition = 0;
//                                    }
//                                }
//                            }
//                        }
//                        break;
//                }
//                break;
//            case 1:// ----------------------------------------------------------------------------------------------- drive to driving pos
//                switch (robot.oldArmPosition) {
//                    case 0:
//                        robot.collectorShoulder.setPosition(robot.COLLECTOR_SHOULDER_PASSIVE); // drive the shoulder to the transfer position
//                        if (System.currentTimeMillis() - robot.startOfSequencerTime >= 800) { // wait to move till time is met
//                            robot.collectorElbow.setPosition(robot.COLLECTOR_ELBOW_PASSIVE);
//                            if (System.currentTimeMillis() - robot.startOfSequencerTime >= 1500) {
//                                robot.oldArmPosition = 1;
//                            }
//                        }
//                            break;
//                    case 1:
//                        // drive pos
//                        robot.oldArmPosition = 1;
//
//                        break;
//                    case 2:
//                        // collect
//                        robot.collectorShoulder.setPosition(robot.COLLECTOR_SHOULDER_PASSIVE); // drive the shoulder to the transfer position
//                        if (System.currentTimeMillis() - robot.startOfSequencerTime >= 900) { // wait to move till time is met
//                            robot.collectorElbow.setPosition(robot.COLLECTOR_ELBOW_PASSIVE);
//                            if (System.currentTimeMillis() - robot.startOfSequencerTime >= 1500) {
//                                robot.oldArmPosition = 1;
//                            }
//                        }
//                        break;
//                    case 3:
//                        // deposit
//                        robot.collectorShoulder.setPosition(robot.COLLECTOR_SHOULDER_OUT - 0.1); // drive the shoulder to the out position
//                        robot.depositorElbow.setPosition(robot.DEPOSITOR_ELBOW_IN);
//                        if (System.currentTimeMillis() - robot.startOfSequencerTime >= 800) { // wait to move till time is met
//                            robot.depositorShoulder.setPosition(robot.DEPOSITOR_SHOULDER_IN); // drive the shoulder to the transfer position
//                            if (System.currentTimeMillis() - robot.startOfSequencerTime >= 1100) {
//                                robot.collectorElbow.setPosition(robot.COLLECTOR_ELBOW_IN);
//                                robot.oldArmPosition = 1;
//                            }
//                        }
//                        break;
//                }
//                break;
//
//            case 2:// ----------------------------------------------------------------------------------------------- drive to collect pos
//                switch (robot.oldArmPosition) {
//                    case 0:
//                        // transfer
//                        robot.collectorElbow.setPosition(robot.COLLECTOR_ELBOW_OUT); // drive the shoulder to the transfer position
//                        if (System.currentTimeMillis() - robot.startOfSequencerTime >= 750) { // wait to move till time is met
//                            robot.collectorShoulder.setPosition(robot.COLLECTOR_SHOULDER_OUT);
//                            if (System.currentTimeMillis() - robot.startOfSequencerTime >= 2000) {
//                                robot.oldArmPosition = 2;
//                            }
//                        }
//                        break;
//                    case 1:
//                        // driving
//                        robot.collectorElbow.setPosition(robot.COLLECTOR_ELBOW_OUT); // drive the shoulder to the transfer position
//                        if (System.currentTimeMillis() - robot.startOfSequencerTime >= 750) { // wait to move till time is met
//                            robot.collectorShoulder.setPosition(robot.COLLECTOR_SHOULDER_OUT);
//                            if (System.currentTimeMillis() - robot.startOfSequencerTime >= 1500) {
//                                robot.oldArmPosition = 2;
//                            }
//                        }
//                        break;
//                    case 2:
//                        // collect
//                        break;
//                    case 3:
//                        // deposit
//                        robot.depositorShoulder.setPosition(robot.DEPOSITOR_SHOULDER_IN); // drive the shoulder to the transfer position
//                        if (System.currentTimeMillis() - robot.startOfSequencerTime >= 100) { // wait to move till time is met
//                            robot.depositorElbow.setPosition(robot.DEPOSITOR_ELBOW_IN);
//                            if (System.currentTimeMillis() - robot.startOfSequencerTime >= 100) {
//                                if (System.currentTimeMillis() - robot.startOfSequencerTime >= 100) { // wait to move till time is met
//                                    robot.collectorElbow.setPosition(robot.COLLECTOR_ELBOW_OUT);
//                                    if (System.currentTimeMillis() - robot.startOfSequencerTime >= 700) {
//                                        robot.collectorShoulder.setPosition(robot.COLLECTOR_SHOULDER_OUT);
//                                        if (System.currentTimeMillis() - robot.startOfSequencerTime >= 1000) {
//                                            robot.oldArmPosition = 2;
//                                        }
//                                    }
//                                }
//                            }
//                        }
//                }
//                        break;
//                    case 3:// ----------------------------------------------------------------------------------------------- drive to deposit pos
//                switch (robot.oldArmPosition) {
//                    case 0:
//                        // transfer
//                        robot.collectorElbow.setPosition(robot.COLLECTOR_ELBOW_OUT); // drive the shoulder to the transfer position
//                        if (System.currentTimeMillis() - robot.startOfSequencerTime >= 750) { // wait to move till time is met
//                            robot.collectorShoulder.setPosition(robot.COLLECTOR_SHOULDER_PASSIVE);
//                            if (System.currentTimeMillis() - robot.startOfSequencerTime >= 1200) {
//                                if (System.currentTimeMillis() - robot.startOfSequencerTime >= 1500){
//                                    robot.depositorShoulder.setPosition(robot.DEPOSITOR_SHOULDER_OUT);
//                                    if (System.currentTimeMillis() - robot.startOfSequencerTime >= 2000){
//                                        robot.depositorElbow.setPosition(robot.DEPOSITOR_ELBOW_OUT);
//                                        if (System.currentTimeMillis() - robot.startOfSequencerTime >= 2300){
//                                            robot.oldArmPosition = 3;
//                                        }
//                                    }
//                                }
//                            }
//                        }
//                        break;
//                    case 1:
//                        robot.collectorElbow.setPosition(robot.COLLECTOR_ELBOW_OUT); // drive the shoulder to the transfer position
//                        if (System.currentTimeMillis() - robot.startOfSequencerTime >= 760) { // wait to move till time is met
//                            robot.collectorShoulder.setPosition(robot.COLLECTOR_SHOULDER_PASSIVE);
//                            if (System.currentTimeMillis() - robot.startOfSequencerTime >= 1200) {
//                                if (System.currentTimeMillis() - robot.startOfSequencerTime >= 1500){
//                                    robot.depositorShoulder.setPosition(robot.DEPOSITOR_SHOULDER_OUT);
//                                    if (System.currentTimeMillis() - robot.startOfSequencerTime >= 2000){
//                                        robot.depositorElbow.setPosition(robot.DEPOSITOR_ELBOW_OUT);
//                                        if (System.currentTimeMillis() - robot.startOfSequencerTime >= 2300){
//                                            robot.oldArmPosition = 3;
//                                        }
//                                    }
//                                }
//                            }
//                        }
//                        break;
//                    case 2:
//                        // collect
//                        robot.collectorShoulder.setPosition(robot.COLLECTOR_SHOULDER_IN); // drive the shoulder to the transfer position
//                        if (System.currentTimeMillis() - robot.startOfSequencerTime >= 750) { // wait to move till time is met
//                            robot.collectorElbow.setPosition(robot.COLLECTOR_ELBOW_IN);
//                            if (System.currentTimeMillis() - robot.startOfSequencerTime >= 1500) {
//                                robot.oldArmPosition = 3;
//                            }
//                        }
//                        break;
//                    case 3:
//                        // deposit
//                        break;
//                }
//                break;

//        }

            // drivetrain
            robot.driveTrainTeleOp();

            if (engine.gamepad1.b){
                targetHeading = robot.backDropLock;

            }
            // lift
            SliderTeleOp();
            // collector depositor
            robot.CollectorToggle();
            // depositor toggle
            robot.DepositorToggle();


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
        }
    }

