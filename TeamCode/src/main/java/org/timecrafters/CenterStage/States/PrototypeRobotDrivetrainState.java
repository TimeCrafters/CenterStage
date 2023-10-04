package org.timecrafters.CenterStage.States;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.timecrafters.CenterStage.Common.PrototypeRobot;

import dev.cyberarm.engine.V2.CyberarmState;

public class PrototypeRobotDrivetrainState extends CyberarmState {
    private PrototypeRobot robot;
    private int maxExtension = 2000;
    private int minExtension = 0;

    public PrototypeRobotDrivetrainState(PrototypeRobot robot) {
        this.robot = robot;
    }

    @Override
    public void exec() {
        // drivetrain
        robot.driveTrainTeleOp();

        // flip arms
        if (engine.gamepad1.x) {
            robot.depositorShoulder.setPosition(0.75);
        } else if (engine.gamepad1.a) {
            robot.depositorShoulder.setPosition(0.05);
        }

        if (engine.gamepad1.y){
            robot.depositorElbow.setPosition(0.33);
        } else if (engine.gamepad1.b){
            robot.depositorElbow.setPosition(0);
        }

        // depositor
        if (engine.gamepad1.right_bumper) {
            robot.depositor.setPosition(0.8);
        } else if (engine.gamepad1.left_bumper) {
            robot.depositor.setPosition(0.2);
        }


        // lift
        if (engine.gamepad1.right_trigger != 0){
            if (robot.lift.getCurrentPosition() >= maxExtension){
                robot.lift.motor.setPower(0);
            } else if (robot.lift.getCurrentPosition() >= maxExtension - 200){
                robot.lift.motor.setPower(0.35);
            }else {
                robot.lift.motor.setPower(engine.gamepad1.right_trigger);
            }
        } else if (engine.gamepad1.left_trigger != 0){

            if (robot.lift.getCurrentPosition() <= minExtension) {
                robot.lift.motor.setPower(0);
            } else if (robot.lift.getCurrentPosition() < 350){
                robot.lift.motor.setPower(-0.3);
            }else {
                robot.lift.motor.setPower(-engine.gamepad1.left_trigger);
            }
        } else {
            robot.lift.motor.setPower(0);
        }

    }


    @Override
    public void telemetry() {
        engine.telemetry.addData("Lift Encoder Pos", robot.lift.motor.getCurrentPosition());
        engine.telemetry.addData("imu", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }
}
