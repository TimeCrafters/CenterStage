package org.timecrafters.CenterStage.TeleOp.States;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.timecrafters.CenterStage.Common.PrototypeRobot;

import dev.cyberarm.engine.V2.CyberarmState;

public class PrototypeRobotDrivetrainState extends CyberarmState {
    private PrototypeRobot robot;
    private int maxExtension = 2000;
    private int minExtension = 0;

    private long lastCheckedTime;

    public PrototypeRobotDrivetrainState(PrototypeRobot robot) {
        this.robot = robot;
    }

//    }
    // --------------------------------------------------------------------------------------------------------- Slider control function
    private void sliderTeleOp(){
        if (engine.gamepad1.right_trigger != 0){
            if (robot.lift.getCurrentPosition() >= maxExtension){
                robot.lift.setPower(0);
            } else if (robot.lift.getCurrentPosition() >= maxExtension - 200){
                robot.lift.setPower(0.35);
            }else {
                robot.lift.setPower(engine.gamepad1.right_trigger);
            }
        } else if (engine.gamepad1.left_trigger != 0){

            if (robot.lift.getCurrentPosition() <= minExtension) {
                robot.lift.setPower(0);
            } else if (robot.lift.getCurrentPosition() < 350){
                robot.lift.setPower(-0.3);
            }else {
                robot.lift.setPower(-engine.gamepad1.left_trigger);
            }
        } else {
            robot.lift.setPower(0);
        }
    }

    @Override
    public void exec() {
        if (engine.gamepad2.a){
            robot.armPosition = 0;
        } else if (engine.gamepad2.x){
            robot.armPosition = 1;
        } else if (engine.gamepad2.b){
            robot.armPosition = 2;
        } else if (engine.gamepad2.y){
            robot.armPosition = 3;
        }

        robot.depositor.setPosition(robot.depositorPos);
        robot.collector.setPosition(robot.collectorPos);

        // drivetrain
        robot.driveTrainTeleOp();
        // arm sequencer
        robot.ArmSequences();
        // lift
        sliderTeleOp();
        // collector depositor
        robot.CollectorToggle();
        // depositor toggle
        robot.DepositorToggle();

    }


    @Override
    public void telemetry() {
        engine.telemetry.addData("Lift Encoder Pos", robot.lift.getCurrentPosition());
        engine.telemetry.addData("imu", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        engine.telemetry.addData("arm Pos", robot.armPosition);
        engine.telemetry.addData("old arm pos", robot.oldArmPosition);
        engine.telemetry.addData("depositor pos", robot.depositorPos);
        engine.telemetry.addData("collector pos", robot.collectorPos);
    }
}
