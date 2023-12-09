package org.timecrafters.CenterStage.Autonomous.CompetitionStates;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.timecrafters.CenterStage.Common.CompetitionRobotV1;

import dev.cyberarm.engine.V2.CyberarmState;

@Config
public class ClawArmState extends CyberarmState {

    CompetitionRobotV1 robot;
    public String armPos;

    public ClawArmState(CompetitionRobotV1 robot, String groupName, String actionName) {
        this.robot = robot;
        this.armPos = robot.configuration.variable(groupName, actionName, "armPos").value();
    }

    @Override
    public void exec() {
        // odometry driving ALWAYS
        robot.DriveToCoordinates();
        robot.OdometryLocalizer();

        // driving arm to pos
        if (armPos.equals("collect")){
            if (robot.lift.getCurrentPosition() >= 1){
                robot.lift.setPower(-0.6);
            } else {
                robot.lift.setPower(0);
                robot.shoulder.setPosition(robot.shoulderCollect);
                robot.elbow.setPosition(robot.elbowCollect);
                robot.target = 10;

            }
        }
        if (armPos.equals("passive")){
            if (robot.lift.getCurrentPosition() >= 1){
                robot.lift.setPower(-0.6);
            } else {
                robot.lift.setPower(0);
                robot.shoulder.setPosition(robot.shoulderPassive);
                robot.elbow.setPosition(robot.elbowPassive);
                robot.target = 850;
            }
        }
        if (armPos.equals("deposit")){
            robot.shoulder.setPosition(robot.shoulderDeposit);
            robot.elbow.setPosition(robot.elbowDeposit);
            robot.target = 370;

        }
        if (armPos.equals("hover")) {
            robot.shoulder.setPosition(robot.shoulderCollect);
            robot.elbow.setPosition(robot.elbowCollect);
            robot.target = 150;

        }
        robot.armPos = armPos;
        robot.clawArmControl();

        if (robot.clawArm.getCurrentPosition() >= robot.target - 20 && robot.clawArm.getCurrentPosition() <= robot.target + 20) {
//            setHasFinished(true);
        }



    }


    @Override
    public void telemetry() {
        engine.telemetry.addData("x pos", robot.positionX);
        engine.telemetry.addData("y pos", robot.positionY);
        engine.telemetry.addData("h pos odo", Math.toDegrees(robot.positionH));
        engine.telemetry.addData("aux encoder", robot.currentAuxPosition);
        engine.telemetry.addData("left encoder", robot.currentLeftPosition);
        engine.telemetry.addData("right encoder", robot.currentRightPosition);
        engine.telemetry.addData("h pos imu", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        engine.telemetry.addData("front right power", robot.frontRightPower);
        engine.telemetry.addData("front left power", robot.frontLeftPower);
        engine.telemetry.addData("back right power", robot.backRightPower);
        engine.telemetry.addData("back left power", robot.backLeftPower);
        engine.telemetry.addData("arm pos", robot.clawArm.getCurrentPosition());

    }
}
