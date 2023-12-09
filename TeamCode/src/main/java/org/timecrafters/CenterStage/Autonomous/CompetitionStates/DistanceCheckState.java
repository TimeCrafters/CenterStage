package org.timecrafters.CenterStage.Autonomous.CompetitionStates;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.timecrafters.CenterStage.Common.CompetitionRobotV1;

import dev.cyberarm.engine.V2.CyberarmState;

@Config
public class DistanceCheckState extends CyberarmState {

    CompetitionRobotV1 robot;
    public String armPos;
    public long lastCheckedTIme = System.currentTimeMillis();

    public DistanceCheckState(CompetitionRobotV1 robot, String groupName, String actionName) {
        this.robot = robot;
    }

    @Override
    public void exec() {
        // odometry driving ALWAYS
        robot.DriveToCoordinates();
        robot.OdometryLocalizer();

        if (robot.customObject.getDistance(DistanceUnit.MM) < 1000 && robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > -47 && robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < -43){
            robot.objectPos = "Right";
        } else if (robot.customObject.getDistance(DistanceUnit.MM) > 1000){
            robot.hTarget = 0;
            }
        if (robot.objectPos != "right")
        if (Math.abs(robot.backLeftPower) < 0.15 &&
                Math.abs(robot.backRightPower) < 0.15 &&
                Math.abs(robot.frontLeftPower) < 0.15 &&
                Math.abs(robot.frontRightPower) < 0.15 && System.currentTimeMillis() - lastCheckedTIme > 2000){
            if (robot.customObject.getDistance(DistanceUnit.MM) < 1000 && System.currentTimeMillis() - lastCheckedTIme > 2000){
                robot.objectPos = "middle";
            } else {
                robot.hTarget = 45;
                robot.objectPos = "left";
            }
        }

        if (robot.objectPos.equals("left") || robot.objectPos.equals("right") || robot.objectPos.equals("middle")){
            setHasFinished(true);
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
        engine.telemetry.addData("distance ", robot.customObject.getDistance(DistanceUnit.MM));
        engine.telemetry.addData("object location ", robot.objectPos);
    }
}
