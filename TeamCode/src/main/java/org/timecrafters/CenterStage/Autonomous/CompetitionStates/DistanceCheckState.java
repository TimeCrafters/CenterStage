package org.timecrafters.CenterStage.Autonomous.CompetitionStates;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.timecrafters.CenterStage.Common.CompetitionRobotV1;

import dev.cyberarm.engine.V2.CyberarmState;

@Config
public class DistanceCheckState extends CyberarmState {

    CompetitionRobotV1 robot;
    public long lastCheckedTIme = System.currentTimeMillis();
    public String objectPosCheck;

    public DistanceCheckState(CompetitionRobotV1 robot, String groupName, String actionName) {
        this.robot = robot;
        this.objectPosCheck = robot.configuration.variable(groupName, actionName, "objectPosCheck").value();
    }

    @Override
    public void exec() {
        // odometry driving ALWAYS
        robot.DriveToCoordinates();
        robot.OdometryLocalizer();

        if (robot.armPos.equals("right")  || robot.armPos.equals("middle") || robot.armPos.equals("left")){
            setHasFinished(true);
        } else {
            if (Math.abs(robot.backLeftPower) < 0.15 &&
                Math.abs(robot.backRightPower) < 0.15 &&
                Math.abs(robot.frontLeftPower) < 0.15 &&
                Math.abs(robot.frontRightPower) < 0.15) {
                if (robot.customObject.getDistance(DistanceUnit.MM) < 1000 && System.currentTimeMillis() - lastCheckedTIme > 1000) {
                    robot.objectPos = objectPosCheck;
                    setHasFinished(true);
                } else if (robot.customObject.getDistance(DistanceUnit.MM) > 1000 && System.currentTimeMillis() - lastCheckedTIme > 1000) {
                    if (robot.loopCheck == 1 && robot.objectPos != objectPosCheck){
                        robot.objectPos = "left";
                        setHasFinished(true);
                    }
                    robot.loopCheck += 1;
                    setHasFinished(true);
                }
            }
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
