package org.timecrafters.CenterStage.Autonomous.States;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.timecrafters.CenterStage.Common.SodiPizzaMinibotObject;
import org.timecrafters.TimeCraftersConfigurationTool.library.TimeCraftersConfiguration;

import dev.cyberarm.engine.V2.CyberarmState;

public class SodiPizzaAutoTurnState extends CyberarmState {
    final private SodiPizzaMinibotObject robot = new SodiPizzaMinibotObject();
    final private String groupName, actionName;
    private long lastMoveTime;
    private double turnSpeedRaw, turnSpeed;
    private int startPos;
    private double targetRot;
    private double currentRot;
    private double neededRot;
    public int readyToTurn;
    
//    private double rightTurnCW = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + 90;
//    private double rightTurnCCW = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - 90;
//    private double backTurnCW = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + 180;
//    private double backTurnCCW = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - 180;

  /** Rot = rotation **/

    public SodiPizzaAutoTurnState() {
        groupName = " ";
        actionName = " ";
        robot.setup();
    }

    private double getTurnSpeed() {
        if (Math.abs(neededRot) > 5) {
            turnSpeed = (turnSpeedRaw * neededRot) / 10;
        }
            return turnSpeed;
    }

    public void CalculateNeededRot() {
        if (targetRot >= 0 && currentRot >= 0) {
            neededRot = Math.abs(targetRot - currentRot);
        } else if (targetRot <= 0 && currentRot <= 0) {
            neededRot = Math.abs(targetRot - currentRot);
        } else if (targetRot >= 0 && currentRot <= 0) {
            neededRot = (targetRot + Math.abs(currentRot));
        } else if (targetRot <= 0 && currentRot >= 0) {
            neededRot = (currentRot + Math.abs(targetRot));
        }
    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("Target Rotation", targetRot);
        engine.telemetry.addData("Current Rotation", currentRot);
        engine.telemetry.addData("Power", robot.leftFront.getPower());
        engine.telemetry.addData("Distance Sensor Reading", robot.distSensor.getDistance(DistanceUnit.MM));
        engine.telemetry.addLine();
        engine.telemetry.addData("Internal Ready To Turn Value", readyToTurn);
        engine.telemetry.addData("Engine Ready To Turn Value", engine.blackboardGetInt("readyToTurn"));


    }

    @Override
    public void start() {

        turnSpeedRaw = 0;
        getTurnSpeed();

        robot.leftFront.setPower(turnSpeed);
        robot.leftBack.setPower(turnSpeed);
        robot.rightFront.setPower(turnSpeed);
        robot.rightBack.setPower(turnSpeed);

        engine.blackboardSet("readyToTurn", 0);

    }


    @Override
    public void exec() {

        readyToTurn = engine.blackboardGet("readyToTurn");

        targetRot = -90;

        currentRot = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        CalculateNeededRot();

        if (readyToTurn == 1 && Math.abs(neededRot) > 10) {

            turnSpeedRaw = 0.3;
            getTurnSpeed();

            robot.leftFront.setPower(turnSpeed);
            robot.leftBack.setPower(turnSpeed);
            robot.rightFront.setPower(-turnSpeed);
            robot.rightBack.setPower(-turnSpeed);

        } else if (readyToTurn == 1 && Math.abs(neededRot) < 3) {

            turnSpeedRaw = 0;
            getTurnSpeed();

            robot.leftFront.setPower(turnSpeed);
            robot.leftBack.setPower(turnSpeed);
            robot.rightFront.setPower(turnSpeed);
            robot.rightBack.setPower(turnSpeed);

            engine.blackboardSet("readyToTurn", 0);
        }

    }
}
