package org.timecrafters.CenterStage.Autonomous.States;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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

    @Override
    public void telemetry() {
        engine.telemetry.addData("Target Pos", robot.leftFront.getTargetPosition());
        engine.telemetry.addData("Power", robot.leftFront.getPower());
    }

    @Override
    public void start() {

        startPos = robot.leftFront.getCurrentPosition();
        turnSpeedRaw = 0;

        robot.leftFront.setPower(turnSpeed);
        robot.leftBack.setPower(turnSpeed);
        robot.rightFront.setPower(turnSpeed);
        robot.rightBack.setPower(turnSpeed);

        neededRot = (targetRot - robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
//        robot.imu.resetYaw();
    }


    @Override
    public void exec() {

        readyToTurn = engine.blackboardGet("readyToTurn");

        currentRot = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        if (readyToTurn == 1 && robot.leftFront.getCurrentPosition() == startPos && Math.abs(neededRot) > 10) {

            targetRot = -90;

            turnSpeedRaw = 0.3;

            robot.leftFront.setPower(turnSpeed);
            robot.leftBack.setPower(turnSpeed);
            robot.rightFront.setPower(-turnSpeed);
            robot.rightBack.setPower(-turnSpeed);

        } else if (readyToTurn == 1 && Math.abs(neededRot) < 5) {
            turnSpeedRaw = 0;
            robot.leftFront.setPower(turnSpeed);
            robot.leftBack.setPower(turnSpeed);
            robot.rightFront.setPower(turnSpeed);
            robot.rightBack.setPower(turnSpeed);

            engine.blackboardSet("readyToTurn", 0);
        }

    }
}
