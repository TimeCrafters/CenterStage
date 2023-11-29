package org.timecrafters.CenterStage.Autonomous.States;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.timecrafters.CenterStage.Common.SodiPizzaMinibotObject;

import dev.cyberarm.engine.V2.CyberarmState;

public class SodiPizzaAutoTurnState extends CyberarmState {
    final private SodiPizzaMinibotObject robot = new SodiPizzaMinibotObject();
    final private String groupName, actionName;
    private long lastMoveTime;
    private double turnSpeedRaw, turnSpeed;
    private int startPos;
    private double targetRot;
    private double currentRot;
    private double neededRot = targetRot - currentRot;

    private double rightTurnCW = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + 90;
    private double rightTurnCCW = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - 90;
    private double backTurnCW = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + 180;
    private double backTurnCCW = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - 180;

  /** Rot = rotation **/

    public SodiPizzaAutoTurnState() {
        groupName = " ";
        actionName = " ";
        robot.setup();
    }

    public SodiPizzaAutoTurnState(int readyToTurnParm) {
        groupName = " ";
        actionName = " ";
        robot.setup();
        robot.readyToTurn = readyToTurnParm;
    }

    private double getTurnSpeed() {
        if (Math.abs(neededRot) > 5) {
            turnSpeed = turnSpeedRaw * neededRot / 10;
        }
            return turnSpeed;
    }

    @Override
    public void start() {

        startPos = robot.leftFront.getCurrentPosition();
        turnSpeedRaw = 0;

        robot.leftFront.setPower(turnSpeed);
        robot.leftBack.setPower(turnSpeed);
        robot.rightFront.setPower(turnSpeed);
        robot.rightBack.setPower(turnSpeed);

        robot.imu.resetYaw();
    }


    @Override
    public void exec() {

        currentRot = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        if (robot.readyToTurn == 1 && robot.leftFront.getCurrentPosition() == startPos && Math.abs(neededRot) > 10) {

            targetRot = currentRot + 90;

            turnSpeedRaw = 0.3;

            robot.leftFront.setPower(turnSpeed);
            robot.leftBack.setPower(turnSpeed);
            robot.rightFront.setPower(-turnSpeed);
            robot.rightBack.setPower(-turnSpeed);

        } else if (robot.readyToTurn == 1 && Math.abs(neededRot) < 5) {
            turnSpeedRaw = 0;
            robot.leftFront.setPower(turnSpeed);
            robot.leftBack.setPower(turnSpeed);
            robot.rightFront.setPower(turnSpeed);
            robot.rightBack.setPower(turnSpeed);

            robot.readyToTurn = 0;
        }

    }
}
