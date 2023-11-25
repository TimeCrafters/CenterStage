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
    private double currentRot = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    private double neededRot = targetRot - currentRot;
  /** Rot = rotation **/

    public SodiPizzaAutoTurnState() {
        groupName = " ";
        actionName = " ";
        robot.setup();
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

        if (robot.readyToTurn == 1 && robot.leftFront.getCurrentPosition() == startPos && Math.abs(neededRot) > 10) {

            targetRot = currentRot + 90;

            turnSpeedRaw = 0.3;

            robot.leftFront.setPower(turnSpeed);
            robot.leftBack.setPower(turnSpeed);
            robot.rightFront.setPower(-turnSpeed);
            robot.rightBack.setPower(-turnSpeed);

        } else if (robot.readyToTurn == 1 && Math.abs(neededRot) < 10) {
            turnSpeedRaw = 0;
            robot.leftFront.setPower(turnSpeed);
            robot.leftBack.setPower(turnSpeed);
            robot.rightFront.setPower(turnSpeed);
            robot.rightBack.setPower(turnSpeed);
        }

    }
}
