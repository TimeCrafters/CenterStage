package org.timecrafters.CenterStage.Autonomous.States;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.timecrafters.CenterStage.Common.SodiPizzaMinibotObject;

import dev.cyberarm.engine.V2.CyberarmState;

public class SodiPizzaAutoFirstDriveState extends CyberarmState{
    final private SodiPizzaMinibotObject robot;
    final private String groupName, actionName;
    private long lastMoveTime;
    private double drivePower, drivePowerRaw;
    public int readyToTurn, neededTicks, currentTicks, targetTicks = 1500;

    public SodiPizzaAutoFirstDriveState() {
        groupName = " ";
        actionName = " ";
        robot = new SodiPizzaMinibotObject();
        robot.setup();
    }

    private double getDrivePower() {
        if (Math.abs(neededTicks) > 1) {
            drivePower = (drivePowerRaw * neededTicks) / 10;
        }
        return drivePower;
    }

    public void CalculateNeededTicks() {
        if (targetTicks >= 0 && currentTicks >= 0) {
            neededTicks = Math.abs(targetTicks - currentTicks);
        } else if (targetTicks < 0 && currentTicks < 0) {
            neededTicks = Math.abs(targetTicks - currentTicks);
        } else if (targetTicks > 0 && currentTicks < 0) {
            neededTicks = (targetTicks + Math.abs(currentTicks));
        } else if (targetTicks < 0 && currentTicks > 0) {
            neededTicks = (currentTicks + Math.abs(targetTicks));
        }
    }
    
    @SuppressLint("SuspiciousIndentation")

    @Override
    public void start() {

        lastMoveTime = System.currentTimeMillis();
        readyToTurn = engine.blackboardGetInt("readyToTurn");

    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("Target Ticks?", targetTicks);
        engine.telemetry.addData("Current Ticks?", currentTicks);
        engine.telemetry.addData("Ticks Needed?", neededTicks);
        engine.telemetry.addLine();
        engine.telemetry.addData("Internal Ready To Turn Value", readyToTurn);
        engine.telemetry.addData("Distance Sensor Reading", robot.distSensor.getDistance(DistanceUnit.MM));

    }

    @Override
    public void exec() {

        readyToTurn = engine.blackboardGet("readyToTurn");

        currentTicks = robot.leftFront.getCurrentPosition();

        CalculateNeededTicks();

        // Move forward from 0 to targetTicks
        if (robot.leftFront.getCurrentPosition() <= 10 && robot.leftFront.getCurrentPosition() >= -10 && readyToTurn == 0) {

            robot.leftFront.setTargetPosition(targetTicks);
            robot.leftBack.setTargetPosition(targetTicks);
            robot.rightFront.setTargetPosition(targetTicks);
            robot.rightBack.setTargetPosition(targetTicks);

            drivePowerRaw = 0.5;
            getDrivePower();

            robot.leftFront.setPower(drivePower);
            robot.leftBack.setPower(drivePower);
            robot.rightFront.setPower(drivePower);
            robot.rightBack.setPower(drivePower);

        }
        //Stop and finish set after return to 0
        else if (robot.leftFront.getCurrentPosition() >= targetTicks - 10 && robot.leftFront.getCurrentPosition() <= targetTicks + 10) {

            drivePowerRaw = 0;
            getDrivePower();

            robot.leftFront.setPower(drivePower);
            robot.leftBack.setPower(drivePower);
            robot.rightFront.setPower(drivePower);
            robot.rightBack.setPower(drivePower);

            engine.blackboardSet("readyToTurn", 1);

            setHasFinished(true);
        }
    }
}
