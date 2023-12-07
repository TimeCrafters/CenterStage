package org.timecrafters.CenterStage.Autonomous.SodiStates;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.timecrafters.CenterStage.Common.SodiPizzaMinibotObject;

import dev.cyberarm.engine.V2.CyberarmState;

public class SodiPizzaAutoFirstDriveState extends CyberarmState{
    final private SodiPizzaMinibotObject robot;
    final private String groupName, actionName;
    private long lastMoveTime;
    private double drivePower, drivePowerRaw;
    public int readyToTurn, neededTicks, currentTicks, targetTicks = 1000;

    public SodiPizzaAutoFirstDriveState() {
        groupName = " ";
        actionName = " ";
        robot = new SodiPizzaMinibotObject();
        robot.setup();
    }

    private double getDrivePower() {
        if (Math.abs(neededTicks) > 250) {
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

        robot.imu.resetYaw();

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
        if (robot.leftFront.getCurrentPosition() <= 10 && robot.leftFront.getCurrentPosition() >= -10/* && robot.distSensor.getDistance(DistanceUnit.MM) >= 100*/) {
            if (readyToTurn == 0) {
                targetTicks = 1000;

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
            } else if (readyToTurn == 2) {

                targetTicks = 500;

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

        }

        //Stop and finish set after reached targetTicks within tolerance of 2
        else if (robot.leftFront.getCurrentPosition() >= targetTicks - 10 && robot.leftFront.getCurrentPosition() <= targetTicks + 10) {

            drivePowerRaw = 0;
            getDrivePower();

            robot.leftFront.setPower(drivePower);
            robot.leftBack.setPower(drivePower);
            robot.rightFront.setPower(drivePower);
            robot.rightBack.setPower(drivePower);

            engine.blackboardSet("readyToTurn", readyToTurn + 1);

            setHasFinished(true);
        }
    }
}
