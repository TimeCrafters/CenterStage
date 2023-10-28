package org.timecrafters.CenterStage.Autonomous.States;

/**BEGAN WITH 43 LINES**/

import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.timecrafters.CenterStage.Common.ProtoBotSodi;
import org.timecrafters.CenterStage.Common.PrototypeRobot;

import dev.cyberarm.engine.V2.CyberarmState;

public class ArmStateSodi extends CyberarmState {

    private final boolean stateDisabled;
    ProtoBotSodi robot;
    private int testSequence;
    private int targetPos;
    private int currentPos;
    private int totalDist;


    public ArmStateSodi(ProtoBotSodi robot, String groupName, String actionName) {
        this.robot = robot;
        this.stateDisabled = !robot.configuration.action(groupName, actionName).enabled;
    }

    private int getTotalDist() {
        int totalDist = targetPos - currentPos;
        return totalDist;
    }




    @Override
    public void start() {
        //add variables that need to be reinitialized
        //NOT REINITILLIZED >:(
    }

    @Override
    public void init() {


    }

    @Override
    public void exec() {

//        if (robot.liftMotor.motor.getCurrentPosition() >= 2800 &&
//        Math.abs(robot.liftMotor.motor.getPower()) != robot.liftMotor.motor.getPower()) {
//            robot.liftMotor.motor.setPower(0);
//        }
//
//        if (robot.liftMotor.motor.getCurrentPosition() < 0 &&
//        Math.abs(robot.liftMotor.motor.getPower()) != robot.liftMotor.motor.getPower()) {
//            robot.liftMotor.motor.setPower(0);
//        }
//
//        if (robot.liftMotor.motor.getCurrentPosition() >= 0 &&
//        robot.liftMotor.motor.getCurrentPosition() <= 50) {
//
//        }
//
//        if (robot.liftMotor.motor.getCurrentPosition() >= 50 &&
//        robot.liftMotor.motor.getCurrentPosition() <= 250) {
//
//        }
//
//
    }
}
