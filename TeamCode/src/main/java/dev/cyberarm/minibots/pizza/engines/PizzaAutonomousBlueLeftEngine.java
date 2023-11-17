package dev.cyberarm.minibots.pizza.engines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.cyberarm.engine.V2.CyberarmEngine;
import dev.cyberarm.minibots.pizza.PizzaMinibot;
import dev.cyberarm.minibots.pizza.states.Rotate;
import dev.cyberarm.minibots.pizza.states.ServoMove;
import dev.cyberarm.minibots.pizza.states.StrafeMove;
import dev.cyberarm.minibots.pizza.states.TankMove;

@Autonomous(name = "Cyberarm Pizza Autonomous BLUE LEFT", group = "MINIBOT", preselectTeleOp = "Cyberarm Pizza Teleop")
public class PizzaAutonomousBlueLeftEngine extends CyberarmEngine {
    private PizzaMinibot robot;
    @Override
    public void setup() {
        this.robot = new PizzaMinibot(this);
        robot.imu.resetYaw();

        /// Close the gripper
        addState(new ServoMove(
                robot,
                "gripper",
                PizzaMinibot.GRIPPER_CLOSED,
                100
        ));

        /// Move Purple Pixel into position
        addState(new TankMove(
                robot,
                -900,
                -900,
                -0.15,
                -0.17,
                10,
                5000
        ));
        addState(new Rotate(
                robot,
                0,
                0.15,
                1,
                5000
        ));

        /// Retreat from pixel
        addState(new TankMove(
                robot,
                150,
                150,
                0.15,
                0.17,
                10,
                5000
        ));
        /// Rotate "towards" backstage board
        addState(new Rotate(
                robot,
                90,
                0.15,
                1,
                5000
        ));

        /// Move clear of the pixel
        addState(new TankMove(
                robot,
                500,
                500,
                0.15,
                0.17,
                10,
                5000
        ));
        /// Strafe so arm is in range of board
        addState(new StrafeMove(
                robot,
                1000,
                0.5,
                10,
                5000
        ));
        /// Correct robots rotation "towards" backstage board
        addState(new Rotate(
                robot,
                90,
                0.15,
                1,
                5000
        ));

        /// Move up to the backboard and pre-position arm
        addState(new TankMove(
                robot,
                500,
                500,
                0.15,
                0.17,
                10,
                5000
        ));
        addParallelStateToLastState(new ServoMove(
                robot,
                "arm",
                PizzaMinibot.ARM_DELIVER,
                1000
        ));

        /// Correct rotation; Rotate "towards" backstage board
        addState(new Rotate(
                robot,
                90,
                0.15,
                1,
                5000
        ));
        /// Move up against the backboard
        addState(new TankMove(
                robot,
                200,
                200,
                0.15,
                0.17,
                10,
                1250
        ));
        /// Correct rotation again
        addState(new Rotate(
                robot,
                90,
                0.15,
                1,
                5000
        ));
        /// Deposit pixel on backboard
        addState(new ServoMove(
                robot,
                "gripper",
                PizzaMinibot.GRIPPER_OPEN,
                1000
        ));
        /// Stow the arm
        addState(new ServoMove(
                robot,
                "arm",
                PizzaMinibot.ARM_STOW,
                250
        ));
        /// Move away from the backboard
        addState(new TankMove(
                robot,
                -250,
                -250,
                -0.15,
                -0.17,
                10,
                5000
        ));
        /// Rotate towards the corner
        addState(new Rotate(
                robot,
                178,
                0.15,
                1,
                5000
        ));
        /// Move into corner
        addState(new TankMove(
                robot,
                -1000,
                -1000,
                -0.15,
                -0.17,
                10,
                5000
        ));
    }

    @Override
    public void loop() {
        robot.standardTelemetry();

        super.loop();
    }
}
