package dev.cyberarm.minibots.yellow.engines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.cyberarm.engine.V2.CyberarmEngine;
import dev.cyberarm.minibots.yellow.YellowMinibot;
import dev.cyberarm.minibots.yellow.states.Rotate;
import dev.cyberarm.minibots.yellow.states.TankMove;

@Autonomous(name = "Cyberarm Yellow Autonomous BLUE RIGHT", group = "MINIBOT", preselectTeleOp = "Cyberarm Yellow Teleop")
public class YellowMinibotAutonomousBlueRightEngine extends CyberarmEngine {
    @Override
    public void setup() {
        YellowMinibot robot = new YellowMinibot(this);
        robot.imu.resetYaw();

        /// Move Purple pixel into position
        addState(new TankMove(
                robot,
                1450,
                1450,
                -1.0,
                -1.0,
                10,
                5000
        ));
        /// Move away from pixel
        addState(new TankMove(
                robot,
                250,
                250,
                1.0,
                1.0,
                10,
                5000
        ));
        /// Turn around
        addState(new Rotate(
                robot,
                182,
                1.0,
                0.25,
                1,
                5000
        ));
        /// Move back towards yellow pixel
        addState(new TankMove(
                robot,
                850,
                850,
                -1.0,
                -1.0,
                10,
                5000
        ));
        /// Turn around
        addState(new Rotate(
                robot,
                270,
                1.0,
                0.25,
                1,
                5000
        ));
        /// Move towards backstage
        addState(new TankMove(
                robot,
                4000,
                4000,
                -1.0,
                -1.0,
                10,
                8000
        ));
    }
}


