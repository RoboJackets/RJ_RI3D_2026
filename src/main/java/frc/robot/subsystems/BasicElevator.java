package frc.robot.subsystems;

import frc.robot.subsystems.base.SettableSpark;

import static frc.robot.Constants.*;

/**
 * To be used if the more advanced elevator system does not work
 */
public class BasicElevator extends SettableSpark {
    public BasicElevator() {
        super("Basic Elevator", ELEVATOR1_CAN_ID, false);
    }
}
