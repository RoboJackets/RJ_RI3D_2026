package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.base.SettableSpark;

import static frc.robot.Constants.*;
import static frc.robot.Utilities.*;

public class ElevatorSubsystem extends SettableSpark {
    public enum ElevatorStates {
        UP(1),
        STOP(0),
        DOWN(-1);

        public int modifier;

        private ElevatorStates(int i) {
            modifier = i;
        }
    }

    private static double ELEVATOR_SPEED = 0.5;

    public ElevatorSubsystem() {
        super("Elevator", ELEVATOR1_CAN_ID, false, () -> ELEVATOR_SPEED);
    }

    public void moveElevator(ElevatorStates state) {
        setPower(ELEVATOR_SPEED * state.modifier);
    }

    public Command getMoveElevatorCommand(ElevatorStates state) {
        return getSetPowerCommand(ELEVATOR_SPEED * state.modifier);
    }
    @Override
    public void periodic() {
        ELEVATOR_SPEED = getNumber("edit ELEVATOR_SPEED", ELEVATOR_SPEED);
        SmartDashboard.putNumber("view elevator currentSetPower", getPower());
    }
}