package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.tools.PIDElevatorCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeAndTransferSubsystem;
import frc.robot.subsystems.RevShooterFlywheelSubsystem;
import static frc.robot.Utilities.*;

public class DashboardCommands extends SubsystemBase {
    private double climberPower, climberPosition, topShooterFlywheelPower,
            bottomShooterFlywheelPower, indexerPower, intakePower, transferPower;

    @Override
    public void periodic() {
        climberPower = getNumber("edit climberPower", climberPower);
        climberPosition = getNumber("edit climberPosition", climberPosition);
        topShooterFlywheelPower = getNumber("edit topShooterFlywheelPower", topShooterFlywheelPower);
        bottomShooterFlywheelPower = getNumber("edit bottomShooterFlywheelPower", bottomShooterFlywheelPower);
        indexerPower = getNumber("edit indexerPower", indexerPower);
        intakePower = getNumber("edit intakePower", intakePower);
        transferPower = getNumber("edit transferPower", transferPower);
    }

    public Command climberPowerCommand(ElevatorSubsystem elevator) {
        return elevator.getSetPowerCommand(() -> climberPower);
    }

    public Command climberPositionCommand(ElevatorSubsystem elevator) {
        return new PIDElevatorCommand(elevator, () -> climberPosition);
    }

    public Command shooterPowerCommand(RevShooterFlywheelSubsystem shooter) {
        return shooter.getSetPowerCommand(() -> topShooterFlywheelPower, () -> bottomShooterFlywheelPower);
    }

    public Command indexerPowerCommand(Indexer indexer) {
        return indexer.getSetPowerCommand(() -> indexerPower);
    }

    // public Command intakePowerCommand(IntakeAndTransferSubsystem intake) {
    //     return intake.getSetPowerCommand(() -> intakePower, () -> transferPower);
    //}
  }