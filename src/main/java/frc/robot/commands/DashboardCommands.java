package frc.robot.commands;

import badgerlog.annotations.Entry;
import badgerlog.annotations.EntryType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.tools.PIDElevatorCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeAndTransferSubsystem;
import frc.robot.subsystems.RevShooterFlywheelSubsystem;

public class DashboardCommands {
    @Entry(EntryType.SUBSCRIBER)
    // @Table("DashboardCommands") // shouldn't be needed?
    private static double climberPower, climberPosition, topShooterFlywheelPower,
            bottomShooterFlywheelPower, indexerPower, intakePower;

    public static Command climberPowerCommand(ElevatorSubsystem elevator) {
        return elevator.getSetPowerCommand(() -> climberPower);
    }

    public static Command climberPositionCommand(ElevatorSubsystem elevator) {
        return new PIDElevatorCommand(elevator, () -> climberPosition);
    }

    public static Command shooterPowerCommand(RevShooterFlywheelSubsystem shooter) {
        return shooter.getSetPowerCommand(() -> topShooterFlywheelPower, () -> bottomShooterFlywheelPower);
    }

    public static Command indexerPowerCommand(Indexer indexer) {
        return indexer.getSetPowerCommand(() -> indexerPower);
    }

    public static Command intakePowerCommand(IntakeAndTransferSubsystem intake) {
        return intake.getSetPowerCommand(() -> intakePower);
    }
  }