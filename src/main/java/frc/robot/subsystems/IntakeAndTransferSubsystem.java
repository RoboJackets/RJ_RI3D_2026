package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.base.SettableSpark;

import static frc.robot.Constants.*;

public class IntakeAndTransferSubsystem extends SubsystemBase {
    private final SettableSpark intake, transfer;

    public IntakeAndTransferSubsystem() {
        intake = new SettableSpark("intake", INTAKE_CAN_ID, false);
        transfer = new SettableSpark("transfer", TRANSFER_CAN_ID, false);
    }

    public void setPower(double power) {
        intake.setPower(power);
        transfer.setPower(power);
    }

    public Command getOnCommand() {
        return intake.getOnCommand().alongWith(transfer.getOnCommand());
    }

    public Command getSetPowerCommand(double power) {
        return intake.getSetPowerCommand(() -> power).alongWith(transfer.getSetPowerCommand(() -> power));
    }

    public Command getSetPowerCommand(DoubleSupplier powSupplier) {
        return intake.getSetPowerCommand(powSupplier).alongWith(transfer.getSetPowerCommand(powSupplier));
    }
}
