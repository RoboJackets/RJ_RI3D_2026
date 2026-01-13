package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.base.SettableSpark;

import static frc.robot.Constants.*;
import static frc.robot.Utilities.*;

public class IntakeAndTransferSubsystem extends SubsystemBase {
    private final SettableSpark intake, transfer1;

    private static double intakeSpeed = 1, transferSpeed = 1;
    
    @Override
    public void periodic() {
        intakeSpeed = getNumber("edit intakeSpeed", intakeSpeed);
        transferSpeed = getNumber("edit transferSpeed", transferSpeed);
    }

    public IntakeAndTransferSubsystem() {
        intake = new SettableSpark("intake", INTAKE_CAN_ID, true, () -> intakeSpeed, MotorType.kBrushed);
        transfer1 = new SettableSpark("transfer1", TRANSFER1_CAN_ID, false, () -> transferSpeed);
    }

    public void setPower(double intakePower, double transferPower) {
        intake.setPower(intakePower);
        transfer1.setPower(transferPower);
    }

    public Command getOnCommand() {
        return intake.getOnCommand()
            .alongWith(transfer1.getOnCommand());
    }

    public Command getSetPowerCommand(double power) {
        return intake.getSetPowerCommand(() -> power)
            .alongWith(transfer1.getSetPowerCommand(() -> power));
    }

    public Command getSetPowerCommand(DoubleSupplier intakePowSupplier, DoubleSupplier transferPowerSupplier) {
        return intake.getSetPowerCommand(intakePowSupplier)
            .alongWith(transfer1.getSetPowerCommand(transferPowerSupplier));
    }
}
