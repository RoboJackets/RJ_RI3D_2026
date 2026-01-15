package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.base.SettableSpark;

import static frc.robot.Constants.*;
import static frc.robot.Utilities.*;

public class IntakeAndTransferSubsystem extends SubsystemBase {
    private final SparkMax intake, transfer1;

    private static double intakeSpeed = 0.5, transferSpeed = -0.35;
    
    @Override
    public void periodic() {
        intakeSpeed = getNumber("edit intakeSpeed", intakeSpeed);
        transferSpeed = getNumber("edit transferSpeed", transferSpeed);
    }

    public IntakeAndTransferSubsystem() {
        intake = new SparkMax(INTAKE_CAN_ID, MotorType.kBrushed);
        transfer1 = new SparkMax(TRANSFER1_CAN_ID, MotorType.kBrushless);
       // intake = new SettableSpark("intake", INTAKE_CAN_ID, false, () -> intakeSpeed, MotorType.kBrushed);
        //transfer1 = new SettableSpark("transfer1", TRANSFER1_CAN_ID, false, () -> transferSpeed);
    }

    public void setPower(double intakePower, double transferPower) {
        intake.set(intakePower);
        transfer1.set(transferPower);
    }

    // public Command getOnCommand() {
    //     return intake.getOnCommand()
    //         .alongWith(transfer1.getOnCommand());
    // }

    // public Command getSetPowerCommand(double power) {
    //     return intake.getSetPowerCommand(() -> power)
    //         .alongWith(transfer1.getSetPowerCommand(() -> power));
    // }

    // public Command getSetPowerCommand(DoubleSupplier intakePowSupplier, DoubleSupplier transferPowerSupplier) {
    //     return intake.set(intakePowSupplier)
    //         .alongWith(transfer1.set(transferPowerSupplier));
    // }
    public Command getPowerCommand(double intakeSetSpeed, double transferSetSpeed) {
        return this.runEnd(()->{intake.set(intakeSetSpeed);transfer1.set(transferSetSpeed);},()->{intake.set(0);transfer1.set(0);});
    }

    public Command getPowerCommand() {
        return getPowerCommand(intakeSpeed, transferSpeed);
    }

    public double getTransferPower() {
        return transfer1.get();
    }
}
