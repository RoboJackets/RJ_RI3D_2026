package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import badgerlog.annotations.Entry;
import badgerlog.annotations.EntryType;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class IntakeAndTransferSubsystem extends SubsystemBase {
    @Entry(EntryType.SUBSCRIBER)
    public static double SPEED;

    private final SparkMax intake, indexer;

    public IntakeAndTransferSubsystem() {
        intake = new SparkMax(INTAKE_CAN_ID, MotorType.kBrushless);
        indexer = new SparkMax(INDEXER_CAN_ID, MotorType.kBrushless);

        final SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake).smartCurrentLimit(40);

        config.inverted(false); // intake inversion
        intake.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config.inverted(false); // indexer inversion
        indexer.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setPower(double power) {
        intake.set(power);
    }

    public Command getOnCommand() {
        return getSetPowerCommand(SPEED);
    }

    public Command getSetPowerCommand(double power) {
        return getSetPowerCommand(() -> power);
    }

    public Command getSetPowerCommand(DoubleSupplier powSupplier) {
        return this.runEnd(() -> {
            intake.set(powSupplier.getAsDouble());
        }, () -> {
            setPower(0);
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Power", intake.get());
    }
}
