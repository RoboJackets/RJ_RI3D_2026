package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class IntakeSubsystem extends SubsystemBase {
    
    private SparkMax intake;
    private SparkMaxConfig config = new SparkMaxConfig();

    public IntakeSubsystem() {
        intake = new SparkMax(0, MotorType.kBrushless);

        config.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    }

    public void setPower(double power) {
        intake.set(power);
    }

    public Command getSetPowerCommand(double power) {
        return this.startEnd(() -> {
            intake.set(power);
        }, () -> {
            setPower(0);
        });
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