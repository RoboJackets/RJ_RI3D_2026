package frc.robot.subsystems.base;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import badgerlog.annotations.Entry;
import badgerlog.annotations.EntryType;
import badgerlog.annotations.Table;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SettableSpark extends SubsystemBase {
    @SuppressWarnings("unused") // used by @Table
    private final String name;

    @Entry(EntryType.SUBSCRIBER)
    @Table("{name}")
    public double defaultSpeed = 1;

    @Entry(EntryType.PUBLISHER)
    @Table("{name}")
    private double currentPower;

    private final SparkMax motor;

    public SettableSpark(final String name, int can_id, boolean invert, double defaultSpeed) {
        super(name);
        this.name = name;

        motor = new SparkMax(can_id, MotorType.kBrushless);
        final SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake).smartCurrentLimit(40);

        config.inverted(invert); // intake inversion
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public SettableSpark(final String name, int can_id, boolean invert) {
        this(name, can_id, invert, 1D);
    }

    public void setPower(double power) {
        motor.set(power);
        this.currentPower = power;
    }

    public Command getOnCommand() {
        return getOnCommand(false);
    }

    public Command getOnCommand(boolean invert) {
        return getSetPowerCommand(defaultSpeed * (invert ? -1 : 1));
    }

    public Command getSetPowerCommand(double power) {
        return getSetPowerCommand(() -> power);
    }

    public Command getSetPowerCommand(DoubleSupplier powSupplier) {
        return this.runEnd(() -> {
            setPower(powSupplier.getAsDouble());
        }, () -> {
            setPower(0);
        });
    }

    public RelativeEncoder getEncoder() {
        return motor.getEncoder();
    }

    @Override
    public String getName() {
        return name;
    }
}
