package frc.robot.subsystems.base;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SettableSpark extends SubsystemBase {
    private String name;

    private DoubleSupplier defaultSpeed;

    private double currentPower;

    private final SparkMax motor;

    private boolean invert;

    public SettableSpark(final String name, int can_id, boolean invert, DoubleSupplier defaultSpeed, MotorType motorType) {
        super(name);
        this.name = name;

        motor = new SparkMax(can_id, motorType);
        final SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake).smartCurrentLimit(40);

        config.inverted(invert); // intake inversion
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        this.defaultSpeed = defaultSpeed;
        
        this.invert = invert;
    }

    public SettableSpark(final String name, int can_id, boolean invert, DoubleSupplier defaultSpeed) {
        this(name, can_id, invert, defaultSpeed, MotorType.kBrushless);
    }
    
    public SettableSpark(final String name, int can_id, boolean invert) {
        this(name, can_id, invert, () -> 1D);
    }

    public void setPower(double power) {
        motor.set(power * (this.invert ? -1 : 1));
        this.currentPower = power;
    }

    public Command getOnCommand() {
        return getOnCommand(false);
    }

    public Command getOnCommand(boolean invert) {
        return getSetPowerCommand(defaultSpeed.getAsDouble() * (invert ? -1 : 1));
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

    public double getPower() {
        return currentPower;
    }
}
