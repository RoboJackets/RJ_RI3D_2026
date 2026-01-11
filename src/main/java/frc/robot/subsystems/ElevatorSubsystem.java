package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ElevatorSubsystem extends SubsystemBase {
    public enum ElevatorStates {
        UP(1),
        STOP(0),
        DOWN(-1);

        public int modifier;

        private ElevatorStates(int i) {
            modifier = i;
        }
    }
    
    private SparkMax elevator;
    private SparkMaxConfig config = new SparkMaxConfig();
    private final double ELEVATOR_SPEED = 0.5;

    public ElevatorSubsystem() {
        elevator = new SparkMax(0, MotorType.kBrushless);

        config.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(40);

        elevator.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void moveElevator(ElevatorStates state) {
        setPower(ELEVATOR_SPEED * state.modifier);
    }

    public void setPower(double power) {
        elevator.set(power);
    }

    public Command getMoveElevatorCommand(ElevatorStates state) {
        return getSetPowerCommand(ELEVATOR_SPEED * state.modifier);
    }

    public Command getSetPowerCommand(double power) {
        return this.startEnd(() -> {
            elevator.set(power);
        }, () -> {
            setPower(0);
        });
    }

    public Command getSetPowerCommand(DoubleSupplier powSupplier) {
        return this.runEnd(() -> {
            elevator.set(powSupplier.getAsDouble());
        }, () -> {
            setPower(0);
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("elevator Power", elevator.get());
    }
}