package frc.robot.commands.tools;

import java.util.function.DoubleSupplier;

import badgerlog.annotations.Entry;
import badgerlog.annotations.EntryType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class PIDElevatorCommand extends Command {

    @Entry(EntryType.SUBSCRIBER)
    private double kP = 0D, kI = 0D, kD = 0D, MAX_POWER = 0.5D;

    private PIDController controller;
    private ElevatorSubsystem elevator;

    @Entry(EntryType.PUBLISHER)
    private DoubleSupplier target;

    private DoubleSupplier current;

    public PIDElevatorCommand(ElevatorSubsystem elevator, DoubleSupplier target) {
        this.elevator = elevator;
        this.target = target;
        this.current = elevator.getEncoder()::getPosition;
        addRequirements(elevator);
    }

    public PIDElevatorCommand(ElevatorSubsystem elevator, double target) {
        this(elevator, () -> target);
    }
    
    @Override
    public void initialize() {
        controller = new PIDController(kP, kI, kD);
    }

    @Override
    public void execute() {
        elevator.setPower(
            MathUtil.clamp(
                controller.calculate(current.getAsDouble(), target.getAsDouble()),
                -MAX_POWER,
                MAX_POWER
            )
        );
    }
}
