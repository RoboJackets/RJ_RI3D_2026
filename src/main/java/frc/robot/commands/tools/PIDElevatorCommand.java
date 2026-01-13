package frc.robot.commands.tools;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import static frc.robot.Utilities.*;

public class PIDElevatorCommand extends Command {
    private double kP = 0D, kI = 0D, kD = 0D, MAX_POWER = 0.5D;

    private PIDController controller;
    private ElevatorSubsystem elevator;

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
        final double target = this.target.getAsDouble();
        elevator.setPower(
            MathUtil.clamp(
                controller.calculate(current.getAsDouble(), target),
                -MAX_POWER,
                MAX_POWER
            )
        );

        kP = getNumber("edit PIDelevator kP", kP);
        kI = getNumber("edit PIDelevator kI", kI);
        kD = getNumber("edit PIDelevator kD", kD);

        SmartDashboard.putNumber("view target", target);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setPower(0D);
    }
}
