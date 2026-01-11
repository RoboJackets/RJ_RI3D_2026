package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

public class ArmSubsystem extends SubsystemBase {
    
    private SparkMax arm;
    private RelativeEncoder relativeEncoder;
    private SparkClosedLoopController pidController;
    private SparkMaxConfig config = new SparkMaxConfig();

    public ArmSubsystem() {
        arm = new SparkMax(0, MotorType.kBrushless);

        config.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    
        relativeEncoder = arm.getEncoder();
        
        pidController = arm.getClosedLoopController();
        
        config.closedLoop.p(.03).i(0).d(0);
        
        pidController.setReference(0, ControlType.kPosition);

        arm.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setPower(double power) {
        arm.set(power);
    }

    public void setPosition(double position) {
        pidController.setReference(position, ControlType.kPosition);
    }

    public Command getSetPowerCommand(double power) {
        return this.startEnd(() -> {
            arm.set(power);
        }, () -> {
            setPower(0);
        });
    }

    public Command getSetPowerCommand(DoubleSupplier powSupplier) {
        return this.runEnd(() -> {
            arm.set(powSupplier.getAsDouble());
        }, () -> {
            setPower(0);
        });
    }

    public Command getSetPositionCommand(double position) {
        return this.runOnce(() -> {
            this.setPosition(
                position
            );
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Position", relativeEncoder.getPosition());
    }


}