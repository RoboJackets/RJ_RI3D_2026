package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import badgerlog.annotations.Entry;
import badgerlog.annotations.EntryType;

import static frc.robot.Utilities.*;
import static frc.robot.Constants.*;


public class CTREShooterFlywheelSubsystem extends SubsystemBase {
    public enum ControlMode {
        VelocityPID,
        DutyCycle,
        Coast
    }

    @Entry(EntryType.SUBSCRIBER)
    private static double MAX_VOLTAGE = 11.5; // can be changed

    @Entry(EntryType.SUBSCRIBER)
    private double toggledDutyCycleSpeed = 1D;

    @Entry(EntryType.PUBLISHER)
    private double targetTopRPM = 0, currentTopRPM = 0, targetBottomRPM = 0, currentBottomRPM = 0;

    private static final double SHOOTER_TO_MOTOR_RATIO = 24D / 18D; // X shooter rotations : 1 motor rotation

    public static final double MAX_RPM = Math.round(SHOOTER_TO_MOTOR_RATIO * 6000);

    private static final double TOP_DIAMETER_INCHES = 4, BOTTOM_DIAMETER_INCHES = 3;

    private static final InvertedValue TOP_INVERTED = InvertedValue.Clockwise_Positive,
            BOTTOM_INVERTED = InvertedValue.CounterClockwise_Positive;
    

    private TalonFX topKraken, bottomKraken;

    private VelocityVoltage velocityControllerTop = new VelocityVoltage(0).withSlot(0),
            velocityControlBottom = new VelocityVoltage(0).withSlot(0); // start at 0 rps, config will be applied from slot 0


    private DutyCycleOut dutyCycleTop = new DutyCycleOut(0D), dutyCycleBottom = new DutyCycleOut(0D);
    private NeutralOut coastTop = new NeutralOut(), coastBottom = new NeutralOut();

    private ControlMode controlMode = ControlMode.VelocityPID;
    
    public CTREShooterFlywheelSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kS = 0.1; // picked randomly from CTRE example
        config.Slot0.kV = 0.12; // kraken x60 is 500 kv, at 12v this is the number you get for volts / rps
        config.Slot0.kP = 0.11; // picked randomly from CTRE example
        config.Slot0.kI = 0; // not needed
        config.Slot0.kD = 0; // not needed
        config.Voltage.withPeakForwardVoltage(Volts.of(MAX_VOLTAGE)).withPeakReverseVoltage(Volts.of(-MAX_VOLTAGE));

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast; // maybe not needed

        config.CurrentLimits.SupplyCurrentLowerLimit = 50; // 50A
        config.CurrentLimits.SupplyCurrentLowerTime = 2.5; // 2.5s before current dorps
        
        applyTalonFXConfig(topKraken, config.withMotorOutput(new MotorOutputConfigs().withInverted(TOP_INVERTED)));
        applyTalonFXConfig(bottomKraken, config.withMotorOutput(new MotorOutputConfigs().withInverted(BOTTOM_INVERTED)));

        topKraken = new TalonFX(TOP_SHOOTER_NEO_CAN_ID);
        bottomKraken = new TalonFX(BOTTOM_SHOOTER_NEO_CAN_ID);
    }

    public ControlMode getControlMode() {
        return controlMode;
    }

    public void setControlMode(ControlMode mode) {
        controlMode = mode;
    }

    @Override
    public void periodic() {
        currentTopRPM = topKraken.getRotorVelocity().getValueAsDouble() * SHOOTER_TO_MOTOR_RATIO;
        currentBottomRPM = bottomKraken.getRotorVelocity().getValueAsDouble() * SHOOTER_TO_MOTOR_RATIO;
        
        switch (controlMode) {
            case VelocityPID -> {
                topKraken.setControl(velocityControllerTop.withVelocity(targetTopRPM / SHOOTER_TO_MOTOR_RATIO));
                bottomKraken.setControl(velocityControlBottom.withVelocity(targetBottomRPM / SHOOTER_TO_MOTOR_RATIO));
            }
            case DutyCycle -> {
                topKraken.setControl(dutyCycleTop.withOutput(targetTopRPM / MAX_RPM));
                bottomKraken.setControl(dutyCycleBottom.withOutput(targetBottomRPM / MAX_RPM));
            }
            case Coast -> {
                topKraken.setControl(coastTop);
                bottomKraken.setControl(coastBottom);
            }
        }
        
    }

    public double getCurrentTopRPM() {
        return currentTopRPM;
    }
    
    public double getCurrentBottomRPM() {
        return currentBottomRPM;
    }

    public void setTopTargetVelocity(double rpm) {
        targetTopRPM = rpm;
    }

    public void setBottomTargetVelocity(double rpm) {
        targetBottomRPM = rpm;
    }
    
    public void setVelocity(double rpm) {
        targetTopRPM = rpm;
        targetBottomRPM = rpm;
    }

    public Command getOnCommand() {
        return getSetPowerCommand(toggledDutyCycleSpeed);
    }

    public Command getSetPowerCommand(double power) {
        return getSetPowerCommand(() -> power);
    }
    
    public Command getSetPowerCommand(final DoubleSupplier powSupplier) {
        return getSetPowerCommand(powSupplier, 1, 1);
    }

    /**
     * simple power command meant for usage by controller, smartdashboard, etc.
     * @param powSupplier duty cycle power double supplier --- should be in[-1, 1]
     * @param topMult should be [-1, 1]. could be used to make one wheel move slower than other
     * @param bottomMult same restrictions as top mult
     * @return
     */
    public Command getSetPowerCommand(final DoubleSupplier powSupplier, final double topMult, final double bottomMult) {
        final double correctedTopMult = MathUtil.clamp(topMult, -1, 1) * MAX_RPM;
        final double correctedBottomMult = MathUtil.clamp(bottomMult, -1, 1) * MAX_RPM;

        return this.runOnce(() -> controlMode = ControlMode.DutyCycle)
            .andThen(this.runEnd(() -> {
                    final double target = powSupplier.getAsDouble();
                    setTopTargetVelocity(target * correctedTopMult);
                    setBottomTargetVelocity(target * correctedBottomMult);
                }, () -> {
                    setVelocity(0);
                }
            )
        );
    }

    public Command getSetVelocityCommand(double velocity) {
        return this.runOnce(() -> {
            this.setVelocity(
                velocity
            );
        });
    }
}