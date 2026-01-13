package frc.robot.subsystems;

import static frc.robot.Constants.BOTTOM_SHOOTER_NEO_CAN_ID;
import static frc.robot.Constants.TOP_SHOOTER_NEO_CAN_ID;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Utilities.*;


public class RevShooterFlywheelSubsystem extends SubsystemBase {
    public enum ControlMode {
        VelocityPID,
        DutyCycle,
        Coast
    }

    private static double MAX_VOLTAGE = 11.5; // can be changed

    private double toggledDutyCycleSpeed = 1D;

    private double targetTopRPM = 0, currentTopRPM = 0, targetBottomRPM = 0, currentBottomRPM = 0;

    private void updateSmartDashboard() {
        MAX_VOLTAGE = getNumber("edit SHOOTER MAX_VOLTAGE", MAX_VOLTAGE);
        toggledDutyCycleSpeed = getNumber("edit SHOOTER toggledDutyCycleSpeed", toggledDutyCycleSpeed);

        SmartDashboard.putNumber("view targetTopRPM SHOOTER", targetTopRPM);
        SmartDashboard.putNumber("view currentTopRPM SHOOTER", currentTopRPM);
        SmartDashboard.putNumber("view targetBottomRPM SHOOTER", targetBottomRPM);
        SmartDashboard.putNumber("view currentBottomRPM SHOOTER", currentBottomRPM);
    }

    private static final double SHOOTER_TO_MOTOR_RATIO = 24D / 18D; // X shooter rotations : 1 motor rotation

    public static final double MAX_RPM = Math.round(SHOOTER_TO_MOTOR_RATIO * 6000);

    @SuppressWarnings("unused")
    private static final double TOP_DIAMETER_INCHES = 4, BOTTOM_DIAMETER_INCHES = 3;

    private static final boolean TOP_INVERTED = false,
            BOTTOM_INVERTED = false;
    

    private SparkMax topSpark, bottomSpark;
    private RelativeEncoder topEncoder, bottomEncoder;
    
    private SimpleMotorFeedforward feedforwardController;
    private PIDController pidControllerTop, pidControllerBottom;
    
    private ControlMode controlMode = ControlMode.VelocityPID;
    
    public RevShooterFlywheelSubsystem() {
        topSpark = new SparkMax(TOP_SHOOTER_NEO_CAN_ID, MotorType.kBrushless);
        bottomSpark = new SparkMax(BOTTOM_SHOOTER_NEO_CAN_ID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();

        config.encoder.velocityConversionFactor(SHOOTER_TO_MOTOR_RATIO)
                .quadratureMeasurementPeriod(16)
                .quadratureAverageDepth(2);

        config.smartCurrentLimit(90)
                .idleMode(IdleMode.kCoast)
                .signals.primaryEncoderVelocityPeriodMs(15); // changing status frame 20ms -> 15ms

        config.inverted(TOP_INVERTED);
        topSpark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config.inverted(BOTTOM_INVERTED);
        bottomSpark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        topEncoder = topSpark.getEncoder();
        bottomEncoder = bottomSpark.getEncoder();
    
        feedforwardController = new SimpleMotorFeedforward(0.1, .12);
        pidControllerTop = new PIDController(.11, 0D, 0D);
        pidControllerBottom = new PIDController(.11, 0D, 0D);
    }

    public ControlMode getControlMode() {
        return controlMode;
    }

    public void setControlMode(ControlMode mode) {
        controlMode = mode;
    }

    @Override
    public void periodic() {
        currentTopRPM = topEncoder.getVelocity();
        currentBottomRPM = bottomEncoder.getVelocity();
            
        switch (controlMode) {
            case VelocityPID -> {
                topSpark.setVoltage(pidControllerTop.calculate(currentTopRPM, targetTopRPM) + feedforwardController.calculate(targetTopRPM));
                bottomSpark.setVoltage(pidControllerBottom.calculate(currentTopRPM, targetBottomRPM) + feedforwardController.calculate(targetBottomRPM));
            }
            case DutyCycle -> {
                topSpark.set(targetTopRPM / MAX_RPM);
                bottomSpark.set(targetBottomRPM / MAX_RPM);
            }
            case Coast -> {
                topSpark.set(0);
                bottomSpark.set(0);
            }
        }
        
        updateSmartDashboard();
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
        return getSetPowerCommand(powSupplier, powSupplier, topMult, bottomMult);
    }

    public Command getSetPowerCommand(final DoubleSupplier topPowerSupplier, final DoubleSupplier bottomPowerSupplier) {
        return getSetPowerCommand(topPowerSupplier, bottomPowerSupplier, 1, 1);
    }

    /**
     * simple power command meant for usage by controller, smartdashboard, etc.
     * @param topPowerSupplier duty cycle power double supplier for top shooter wheel --- should be in [-1, 1]
     * @param bottomPowerSupplier same as above, but for bottom shooter wheel
     * @param topMult should be [-1, 1]. could be used to make one wheel move slower than other
     * @param bottomMult same restrictions as top mult
     * @return
     */
    public Command getSetPowerCommand(final DoubleSupplier topPowerSupplier, final DoubleSupplier bottomPowerSupplier,
            final double topMult, final double bottomMult) {
        final double correctedTopMult = MathUtil.clamp(topMult, -1, 1) * MAX_RPM;
        final double correctedBottomMult = MathUtil.clamp(bottomMult, -1, 1) * MAX_RPM;

        return this.runOnce(() -> controlMode = ControlMode.DutyCycle)
            .andThen(this.runEnd(() -> {
                    setTopTargetVelocity(topPowerSupplier.getAsDouble() * correctedTopMult);
                    setBottomTargetVelocity(bottomPowerSupplier.getAsDouble() * correctedBottomMult);
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