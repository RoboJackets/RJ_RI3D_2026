// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DashboardCommands;
import frc.robot.commands.auto.AlignAndFlywheel;
import frc.robot.commands.auto.AutoAlign;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeAndTransferSubsystem;
import frc.robot.subsystems.RevShooterFlywheelSubsystem;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

import static frc.robot.Constants.LIMELIGHT_2PLUS_CENTER_NAME;
import static frc.robot.Utilities.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {
  public static double POW = 2, MULT_X = -.25, MULT_Y = -.25, MULT_ROT = .5, DEADBAND = .05;

  public static boolean driveRobotOriented = false, controlFlywheelWithAutoAlign = false;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  // private final SwerveSubsystem drivebase;

  // /**
  // * Converts driver input into a field-relative ChassisSpeeds that is
  // controlled by angular velocity.
  //
  // private final SwerveInputStream driveAngularVelocity;

  private final ElevatorSubsystem climber;
  private final Intake intake;
  private final Transfer transfer;
  private final RevShooterFlywheelSubsystem shooter;
  private final Indexer indexer;

  // private final Command driveFieldOrientedCommand, driveRobotOrientedCommand;

  private boolean enableClimberPowDash = false, enableClimberPosDash = false, enableShooterDash = false,
      enableIndexerDash = false, enableIntakeDash = false;

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative
   * input stream.
   */

  // ArmSubsystem armSubsystem = new ArmSubsystem();

  // IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  // ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

    // driveAngularVelocity = SwerveInputStream.of(
    //     drivebase.getSwerveDrive(),
    //     () -> Math.pow(driverXbox.getRawAxis(1), POW) * MULT_X,
    //     () -> Math.pow(driverXbox.getRawAxis(0), POW) * MULT_Y)
    //     .withControllerRotationAxis(() -> driverXbox.getRawAxis(2) * MULT_ROT)
    //     .deadband(DEADBAND)
    //     .scaleTranslation(1)
    //     .allianceRelativeControl(true);

    // driveFieldOrientedCommand = drivebase.driveFieldOriented(driveAngularVelocity);
    // driveRobotOrientedCommand = drivebase.drive(driveAngularVelocity);

    // Configure the trigger bindings
    DriverStation.silenceJoystickConnectionWarning(true);

    climber = new ElevatorSubsystem();
    intake = new Intake();
    transfer = new Transfer();
    shooter = new RevShooterFlywheelSubsystem();
    indexer = new Indexer();

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {
    // drivebase.setDefaultCommand(Commands.either(
    //     driveRobotOrientedCommand,
    //     driveFieldOrientedCommand,
    //     () -> driveRobotOriented));

    // driverXbox.leftStick().onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));

    driverXbox.a().whileTrue(intake.getOnCommand());
    driverXbox.x().whileTrue(transfer.getOnCommand());


    driverXbox.b().toggleOnTrue(shooter.getOnCommand());
    driverXbox.rightTrigger().whileTrue(indexer.getOnCommand());

    driverXbox.povUp().whileTrue(climber.getOnCommand(false));
    driverXbox.povDown().whileTrue(climber.getOnCommand(true));
    // driverXbox.rightBumper().toggleOnTrue(
    //     Commands.either(
    //         new AutoAlign(drivebase, driveAngularVelocity),
    //         new AlignAndFlywheel(drivebase, driveAngularVelocity, shooter),
    //         () -> controlFlywheelWithAutoAlign).until(driverXbox.axisGreaterThan(2, .5)));

    final DashboardCommands dashboardCommands = new DashboardCommands();
    // new Trigger(() -> enableClimberPowDash).whileTrue(dashboardCommands.climberPowerCommand(climber));
    // new Trigger(() -> enableClimberPosDash).whileTrue(dashboardCommands.climberPositionCommand(climber));
    // new Trigger(() -> enableShooterDash).whileTrue(dashboardCommands.shooterPowerCommand(shooter));
    // new Trigger(() -> enableIndexerDash).whileTrue(dashboardCommands.indexerPowerCommand(indexer));
    // new Trigger(() -> enableIntakeDash).whileTrue(dashboardCommands.intakePowerCommand(intake));

    // driverXbox.leftBumper()
    //     .whileTrue(drivebase.drive(SwerveInputStream.of(drivebase.getSwerveDrive(), () -> 1, () -> 1)));

    new Trigger(() -> DriverStation.isEnabled()).whileTrue(((new InstantCommand(() -> {
      LimelightHelpers.setLEDMode_ForceOn(LIMELIGHT_2PLUS_CENTER_NAME);
    }, new Subsystem[] {})
        .andThen(new WaitCommand(0.25))
        .andThen(new InstantCommand(() -> {
          LimelightHelpers.setLEDMode_ForceOff(LIMELIGHT_2PLUS_CENTER_NAME);
        }, new Subsystem[] {}))
        .andThen(new WaitCommand(0.25)))
        .repeatedly()).finallyDo(() -> {
          LimelightHelpers.setLEDMode_ForceOn(LIMELIGHT_2PLUS_CENTER_NAME);
        }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Commands.none();
    // return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake) {
    // drivebase.setMotorBrake(brake);
  }

  public void updateDashboard() {
    POW = getNumber("edit POW", POW);
    MULT_X = getNumber("edit MULT_X", MULT_X);
    MULT_Y = getNumber("edit MULT_Y", MULT_Y);
    MULT_ROT = getNumber("edit MULT_ROT", MULT_ROT);
    DEADBAND = getNumber("edit DEADBAND", DEADBAND);

    driveRobotOriented = getBoolean("edit driveRobotOriented", driveRobotOriented);
    controlFlywheelWithAutoAlign = getBoolean("edit controlFlywheelWithAutoAlign", controlFlywheelWithAutoAlign);

    enableClimberPowDash = getBoolean("edit enableClimberPowDash", enableClimberPowDash);
    enableClimberPosDash = getBoolean("edit enableClimberPosDash", enableClimberPosDash);
    enableShooterDash = getBoolean("edit enableShooterDash", enableShooterDash);
    enableIndexerDash = getBoolean("edit enableIndexerDash", enableIndexerDash);
    enableIntakeDash = getBoolean("edit enableIntakeDash", enableIntakeDash);

  }
}
