// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.BasicElevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeAndTransferSubsystem;
import frc.robot.subsystems.RevShooterFlywheelSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                 "swerve"));

  // /**
  //  * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
  //  */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getRawAxis(1) * -.25,
                                                                () -> driverXbox.getRawAxis(0) * -.25)
                                                            .withControllerRotationAxis(()->driverXbox.getRawAxis(2))
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(false);

  private final BasicElevator climber;
  private final IntakeAndTransferSubsystem intake;
  private final RevShooterFlywheelSubsystem shooter;
  private final Indexer indexer;

  

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */

  // ArmSubsystem armSubsystem = new ArmSubsystem();

  // IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  // ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    climber = new BasicElevator();
    intake = new IntakeAndTransferSubsystem();
    shooter = new RevShooterFlywheelSubsystem();
    indexer = new Indexer();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
      Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

      driverXbox.leftStick().onTrue(Commands.runOnce(drivebase::zeroGyro));

      driverXbox.a().toggleOnTrue(intake.getOnCommand());

      driverXbox.b().toggleOnTrue(shooter.getOnCommand());
      driverXbox.rightTrigger().whileTrue(indexer.getOnCommand());

      driverXbox.povUp().whileTrue(climber.getOnCommand(false));
      driverXbox.povDown().whileTrue(climber.getOnCommand(true));
  }
    

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
     return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
