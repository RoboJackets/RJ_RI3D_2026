package frc.robot.commands.auto;

import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import badgerlog.annotations.Entry;
import badgerlog.annotations.EntryType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAlign extends Command {
    // https://github.com/team1306/Robot2024/blob/main/src/main/java/frc/robot/commands/drive/ShooterDriveCommand.java
    private SwerveSubsystem swerve; // final
    private PIDController controller;
    // shooting location must have an offset

    @Entry(EntryType.SUBSCRIBER)
    public static double kP = 0.01, kI = 0, kD = 0;
    private static final double TOLERANCE_DEGREES = 5;
    private final Translation3d hubLocation;

    public double delta = Double.POSITIVE_INFINITY;
    
    double distanceInches;

    public AutoAlign(SwerveSubsystem swerve) {
        this.swerve = swerve;
        this.addRequirements(swerve);

        if (swerve.isRedAlliance()) {
            hubLocation = Constants.HubLocations.RED.translation3d;
        } else {
            hubLocation = Constants.HubLocations.BLUE.translation3d;
        }
    }

    @Override
    public void initialize() {
        this.controller = new PIDController(kP, kI, kD);
        controller.setTolerance(TOLERANCE_DEGREES);
        
    }

    @Override
    public void execute() {
        controller.setPID(kP, kI, kD);
        Pose2d botPose = Vision.getLimelightPose();

        Translation2d targetPos = new Translation2d(hubLocation.getX(), hubLocation.getY()).minus(botPose.getTranslation());
        distanceInches = Math.sqrt(Math.pow(targetPos.getNorm(), 2) + Math.pow(hubLocation.getZ(), 2));
        Rotation2d angle = Rotation2d.fromRadians(Math.atan2(targetPos.getY(), targetPos.getX()));
        Rotation2d robotAngle = botPose.getRotation();

        delta = angle.minus(robotAngle)
                .plus(Rotation2d.fromDegrees(180))
                .getDegrees();

        final double outputPower = controller.calculate(delta);

        swerve.drive(botPose.getTranslation(), outputPower, true);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(delta) < TOLERANCE_DEGREES;
    }
}
