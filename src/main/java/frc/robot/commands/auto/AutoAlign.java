package frc.robot.commands.auto;

import badgerlog.annotations.Entry;
import badgerlog.annotations.EntryType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class AutoAlign extends Command {
    private static final double ROTATING_FAST_RAD_CUTOFF = Units.degreesToRadians(15);
    
    private SwerveSubsystem swerve; // final
    
    private double distanceInches;

    private final SwerveInputStream swerveInputStream;

    private final Translation2d hubLocation;

    public AutoAlign(SwerveSubsystem swerve, SwerveInputStream swerveInputStream) {
        this.swerve = swerve;
        this.hubLocation = (swerve.isRedAlliance() ? Constants.HubLocations.RED.translation3d : Constants.HubLocations.BLUE.translation3d).toTranslation2d();
        this.swerveInputStream = swerveInputStream.copy().aim(
            new Pose2d(
                hubLocation,
                Rotation2d.kZero
            )
        ).aimWhile(this::isScheduled);

        addRequirements(swerve);
    }
    
    public AutoAlign(SwerveSubsystem swerve) {
        this(swerve, SwerveInputStream.of(swerve.getSwerveDrive(), () -> 0D, () -> 0D));
    }
    
    @Override
    public void execute() {
        swerve.driveFieldOriented(swerveInputStream);
    }

    double getHubDistance() {
        return distanceInches;
    }

    @Entry(EntryType.PUBLISHER)
    @Override
    public boolean isScheduled() {
        return super.isScheduled();
    }

    @Entry(EntryType.PUBLISHER)
    public boolean readyToShoot() {
        return Math.abs(swerveInputStream.get().omegaRadiansPerSecond) < ROTATING_FAST_RAD_CUTOFF;   
    }
}
