package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.auto.AlignAndFlywheel.Regression;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

import static frc.robot.Utilities.*;

public class AutoAlign extends Command {
    private static final double ROTATING_FAST_RAD_CUTOFF = Units.degreesToRadians(15);
    
    private SwerveSubsystem swerve; // final
    
    private double distanceInches;

    private final SwerveInputStream swerveInputStream;

    private final Translation2d hubLocation;


    private double a_move, b_move, c_move;
    
    private Regression shootOnMoveRegression = new Regression(a_move, b_move, c_move);


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
        shootOnMoveRegression.a = a_move;
        shootOnMoveRegression.b = b_move;
        shootOnMoveRegression.c = c_move;

        swerve.driveFieldOriented(swerveInputStream);

        SmartDashboard.putBoolean("view Auto Align Running", isScheduled());
        SmartDashboard.putBoolean("view Aligned", readyToShoot());

        a_move = getNumber("edit a_move", a_move);
        b_move = getNumber("edit b_move", b_move);
        c_move = getNumber("edit c_move", c_move);
    }

    double getHubDistance() {
        return distanceInches;
    }

    @Override
    public boolean isScheduled() {
        return super.isScheduled();
    }

    public boolean readyToShoot() {
        return Math.abs(swerveInputStream.get().omegaRadiansPerSecond) < ROTATING_FAST_RAD_CUTOFF;   
    }
}
