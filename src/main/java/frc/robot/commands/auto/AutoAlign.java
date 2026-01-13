package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
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

    private Translation2d hubLocation;


    private double a_move, b_move, c_move;
    private boolean shootOnMove = false;

    private Regression timeOfFlightRegression = new Regression(a_move, b_move, c_move);


    public AutoAlign(SwerveSubsystem swerve, SwerveInputStream swerveInputStream) {
        this.swerve = swerve;
        this.hubLocation = Constants.HubLocations.RED.translation3d.toTranslation2d(); // temporary, will be set in initialize
        this.swerveInputStream = swerveInputStream.copy().aim(
            new Pose2d(
                hubLocation,
                Rotation2d.kZero
            )
        ).aimWhile(true);

        addRequirements(swerve);
    }
    
    public AutoAlign(SwerveSubsystem swerve) {
        this(swerve, SwerveInputStream.of(swerve.getSwerveDrive(), () -> 0D, () -> 0D));
    }

    @Override
    public void initialize() {
        this.hubLocation = (swerve.isRedAlliance() ? Constants.HubLocations.RED.translation3d : Constants.HubLocations.BLUE.translation3d).toTranslation2d();
    }
    
    @Override
    public void execute() {
        timeOfFlightRegression.a = a_move;
        timeOfFlightRegression.b = b_move;
        timeOfFlightRegression.c = c_move;

        Pose2d botPose = swerve.getPose();

        if (shootOnMove) {
            double timeOfFlight = timeOfFlightRegression.calculate(distanceInches);
            final Twist2d effectFromRobot = swerve.getRobotVelocity().toTwist2d(timeOfFlight);
            botPose = botPose.exp(effectFromRobot);
        }

        Translation2d targetPos = new Translation2d(hubLocation.getX(), hubLocation.getY()).minus(botPose.getTranslation());
        
        distanceInches = targetPos.getNorm();
        SmartDashboard.putNumber("view Auto Align Distance", distanceInches);

        swerve.driveFieldOriented(swerveInputStream.get());

        SmartDashboard.putBoolean("view Auto Align Running", true);
        SmartDashboard.putBoolean("view Aligned", readyToShoot());

        a_move = getNumber("edit a_move", a_move);
        b_move = getNumber("edit b_move", b_move);
        c_move = getNumber("edit c_move", c_move);

        shootOnMove = getBoolean("edit timeOfFlight", shootOnMove);

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

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("view Auto Align Running", false);
        SmartDashboard.putBoolean("view Aligned", false);
    }
}
