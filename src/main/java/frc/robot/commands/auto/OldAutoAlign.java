package frc.robot.commands.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;
import static frc.robot.Utilities.*;


public class OldAutoAlign extends Command {
    // https://github.com/team1306/Robot2024/blob/main/src/main/java/frc/robot/commands/drive/ShooterDriveCommand.java
    private final SwerveSubsystem swerve;
    private PIDController controller;
    // shooting location must have an offset

    public static double kP = 0.01, kI = 0, kD = 0;
    private static final double TOLERANCE_DEGREES = 5;
    private final Translation3d hubLocation;

    public double delta = Double.POSITIVE_INFINITY;
    
    private double distanceInches;

    private double outputPower;

    private final SwerveInputStream swerveInputStream;

    public OldAutoAlign(SwerveSubsystem swerve, SwerveInputStream swerveInputStream) {
        this.swerve = swerve;
        this.swerveInputStream = swerveInputStream.copy().withControllerRotationAxis(() -> outputPower);
        this.addRequirements(swerve);

        if (swerve.isRedAlliance()) {
            hubLocation = Constants.HubLocations.RED.translation3d;
        } else {
            hubLocation = Constants.HubLocations.BLUE.translation3d;
        }
    }
    
    public OldAutoAlign(SwerveSubsystem swerve) {
        this(swerve, SwerveInputStream.of(swerve.getSwerveDrive(), () -> 0D, () -> 0D));
    }
    
    @Override
    public void initialize() {
        this.controller = new PIDController(kP, kI, kD);
        controller.setTolerance(TOLERANCE_DEGREES);
        
    }

    @Override
    public void execute() {
        controller.setPID(kP, kI, kD);

        final Pose2d botPose = swerve.getPose();

        Translation2d targetPos = new Translation2d(hubLocation.getX(), hubLocation.getY()).minus(botPose.getTranslation());
        distanceInches = targetPos.getNorm();
        Rotation2d angle = Rotation2d.fromRadians(Math.atan2(targetPos.getY(), targetPos.getX()));
        Rotation2d robotAngle = botPose.getRotation();

        delta = angle.minus(robotAngle)
                .plus(Rotation2d.fromDegrees(180))
                .getDegrees();

        outputPower = MathUtil.clamp(controller.calculate(delta), -1, -1);

        swerve.driveFieldOriented(swerveInputStream);

        SmartDashboard.putBoolean("view readyToShoot", readyToShoot());

        kP = getNumber("edit oldautoalign kP", kP);
        kI = getNumber("edit oldautoalign kI", kI);
        kD = getNumber("edit oldautoalign kD", kD);
    }
    
    public boolean readyToShoot() {
        return Math.abs(delta) < TOLERANCE_DEGREES;
    }

    double getHubDistance() {
        return distanceInches;
    }
}
