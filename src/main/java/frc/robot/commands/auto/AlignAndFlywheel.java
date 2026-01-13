package frc.robot.commands.auto;

import frc.robot.subsystems.RevShooterFlywheelSubsystem;
import frc.robot.subsystems.RevShooterFlywheelSubsystem.ControlMode;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;
import static frc.robot.Utilities.*;

public class AlignAndFlywheel extends AutoAlign {
    private double a_top, b_top, c_top;
    private double a_bot, b_bot, c_bot;

    public static class Regression {
        public double a, b, c;

        public Regression(double a, double b, double c) {
            this.a = a;
            this.b = b;
            this.c = c;
        }

        public double calculate(double distance) {
            double result = a * Math.pow(distance, 3);
            result += b * Math.pow(distance, 2);
            result += c * distance;
            return result;
        }
    }

    private Regression topRegression = new Regression(a_top, b_top, c_top);
    private Regression botRegression = new Regression(a_bot, b_bot, c_bot);

    private RevShooterFlywheelSubsystem shooter;

    private double delTop, delBot;
    private final double SPEED_DIFF_THRESHOLD = 25;

    public AlignAndFlywheel(SwerveSubsystem swerve, SwerveInputStream swerveInputStream, RevShooterFlywheelSubsystem shooter) {
        super(swerve, swerveInputStream);
        this.shooter = shooter;

        this.addRequirements(swerve, shooter);
    }

    @Override
    public void initialize() {
        super.initialize();
        shooter.setControlMode(ControlMode.VelocityPID);
    }

    @Override
    public void execute() {
        super.execute();
        
        // get distance, do regression (read print), and spin flywheel :D
        topRegression.a = a_top;
        topRegression.b = b_top;
        topRegression.c = c_top;

        botRegression.a = a_bot;
        botRegression.b = b_bot;
        botRegression.c = c_bot;

        final double hubDistance = getHubDistance();

        double topTarget = topRegression.calculate(hubDistance);
        delTop = topTarget - shooter.getCurrentTopRPM();
        double botTarget = botRegression.calculate(hubDistance);
        delBot = botTarget - shooter.getCurrentBottomRPM();

        shooter.setTopTargetVelocity(topTarget);
        shooter.setBottomTargetVelocity(botTarget);

        a_bot = getNumber("edit a_bot", a_bot);
        b_bot = getNumber("edit b_bot", b_bot);
        c_bot = getNumber("edit c_bot", c_top);

        a_top = getNumber("edit a_top", a_top);
        b_top = getNumber("edit b_top", b_top);
        c_top = getNumber("edit c_top", c_top);
    }

    @Override
    public boolean readyToShoot() {
        return super.readyToShoot()
        && Math.abs(delBot) < SPEED_DIFF_THRESHOLD
        && Math.abs(delTop) < SPEED_DIFF_THRESHOLD;
    }
}
