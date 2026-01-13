package frc.robot.subsystems;

import frc.robot.subsystems.base.SettableSpark;
import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.Utilities.*;

public class Intake extends SettableSpark {
    private static double defaultPower = .5;

    public Intake () {
        super("intake", INTAKE_CAN_ID, false, () -> Intake.defaultPower);
    }

    @Override
    public void periodic() {
        Intake.defaultPower = getNumber("edit intake defaultPower", Intake.defaultPower);
        SmartDashboard.putNumber("view intake currentSetPower", getPower());
    }
}
