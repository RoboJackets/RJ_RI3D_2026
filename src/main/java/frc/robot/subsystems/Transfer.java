package frc.robot.subsystems;

import frc.robot.subsystems.base.SettableSpark;
import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.Utilities.*;

public class Transfer extends SettableSpark {
    private static double defaultPower = .5;

    public Transfer () {
        super("transfer", TRANSFER1_CAN_ID, false, () -> Transfer.defaultPower);
    }

    @Override
    public void periodic() {
        Transfer.defaultPower = getNumber("edit transfer defaultPower", Transfer.defaultPower);
        SmartDashboard.putNumber("view transfer currentSetPower", getPower());
    }
}
