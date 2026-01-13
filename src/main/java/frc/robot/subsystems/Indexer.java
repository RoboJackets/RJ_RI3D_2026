package frc.robot.subsystems;

import frc.robot.subsystems.base.SettableSpark;
import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.Utilities.*;

public class Indexer extends SettableSpark {
    private static double defaultPower = 1;

    public Indexer() {
        super("indexer", INDEXER_CAN_ID, false, () -> Indexer.defaultPower);
    }

    @Override
    public void periodic() {
        Indexer.defaultPower = getNumber("edit indexer defaultPower", Indexer.defaultPower);
        SmartDashboard.putNumber("view indexer currentSetPower", getPower());
    }
}
