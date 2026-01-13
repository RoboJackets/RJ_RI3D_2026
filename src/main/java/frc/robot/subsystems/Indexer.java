package frc.robot.subsystems;

import frc.robot.subsystems.base.SettableSpark;
import static frc.robot.Constants.*;

public class Indexer extends SettableSpark {
    public Indexer() {
        super("indexer", INDEXER_CAN_ID, false);
    }
}
