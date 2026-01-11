package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class Utilities {
    public static boolean applyTalonFXConfig(TalonFX talon, TalonFXConfiguration config) {
        for (int i = 0; i < 5; ++i) {
            if (talon.getConfigurator().apply(config).isOK()) return true;
        }
        System.out.println("Config could not be applied to Talon with CAN ID: " + talon.getDeviceID());
        return false;
    }
}
