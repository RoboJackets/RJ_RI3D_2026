package frc.robot;

import java.util.HashSet;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Utilities {
    public static boolean applyTalonFXConfig(TalonFX talon, TalonFXConfiguration config) {
        for (int i = 0; i < 5; ++i) {
            if (talon.getConfigurator().apply(config).isOK()) return true;
        }
        System.out.println("Config could not be applied to Talon with CAN ID: " + talon.getDeviceID());
        return false;
    }

    private static HashSet<String> usedKeys = new HashSet<>(64);
    public static double getNumber(String key, double defaultNumber) {
        if (usedKeys.add(key)) {
            SmartDashboard.putNumber(key, defaultNumber);
            return defaultNumber;
        } else {
            return SmartDashboard.getNumber(key, defaultNumber);
        }
    }

    public static boolean getBoolean(String key, boolean defaultBoolean) {
        if (usedKeys.add(key)) {
            SmartDashboard.putBoolean(key, defaultBoolean);
            return defaultBoolean;
        } else {
            return SmartDashboard.getBoolean(key, defaultBoolean);
        }
    }
}
