package frc.rosemont.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DashboardUtils {
    public static void report(String key, double value) {
        SmartDashboard.putNumber(key, value);
    }

    public static void report(String key, double[] value) {
        SmartDashboard.putNumberArray(key, value);
    }
    
    public static void report(String[] key, boolean[] value) {
        for (int i = 0; i < value.length; i++) {
            SmartDashboard.putBoolean(key[i], value[i]);
        }
    }

    public static void report(String[] key, String[] value) {
        for (int i = 0; i < value.length; i++) {
            SmartDashboard.putString(key[i], value[i]);
        }
    }

    public static void report(String key, Sendable value) {
        SmartDashboard.putData(key, value);
    }
}
