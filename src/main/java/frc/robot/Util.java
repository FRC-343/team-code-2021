package frc.robot;

public class Util {
    public static final double kDefaultDeadband = .02;
    
    public static double deadband(double value, double min) {
        if (Math.abs(value) > min) {
            if (value > 0.0) {
                return (value - min) / (1.0 - min);
            } else {
                return (value + min) / (1.0 - min);
            }
        } else {
            return 0.0;
        }
    }
    public static double deadband(double value) {
        return deadband(value, kDefaultDeadband);
    }
}