package frc.robot;

public class RobotConstants {
    public static final boolean kPractice = false;

    // PUT CONSTANTS HERE WITHOUT VALUE AND THEN PUT IN ROBOT-SPECIFIC FILES
    public final double kHoodMotor;

    private static RobotConstants m_instance;

    public static RobotConstants getInstance() {
        if (m_instance == null) {
            if (kPractice)
                m_instance = new PracticeConstants();
            else
                m_instance = new CompConstants();
        }

        return m_instance;
    }
}
