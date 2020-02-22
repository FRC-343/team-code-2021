package frc.robot;

public class RobotConstants {
    public static final boolean kPractice = false;

    // PUT CONSTANTS HERE WITH A DEFAULT VALUE AND THEN PUT IN ROBOT-SPECIFIC FILES
    public final int kHoodMotor = -1;

    private static RobotConstants m_instance;

    public static RobotConstants getInstance() {
        if (m_instance == null) {
            if (kPractice) {
                m_instance = new PracticeRobotConstants();
            }
            else {
                m_instance = new CompRobotConstants();
            }
        }

        return m_instance;
    }
}
