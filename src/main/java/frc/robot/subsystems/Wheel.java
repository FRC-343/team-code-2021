package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wheel extends SubsystemBase {
    private final Spark m_wheel = new Spark(10);
    private final DoubleSolenoid m_wheelLift = new DoubleSolenoid(1, 6, 7);

    private final ColorSensorV3 m_color = new ColorSensorV3(I2C.Port.kOnboard);

    private final ColorMatch m_colorMatcher = new ColorMatch();

    private static final Color kRed = new Color(0.518311, 0.344971, 0.136963);
    private static final Color kGreen = new Color(0.1689, 0.575439, 0.25585);
    private static final Color kBlue = new Color(0.1267, 0.4160, 0.4575);
    private static final Color kYellow = new Color(0.320068, 0.558105, 0.122070);

    public Wheel() {
        m_colorMatcher.addColorMatch(kRed);
        m_colorMatcher.addColorMatch(kGreen);
        m_colorMatcher.addColorMatch(kBlue);
        m_colorMatcher.addColorMatch(kYellow);

        SendableRegistry.setSubsystem(m_wheel, this.getClass().getSimpleName());
        SendableRegistry.setName(m_wheel, "Wheel");

        SendableRegistry.setSubsystem(m_wheelLift, this.getClass().getSimpleName());
        SendableRegistry.setName(m_wheelLift, "Wheel Lift");
    }
    
    @Override
    public void periodic() {
        ColorMatchResult detectedColor = m_colorMatcher.matchClosestColor(m_color.getColor());

        if (detectedColor.color == kRed) {
            SmartDashboard.putString("color_detected", "red");
        } else if (detectedColor.color == kGreen) {
            SmartDashboard.putString("color_detected", "green");
        } else if (detectedColor.color == kBlue) {
            SmartDashboard.putString("color_detected", "blue");
        } else if (detectedColor.color == kYellow) {
            SmartDashboard.putString("color_detected", "yellow");
        } else {
            SmartDashboard.putString("color_detected", "None Colors there be");
        }
    }

    public void raise() {
        m_wheelLift.set(DoubleSolenoid.Value.kReverse);
    }

    public void lower() {
        m_wheelLift.set(DoubleSolenoid.Value.kForward);
    }

    public void raiseOrLower() {
        if (m_wheelLift.get() == DoubleSolenoid.Value.kReverse) {
            lower();
        } else {
            raise();
        }
    }

    public void setWheel(double speed) {
        m_wheel.set(speed);
    }


}
