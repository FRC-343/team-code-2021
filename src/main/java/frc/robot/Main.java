/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
// Ocean man, take me by the hand, lead me to the land that you understand
// Ocean man, the voyage to the corner of the globe is a real trip
// Ocean man, the crust of a tan man imbibed by the sand
// Soaking up the thirst of the land

// Ocean man, can you see through the wonder of amazement at the oberman
// Ocean man, the crust is elusive when it casts forth to the childlike man
// Ocean man, the sequence of a life form braised in the sand
// Soaking up the thirst of the land

// Ocean man, ocean man
// Ocean man

// Ocean man, take me by the hand, lead me to the land that you understand
// Ocean man, the voyage to the corner of the globe is a real trip
// Ocean man, the crust of a tan man imbibed by the sand
// Soaking up the thirst of the land

// Ocean man, can you see through the wonder of amazement at the oberman
// Ocean man, the crust is elusive when it casts forth to the childlike man
// Ocean man, the sequence of a life form braised in the sand
// Soaking up the thirst of the land

// Ocean man
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all.
 * Unless you know what you are doing, do not modify this file except to
 * change the parameter class to the startRobot call.
 */
public final class Main {
  private Main() {
  }

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
