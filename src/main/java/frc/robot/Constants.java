// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {

    public class RobotArmConstants{
          // ===== CONSTANTS =====
  // Yaw (rotation) limits
  public static final double kYAW_MIN_ANGLE = -180.0;  // degrees
  public static final double kYAW_MAX_ANGLE = 180.0;   // degrees
  
  // Pitch (elevation) limits
  public static final double kPITCH_MIN_ANGLE = 0.0;   // degrees (horizontal)
  public static final double kPITCH_MAX_ANGLE = 90.0;  // degrees (vertical)
  
  // Shooter speeds
  public static final double kSHOOTER_IDLE_SPEED = 0.0;     // rotations per second
  public static final double kSHOOTER_SHOOT_SPEED = 80.0;   // rotations per second (adjust for your robot)
  public static final double kSHOOTER_SPEED_TOLERANCE = 2.0; // rps tolerance to consider "at speed"
  
  // Movement tolerances
  public static final double kYAW_TOLERANCE = 2.0;    // degrees
  public static final double kPITCH_TOLERANCE = 2.0;  // degrees
  
  // Gear ratios (adjust these for your mechanism!)
  public static final double kYAW_GEAR_RATIO = 100.0;    // Example: 100:1 reduction
  public static final double kPITCH_GEAR_RATIO = 100.0;  // Example: 100:1 reduction
}

}
