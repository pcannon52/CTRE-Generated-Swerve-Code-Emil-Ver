// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Turret;  // ← Make sure this matches your folder structure!

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.RobotArm;

/**
 * Complete shooting sequence:
 * 1. Stop shooter (safety)
 * 2. Aim at target position
 * 3. Spin up shooter
 * 4. Wait until at speed
 * 5. Hold for shooting duration
 * 6. Stop shooter
 */
public class AimAndShootCommand extends SequentialCommandGroup { // this is for auto
  
  /**
   * Creates an aim and shoot command with default 1 second shooting duration
   * @param turret The turret subsystem
   * @param yawDegrees Target yaw angle
   * @param pitchDegrees Target pitch angle
   */
  public AimAndShootCommand(RobotArm turret, double yawDegrees, double pitchDegrees) {
    this(turret, yawDegrees, pitchDegrees, 1.0);
  }

  /**
   * Creates an aim and shoot command with custom shooting duration
   * @param turret The turret subsystem
   * @param yawDegrees Target yaw angle
   * @param pitchDegrees Target pitch angle
   * @param shootDurationSeconds How long to hold at shooting speed
   */
  public AimAndShootCommand(RobotArm turret, double yawDegrees, double pitchDegrees, double shootDurationSeconds) {
    super(
      // Step 1: Stop shooter (safety - don't shoot while moving)
      Commands.runOnce(turret::stopShooter, turret),
      
      // Step 2: Aim at target position
      new AimAtCommand(turret, yawDegrees, pitchDegrees),
      
      // Step 3: Spin up shooter and wait until at speed
      Commands.run(turret::startShooter, turret)
        .until(turret::isShooterAtSpeed),
      
      // Step 4: Hold at shooting speed for specified duration
      Commands.run(turret::startShooter, turret)
        .withTimeout(shootDurationSeconds),
      
      // Step 5: Stop shooter
      Commands.runOnce(turret::stopShooter, turret)
    );
  }
}