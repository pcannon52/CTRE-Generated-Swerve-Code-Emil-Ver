// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.generated;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class GyroReset extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final double resetAngle;
  
  /**
   * Creates a new GyroReset that resets the yaw to 0 degrees
   * @param drivetrain The swerve drivetrain subsystem
   */
  public GyroReset(CommandSwerveDrivetrain drivetrain) {
    this(drivetrain, 0);
  }
  
  /**
   * Creates a new GyroReset that resets the yaw to a specific angle
   * @param drivetrain The swerve drivetrain subsystem
   * @param resetAngle The angle to reset to (in degrees)
   */
  public GyroReset(CommandSwerveDrivetrain drivetrain, double resetAngle) {
    this.drivetrain = drivetrain;
    this.resetAngle = resetAngle;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    drivetrain.getPigeon2().setYaw(resetAngle);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}