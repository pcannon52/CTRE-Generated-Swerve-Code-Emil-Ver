// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Turret;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotArm;

/**
 * Manual control of turret using joysticks
 * Automatically stops shooter if turret is moving
 * This is typically set as the DEFAULT COMMAND for the turret
 */
public class ManualTurretControlCommand extends Command {
  private final RobotArm m_turret;
  private final DoubleSupplier m_yawSupplier;
  private final DoubleSupplier m_pitchSupplier;

  /**
   * Creates a manual control command
   * @param turret The turret subsystem
   * @param yawSupplier Supplier for yaw speed (-1.0 to 1.0)
   * @param pitchSupplier Supplier for pitch speed (-1.0 to 1.0)
   */
  public ManualTurretControlCommand(
      RobotArm turret,
      DoubleSupplier yawSupplier,
      DoubleSupplier pitchSupplier
  ) {
    m_turret = turret;
    m_yawSupplier = yawSupplier;
    m_pitchSupplier = pitchSupplier;
    
    addRequirements(turret);
  }

  @Override
  public void execute() {
    // Get joystick values every loop (20ms)
    double yawSpeed = m_yawSupplier.getAsDouble();
    double pitchSpeed = m_pitchSupplier.getAsDouble();
    
    // Control motors with joystick input
    m_turret.rotateYaw(yawSpeed);
    m_turret.movePitch(pitchSpeed);
    
    // Apply deadband
    if (Math.abs(yawSpeed) < 0.1) yawSpeed = 0.0;
    if (Math.abs(pitchSpeed) < 0.1) pitchSpeed = 0.0;

    // Safety: stop shooter if turret is moving
    if (Math.abs(yawSpeed) > 0.1 || Math.abs(pitchSpeed) > 0.1) {
      m_turret.stopShooter();
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Stop all movement when command ends
    m_turret.rotateYaw(0);
    m_turret.movePitch(0);
  }

  @Override
  public boolean isFinished() {
    // Never finishes on its own (runs until interrupted)
    return false;
  }
}