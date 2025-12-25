package frc.robot.Commands.Turret;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotArm;

/**
 * Rotates yaw or pitch at a constant rate
 * Used for fine adjustments - hold button to rotate continuously
 */
public class ConstantRateRotateCommand extends Command {
  private final RobotArm m_turret;
  private final Axis m_axis;
  private final double m_direction;
  private final double m_speedRPS;
  
  private final VelocityVoltage m_velocityControl;
  private final VoltageOut m_stopControl;

  /**
   * Enum to specify which axis to rotate
   */
  public enum Axis {
    YAW,
    PITCH
  }

  /**
   * Creates a constant rate rotation command
   * @param turret The turret subsystem
   * @param axis Which axis to rotate (YAW or PITCH)
   * @param direction 1.0 for positive direction, -1.0 for negative
   * @param speedRPS Rotation speed in rotations per second
   */
  public ConstantRateRotateCommand(RobotArm turret, Axis axis, double direction, double speedRPS) {
    m_turret = turret;
    m_axis = axis;
    m_direction = direction;
    m_speedRPS = speedRPS;
    
    // Pre-create control objects
    m_velocityControl = new VelocityVoltage(0).withSlot(0);
    m_stopControl = new VoltageOut(0);
    
    addRequirements(turret);
  }

  /**
   * Convenience constructor with default speed (180° in 10 seconds = 0.05 RPS)
   * @param turret The turret subsystem
   * @param axis Which axis to rotate
   * @param direction 1.0 for positive, -1.0 for negative
   */
  public ConstantRateRotateCommand(RobotArm turret, Axis axis, double direction) {
    this(turret, axis, direction, 0.05); // Default: 180° in 10s
  }

  @Override
  public void execute() {
    // Calculate target velocity
    double velocity = m_direction * m_speedRPS;
    
    // Apply to appropriate axis
    switch (m_axis) {
      case YAW:
        m_turret.getYawMotor().setControl(m_velocityControl.withVelocity(velocity));
        break;
      case PITCH:
        m_turret.getPitchMotor().setControl(m_velocityControl.withVelocity(velocity));
        break;
    }
    
    // Safety: stop shooter while rotating
    m_turret.stopShooter();
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the motor when button is released
    switch (m_axis) {
      case YAW:
        m_turret.getYawMotor().setControl(m_stopControl);
        break;
      case PITCH:
        m_turret.getPitchMotor().setControl(m_stopControl);
        break;
    }
  }

  @Override
  public boolean isFinished() {
    // Never finishes on its own - runs until button released
    return false;
  }
  
  // ===== FACTORY METHODS FOR CONVENIENCE =====
  
  /** Rotate yaw clockwise at default rate */
  public static ConstantRateRotateCommand yawClockwise(RobotArm turret) {
    return new ConstantRateRotateCommand(turret, Axis.YAW, 1.0);
  }
  
  /** Rotate yaw counter-clockwise at default rate */
  public static ConstantRateRotateCommand yawCounterClockwise(RobotArm turret) {
    return new ConstantRateRotateCommand(turret, Axis.YAW, -1.0);
  }
  
  /** Rotate pitch up at default rate */
  public static ConstantRateRotateCommand pitchUp(RobotArm turret) {
    return new ConstantRateRotateCommand(turret, Axis.PITCH, 1.0);
  }
  
  /** Rotate pitch down at default rate */
  public static ConstantRateRotateCommand pitchDown(RobotArm turret) {
    return new ConstantRateRotateCommand(turret, Axis.PITCH, -1.0);
  }
}