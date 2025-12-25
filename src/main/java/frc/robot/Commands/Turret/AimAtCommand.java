package frc.robot.Commands.Turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotArm;

/**
 * Command to aim turret at a specific yaw and pitch angle
 * Completes when both angles are within tolerance
 */
public class AimAtCommand extends Command {
  private final RobotArm m_turret;
  private final double m_targetYaw;
  private final double m_targetPitch;
  private final boolean m_useMotionMagic;

  /**
   * Creates a command to aim at specific angles
   * @param turret The turret subsystem
   * @param yawDegrees Target yaw in degrees
   * @param pitchDegrees Target pitch in degrees
   */
  public AimAtCommand(RobotArm turret, double yawDegrees, double pitchDegrees) {
    this(turret, yawDegrees, pitchDegrees, false);
  }

  /**
   * Creates a command to aim at specific angles with motion profile option
   * @param turret The turret subsystem
   * @param yawDegrees Target yaw in degrees
   * @param pitchDegrees Target pitch in degrees
   * @param useMotionMagic If true, uses smooth motion magic control
   */
  public AimAtCommand(RobotArm turret, double yawDegrees, double pitchDegrees, boolean useMotionMagic) {
    m_turret = turret;
    m_targetYaw = yawDegrees;
    m_targetPitch = pitchDegrees;
    m_useMotionMagic = useMotionMagic;
    
    // This command requires the turret subsystem
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    // Set target angles once at start
    m_turret.setYawAngle(m_targetYaw, m_useMotionMagic);
    m_turret.setPitchAngle(m_targetPitch, m_useMotionMagic);
  }

  @Override
  public void execute() {
    // Keep commanding position for stability
    m_turret.setYawAngle(m_targetYaw, m_useMotionMagic);
    m_turret.setPitchAngle(m_targetPitch, m_useMotionMagic);
  }

  @Override
  public boolean isFinished() {
    // Finish when both axes are at target
    return m_turret.isYawAtTarget() && m_turret.isPitchAtTarget();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      // If interrupted, stop motors
      m_turret.stopAll();
    }
    // If finished normally, motors will hold position
  }
}