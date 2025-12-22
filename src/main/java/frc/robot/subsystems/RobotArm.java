// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Commands.Turret.AimAtCommand;

public class RobotArm extends SubsystemBase {
  // ===== HARDWARE =====
  private final TalonFX yawMotor = new TalonFX(1);     // Rotates turret left/right
  private final TalonFX pitchMotor = new TalonFX(2);   // Moves arm up/down
  private final TalonFX shooterMotor = new TalonFX(3); // Shoots game pieces (you'll need to add this motor)
  
  // ===== POSITION SIGNALS =====
  private final StatusSignal<Angle> yawPosition = yawMotor.getPosition();
  private final StatusSignal<Angle> pitchPosition = pitchMotor.getPosition();
  
  // ===== CONTROL REQUESTS =====
  private final PositionVoltage yawPositionControl = new PositionVoltage(0).withSlot(0);
  private final PositionVoltage pitchPositionControl = new PositionVoltage(0).withSlot(0);
  private final VelocityVoltage shooterVelocityControl = new VelocityVoltage(0).withSlot(0);
  private final VoltageOut shooterVoltageControl = new VoltageOut(0);
    // Motion Magic for smooth preset movements (optional upgrade)
  private final MotionMagicVoltage yawMotionMagic = new MotionMagicVoltage(0).withSlot(0);
  private final MotionMagicVoltage pitchMotionMagic = new MotionMagicVoltage(0).withSlot(0);
  
  
  // ===== STATE TRACKING =====
  private boolean isReadyToShoot = false;
  private double targetYaw = 0.0;
  private double targetPitch = 45.0;
  private double targetSpeed = 3.0; //in m/s

  public RobotArm() {
    configureYawMotor();
    configurePitchMotor();
    configureShooterMotor();
  }

  // ===== CONFIGURATION METHODS =====
  
  private void configureYawMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    // Current limits - important for mechanism safety
    config.CurrentLimits.SupplyCurrentLimit = 30;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 60;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    
    // Ramp rate for smooth movement
    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.2;
    config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;
    
    // Voltage limits
    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12.0;
    
    // PID gains for position control (TUNE THESE!)
    config.Slot0.kP = 24.0;  // Start with this, increase until it responds quickly
    config.Slot0.kI = 0.0;   // Usually not needed
    config.Slot0.kD = 0.2;   // Add damping if oscillating
    config.Slot0.kV = 0.12;  // Feedforward (optional)
    
    // Gear ratio - converts motor rotations to mechanism rotations
    config.Feedback.SensorToMechanismRatio = Constants.RobotArmConstants.kYAW_GEAR_RATIO;
    
    // Brake mode - holds position when not moving
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
    // Soft limits to prevent mechanism damage
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.RobotArmConstants.kYAW_MAX_ANGLE / 360.0;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.RobotArmConstants.kYAW_MIN_ANGLE / 360.0;
    
    yawMotor.getConfigurator().apply(config);
    yawMotor.setPosition(0); // Set current position as 0 degrees
  }
  
  private void configurePitchMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    // Current limits
    config.CurrentLimits.SupplyCurrentLimit = 30;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 60;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    
    // Ramp rates
    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.2;
    config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;
    
    // Voltage limits
    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12.0;
    
    // PID gains (TUNE THESE!)
    config.Slot0.kP = 24.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.2;
    config.Slot0.kV = 0.12;
    config.Slot0.kG = 0.3;  // Gravity compensation (helps hold position against gravity)
    
    // Gear ratio
    config.Feedback.SensorToMechanismRatio = Constants.RobotArmConstants.kPITCH_GEAR_RATIO;
    
    // Brake mode
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
    // Soft limits
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.RobotArmConstants.kPITCH_MAX_ANGLE / 360.0;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.RobotArmConstants.kPITCH_MIN_ANGLE / 360.0;
    
    pitchMotor.getConfigurator().apply(config);
    pitchMotor.setPosition(0.125); // Start at 45 degrees (1/8 rotations)
  }
  
  private void configureShooterMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    // Higher current limits for shooter
    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 120;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    
    // No ramp for shooter - we want instant response
    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.0;
    
    // Voltage limits
    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12.0;
    
    // PID gains for velocity control (TUNE THESE!)
    config.Slot0.kP = 0.1;   // Start small
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kV = 0.12;  // Feedforward: 12V / max velocity
    
    // Coast mode - less resistance when stopping
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    
    shooterMotor.getConfigurator().apply(config);
  }

  // ===== GETTER METHODS =====
  public TalonFX getYawMotor(){ // for commands
    return yawMotor;
  }

  public TalonFX getPitchMotor(){
    return pitchMotor;
  }

  /** Get current yaw angle in degrees */
  public double getYawAngle() {
    return yawPosition.refresh().getValueAsDouble() * 360.0; // Convert rotations to degrees
  }
  
  /** Get current pitch angle in degrees */
  public double getPitchAngle() {
    return pitchPosition.refresh().getValueAsDouble() * 360.0;
  }
  
  /** Get current shooter velocity in rotations per second */
  public double getShooterVelocity() {
    return shooterMotor.getVelocity().refresh().getValueAsDouble();
  }
  
  /** Check if yaw is at target position */
  public boolean isYawAtTarget() {
    return Math.abs(getYawAngle() - targetYaw) < Constants.RobotArmConstants.kYAW_TOLERANCE;
  }
  
  /** Check if pitch is at target position */
  public boolean isPitchAtTarget() {
    return Math.abs(getPitchAngle() - targetPitch) < Constants.RobotArmConstants.kPITCH_TOLERANCE;
  }
  
  /** Check if shooter is at target speed */
  public boolean isShooterAtSpeed() {
    return Math.abs(getShooterVelocity() - targetSpeed) < Constants.RobotArmConstants.kSHOOTER_SPEED_TOLERANCE;
  }
  
  /** Check if mechanism is ready to shoot (at position AND at speed) */
  public boolean isReadyToShoot() {
    return isYawAtTarget() && isPitchAtTarget() && isShooterAtSpeed();
  }
  
  /** Check if mechanism is currently moving */
  public boolean isMoving() {
    return !isYawAtTarget() || !isPitchAtTarget();
  }

   // ===== CONTROL METHODS =====
  
  /** 
   * Set target yaw angle with motion profile option
   * @param degrees Target angle in degrees
   * @param useMotionMagic If true, uses smooth motion magic control
   */
  public void setYawAngle(double degrees, boolean useMotionMagic) {
    // Clamp to limits
    targetYaw = Math.max(Constants.RobotArmConstants.kYAW_MIN_ANGLE, Math.min(Constants.RobotArmConstants.kYAW_MAX_ANGLE, degrees));
    
    if (useMotionMagic) {
      yawMotor.setControl(yawMotionMagic.withPosition(targetYaw / 360.0));
    } else {
      yawMotor.setControl(yawPositionControl.withPosition(targetYaw / 360.0));
    }
  }
  
  /** 
   * Set target yaw angle (default: direct position control)
   * @param degrees Target angle in degrees
   */
  public void setYawAngle(double degrees) {
    setYawAngle(degrees, false);
  }
  
  /** 
   * Set target pitch angle with motion profile option
   * @param degrees Target angle in degrees
   * @param useMotionMagic If true, uses smooth motion magic control
   */
  public void setPitchAngle(double degrees, boolean useMotionMagic) {
    // Clamp to limits
    targetPitch = Math.max(Constants.RobotArmConstants.kPITCH_MIN_ANGLE, Math.min(Constants.RobotArmConstants.kPITCH_MAX_ANGLE, degrees));
    
    if (useMotionMagic) {
      pitchMotor.setControl(pitchMotionMagic.withPosition(targetPitch / 360.0));
    } else {
      pitchMotor.setControl(pitchPositionControl.withPosition(targetPitch / 360.0));
    }
  }
  
  /** 
   * Set target pitch angle (default: direct position control)
   * @param degrees Target angle in degrees
   */
  public void setPitchAngle(double degrees) {
    setPitchAngle(degrees, false);
  }
  /** 
   * Manually rotate yaw (for joystick control)
   * @param speed Speed from -1.0 to 1.0
   */
  public void rotateYaw(double speed) {
    // Safety: clamp speed
    speed = Math.max(-1.0, Math.min(1.0, speed));
    
    // Apply deadband
    if (Math.abs(speed) < 0.1) {
      speed = 0.0;
    }
    
    yawMotor.setControl(shooterVoltageControl.withOutput(speed * 6.0)); // Max 6V for manual control
  }
  
  /** 
   * Manually move pitch (for joystick control)
   * @param speed Speed from -1.0 to 1.0
   */
  public void movePitch(double speed) {
    // Safety: clamp speed
    speed = Math.max(-1.0, Math.min(1.0, speed));
    
    // Apply deadband
    if (Math.abs(speed) < 0.1) {
      speed = 0.0;
    }
    
    pitchMotor.setControl(shooterVoltageControl.withOutput(speed * 6.0));
  }
  
  /** Start spinning up shooter */
  public void startShooter() {
    shooterMotor.setControl(shooterVelocityControl.withVelocity(Constants.RobotArmConstants.kSHOOTER_SHOOT_SPEED));
  }
  
  /** Stop shooter */
  public void stopShooter() {
    shooterMotor.setControl(shooterVelocityControl.withVelocity(Constants.RobotArmConstants.kSHOOTER_IDLE_SPEED));
  }
  
  /** Emergency stop - stops all motors immediately */
  public void emergencyStop() {
    yawMotor.stopMotor();
    pitchMotor.stopMotor();
    shooterMotor.stopMotor();
  }


  // ===== COMMAND FACTORY METHODS =====
  
  
  /**
   * Command to shoot - spins up shooter, waits until ready, then allows shooting
   * SAFETY: Will not spin shooter if mechanism is still moving!
   */
  public Command shoot() {
    return run(() -> {
      // Safety check: only spin up if not moving
      if (!isMoving()) {
        startShooter();
      } else {
        stopShooter();
      }
    }).until(this::isReadyToShoot)
      .withName("Shoot");
  }
  
  /**
   * Command to reset turret to home position (center, 45 degrees)
   */
  public Command goHome() {
    return new AimAtCommand(this, 0, 45.0);
  }
  
  /**
   * Command to stop all motors
   */
  public Command stopAll() {
    return runOnce(() -> {
      yawMotor.stopMotor();
      pitchMotor.stopMotor();
      stopShooter();
    }).withName("StopAll");
  }

  @Override
  public void periodic() {
    // Update telemetry every loop
    SmartDashboard.putNumber("Turret/Yaw Angle", getYawAngle());
    SmartDashboard.putNumber("Turret/Pitch Angle", getPitchAngle());
    SmartDashboard.putNumber("Turret/Shooter Velocity", getShooterVelocity());
    SmartDashboard.putBoolean("Turret/At Yaw Target", isYawAtTarget());
    SmartDashboard.putBoolean("Turret/At Pitch Target", isPitchAtTarget());
    SmartDashboard.putBoolean("Turret/Shooter At Speed", isShooterAtSpeed());
    SmartDashboard.putBoolean("Turret/Ready To Shoot", isReadyToShoot());
    SmartDashboard.putBoolean("Turret/Is Moving", isMoving());
    
    // Update ready-to-shoot state
    isReadyToShoot = isReadyToShoot();
  }
}