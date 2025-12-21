// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotArm extends SubsystemBase {
  //Creates two motors
  TalonFX yawRobotArmMotor = new TalonFX(1);
  TalonFX PitchRobotArmMotor= new TalonFX(2);
  
// Get position signals from each motor, actualle can line
private final StatusSignal<Angle> yaw_Arm_PositionAngle = yawRobotArmMotor.getPosition();
private final StatusSignal<Angle> pitch_Arm_PositionAngle = PitchRobotArmMotor.getPosition();

  public RobotArm(){
        //this code is only ran once  
    // Base configuration for all drive motors
      TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        
      // Optional: Factory reset to clear any old settings
      yawRobotArmMotor.getConfigurator().apply(new TalonFXConfiguration());
      PitchRobotArmMotor.getConfigurator().apply(new TalonFXConfiguration());

        // Current limits - IMPORTANT for drivetrain!
        driveConfig.CurrentLimits.SupplyCurrentLimit = 40; // how much current the motor can recieve
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true; //enables spikes in current
        driveConfig.CurrentLimits.StatorCurrentLimit = 80;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        
        // Ramp rate for smooth acceleration
        driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.2; // takes 0.2 seconds to get to 100%
        
        // Voltage compensation
        driveConfig.Voltage.PeakForwardVoltage = 12.0;
        driveConfig.Voltage.PeakReverseVoltage = -12.0;
        
        // Apply to motors
        PitchRobotArmMotor.getConfigurator().apply(driveConfig);
        yawRobotArmMotor.getConfigurator().apply(driveConfig);

  }


  public double yawPosistionValue(){
    return yaw_Arm_PositionAngle.refresh().getValueAsDouble(); //getPosition returns a Status angle
  }
  //returns the motor signal as a double
  public double rightPositionValue(){
    return pitch_Arm_PositionAngle.refresh().getValueAsDouble(); //getPosition returns a Status angle
  }


  @Override
  public void periodic() {
  // This method will be called once per scheduler run
    // SmartDashboard.putNumber("left position", yawPositionValue());
    SmartDashboard.putNumber("Right encoder", rightPositionValue());  }

}
