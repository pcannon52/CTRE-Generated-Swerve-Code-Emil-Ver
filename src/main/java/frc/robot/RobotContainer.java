// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric teleDrive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for tele op
   
            private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController m_Joystick = new CommandXboxController(0);
    public final CommandSwerveDrivetrain m_Drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        bindingConfigurations();
    }

    public void bindingConfigurations(){
        // Set default command - runs whenever no other command is using the drivetrain
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_Drivetrain.setDefaultCommand(
            m_Drivetrain.applyRequest(() -> teleDrive
                .withVelocityX(-m_Joystick.getLeftY() * MaxSpeed) // Forward/back
                .withVelocityY(m_Joystick.getLeftX() * MaxSpeed) // Left/right
                .withRotationalRate(-m_Joystick.getRightX() * MaxAngularRate) // Rotation
            )
        );
        
        // neutral mode is applied to the drive motors while disabled.
        // Make sure the motors don't work even when robot is disabled
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            m_Drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        m_Joystick.leftBumper().whileTrue(m_Drivetrain.applyRequest(() -> brake));
        m_Joystick.leftTrigger().whileTrue(m_Drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-m_Joystick.getLeftY(), m_Joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        m_Joystick.start().and(m_Joystick.x()).whileTrue(m_Drivetrain.sysIdTranslationQuasistatic(Direction.kForward));
        // tells it to run the test in reverse direc
        m_Joystick.start().and(m_Joystick.y()).whileTrue(m_Drivetrain.sysIdTranslationDynamic(Direction.kReverse));
        m_Joystick.start().and(m_Joystick.b()).whileTrue(m_Drivetrain.sysIdTranslationQuasistatic(Direction.kForward));
        m_Joystick.start().and(m_Joystick.a()).whileTrue(m_Drivetrain.sysIdTranslationQuasistatic(Direction.kReverse));

        m_Joystick.back().and(m_Joystick.x()).whileTrue(m_Drivetrain.sysIdRotationQuasistatic(Direction.kForward));
        m_Joystick.back().and(m_Joystick.y()).whileTrue(m_Drivetrain.sysIdRotationDynamic(Direction.kReverse));
        m_Joystick.back().and(m_Joystick.b()).whileTrue(m_Drivetrain.sysIdRotationQuasistatic(Direction.kForward));
        m_Joystick.back().and(m_Joystick.a()).whileTrue(m_Drivetrain.sysIdRotationQuasistatic(Direction.kReverse));

        m_Joystick.leftBumper().and(m_Joystick.x()).whileTrue(m_Drivetrain.sysIdSteerQuasistatic(Direction.kForward));
        m_Joystick.leftBumper().and(m_Joystick.y()).whileTrue(m_Drivetrain.sysIdSteerDynamic(Direction.kReverse));
        m_Joystick.leftBumper().and(m_Joystick.b()).whileTrue(m_Drivetrain.sysIdSteerQuasistatic(Direction.kForward));
        m_Joystick.leftBumper().and(m_Joystick.a()).whileTrue(m_Drivetrain.sysIdSteerQuasistatic(Direction.kReverse));
        //sets the new forward motion of a robot
        m_Joystick.leftStick().onTrue(m_Drivetrain.runOnce(() -> m_Drivetrain.seedFieldCentric()));

        m_Drivetrain.registerTelemetry(logger::telemeterize); // calls evrey time it regesters it telemetry
    }
    
    
    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
