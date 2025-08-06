// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.OperatorConstants; 
import frc.robot.subsystem.drivetrain.SwerveDriveChassis;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers; 

public class RobotContainer {

  //private final SendableChooser<Command> autoChooser;

  /* functional default drive request object */
  private final SwerveRequest.FieldCentric m_DriveRequest = new SwerveRequest.FieldCentric()
    .withDeadband(RobotConstants.kMaxLinearSpeed.in(MetersPerSecond) * RobotConstants.kSwerveRequestDeadbandDrive)
    .withRotationalDeadband(RobotConstants.kMaxRotationalSpeed.in(RadiansPerSecond) * RobotConstants.kSwerveRequestDeadbandSteer)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.SwerveDriveBrake m_BrakeRequest = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt m_PointRequest = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric m_ForwardsRequest = new SwerveRequest.RobotCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final Telemetry m_TelemetryLog = new Telemetry(RobotConstants.kMaxLinearSpeed.in(MetersPerSecond));
  private final CommandXboxController m_DriveController = new CommandXboxController(OperatorConstants.kDriverPortId);
  private final CommandXboxController m_OperatorController = new CommandXboxController(OperatorConstants.kOperatorPortId);

  public final SwerveDriveChassis m_SwerveDriveChassis = RobotConstants.createSwerveDriveChassis(); 

  private final SendableChooser<Command> autonChooser; 

  public RobotContainer() {
    autonChooser = AutoBuilder.buildAutoChooser("default auto name");
    SmartDashboard.putData("Auton Mode", autonChooser);
    configureBindings();

    FollowPathCommand.warmupCommand().schedule(); // ready pathplanner for use 
  }

  private void configureBindings() {
    m_SwerveDriveChassis.setDefaultCommand(
      m_SwerveDriveChassis.applySwerveRequest(() ->
      m_DriveRequest.withVelocityX(RobotConstants.kMaxLinearSpeed.times(-m_DriveController.getLeftY()))
      .withVelocityY(RobotConstants.kMaxLinearSpeed.times(-m_DriveController.getLeftX())) 
      .withRotationalRate(RotationsPerSecond.of(0.75).in(RadiansPerSecond) * (-m_DriveController.getRightX()))) // adjust w@ as needed 
      );
    

      // apply idle / neutral mode to motors when bot is disabled
      final var idle = new SwerveRequest.Idle();
      RobotModeTriggers.disabled().whileTrue(
      m_SwerveDriveChassis.applySwerveRequest(() -> idle).ignoringDisable(true)
  );

        m_DriveController.a().whileTrue(m_SwerveDriveChassis.applySwerveRequest(() -> m_BrakeRequest));
        m_DriveController.b().whileTrue(m_SwerveDriveChassis.applySwerveRequest(() ->
        m_PointRequest.withModuleDirection(new Rotation2d(- m_DriveController.getLeftY(), - m_DriveController.getLeftX()))
        ));

        m_DriveController.pov(0).whileTrue(m_SwerveDriveChassis.applySwerveRequest(() ->
        m_ForwardsRequest.withVelocityX(0.5).withVelocityY(0))
        );
        m_DriveController.pov(180).whileTrue(m_SwerveDriveChassis.applySwerveRequest(() ->
        m_ForwardsRequest.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        m_DriveController.back().and(m_DriveController.y()).whileTrue(m_SwerveDriveChassis.sysIdDynamic(Direction.kForward));
        m_DriveController.back().and(m_DriveController.x()).whileTrue(m_SwerveDriveChassis.sysIdDynamic(Direction.kReverse));
        m_DriveController.start().and(m_DriveController.y()).whileTrue(m_SwerveDriveChassis.sysIdQuasistatic(Direction.kForward));
        m_DriveController.start().and(m_DriveController.x()).whileTrue(m_SwerveDriveChassis.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        m_DriveController.leftBumper().onTrue(m_SwerveDriveChassis.runOnce(() -> m_SwerveDriveChassis.seedFieldCentric()));

        m_SwerveDriveChassis.registerTelemetry(m_TelemetryLog::telemeterize);
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}
