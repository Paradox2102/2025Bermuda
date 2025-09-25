// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.RobotState;
import frc.robot.subsystems.climber.CageCatchSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.scoring.ArmSubsystem;
import frc.robot.subsystems.scoring.ClawSubsystem;
import frc.robot.subsystems.scoring.ElevatorSubsystem;
import frc.robot.subsystems.scoring.ElevatorSubsystem.ElevatorState;

import java.io.File;
import swervelib.SwerveInputStream;


public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController m_driverController = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve"));
  private IntakePivotSubsystem m_pivotSubsystem = new IntakePivotSubsystem();
  private IntakeRollerSubsystem m_rollerSubsystem = new IntakeRollerSubsystem();
  private ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private ClawSubsystem m_clawSubsystem = new ClawSubsystem();
  private ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private CageCatchSubsystem m_cageSubsystem = new CageCatchSubsystem();
  private Superstructure m_superstructure = new Superstructure(m_pivotSubsystem, m_rollerSubsystem, m_elevatorSubsystem, m_armSubsystem, m_clawSubsystem, m_climberSubsystem, m_cageSubsystem);

  ElevatorState m_algae_level = ElevatorState.ALGAE_HIGH;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
      () -> m_driverController.getLeftY() * -1,
      () -> m_driverController.getLeftX() * -1)
      .withControllerRotationAxis(m_driverController::getRightX)
      .deadband(OperatorConstants.k_deadBand)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX,
      m_driverController::getRightY)
      .headingWhile(true);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
      () -> -m_driverController.getLeftY(),
      () -> -m_driverController.getLeftX())
      .withControllerRotationAxis(() -> m_driverController.getRawAxis(
          2))
      .deadband(OperatorConstants.k_deadBand)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
      .withControllerHeadingAxis(() -> Math.sin(
          m_driverController.getRawAxis(
              2) *
              Math.PI)
          *
          (Math.PI *
              2),
          () -> Math.cos(
              m_driverController.getRawAxis(
                  2) *
                  Math.PI)
              *
              (Math.PI *
                  2))
      .headingWhile(true)
      .translationHeadingOffset(true)
      .translationHeadingOffset(Rotation2d.fromDegrees(
          0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity);
    Command driveSetpointGen = m_swerveSubsystem.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedAnglularVelocityKeyboard = m_swerveSubsystem.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = m_swerveSubsystem.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    if (RobotBase.isSimulation()) {
      m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocityKeyboard);
    } else {
      m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    m_rollerSubsystem.setDefaultCommand(m_rollerSubsystem.hold());
    m_clawSubsystem.setDefaultCommand(m_clawSubsystem.hold());

    if (Robot.isSimulation()) {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
          Rotation2d.fromDegrees(90));
      driveDirectAngleKeyboard.driveToPose(() -> target,
          new ProfiledPIDController(5,
              0,
              0,
              new Constraints(5, 2)),
          new ProfiledPIDController(5,
              0,
              0,
              new Constraints(Units.degreesToRadians(360),
                  Units.degreesToRadians(180))));

    }
    m_superstructure.setDefaultCommand(m_superstructure.stow());
    //Bind different commands to buttons depending on whether or not the robot holds a coral
    if(m_superstructure.getState() == RobotState.INTAKE) {
        m_driverController.leftTrigger().toggleOnTrue(m_superstructure.groundAlgae());
        m_driverController.leftBumper().toggleOnTrue(m_superstructure.goToLevel(m_algae_level).andThen(m_clawSubsystem.intake().andThen(m_clawSubsystem.setGamePiece(true))));
        m_driverController.a().onTrue(m_superstructure.goToLevel(ElevatorState.NET).andThen(m_superstructure.TriggerSequence(m_driverController.a()).andThen(m_superstructure.scoreAlgaeReset())));
        m_driverController.y().onTrue(m_superstructure.goToLevel(ElevatorState.PROCESSOR).andThen(m_superstructure.TriggerSequence(m_driverController.y()).andThen(m_superstructure.scoreAlgaeReset())));
        m_driverController.rightTrigger().toggleOnTrue(m_superstructure.groundCoral());
        m_driverController.rightBumper().onTrue(m_superstructure.intakeL1());
        m_driverController.b().onTrue(m_superstructure.deployClimber().andThen(m_superstructure.TriggerSequence(m_driverController.b()).andThen(m_superstructure.climb())));
        m_driverController.x().onTrue(m_superstructure.goToHandoff().andThen(m_superstructure.TriggerSequence(m_driverController.x()).andThen(m_superstructure.handoff())));
    } else {

    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_swerveSubsystem.getAutonomousCommand("none");
  }

  public void setMotorBrake(boolean brake) {
    m_swerveSubsystem.setMotorBrake(brake);
  }
}