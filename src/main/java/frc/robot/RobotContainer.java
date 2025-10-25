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
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.RobotState;
import frc.robot.subsystems.climber.CageCatchSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem.IntakeState;
import frc.robot.subsystems.scoring.ArmSubsystem;
import frc.robot.subsystems.scoring.ClawSubsystem;
import frc.robot.subsystems.scoring.ElevatorSubsystem;
import frc.robot.subsystems.scoring.ArmSubsystem.ArmState;
import frc.robot.subsystems.scoring.ElevatorSubsystem.ElevatorState;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import swervelib.SwerveInputStream;


public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController m_driverController = new CommandXboxController(0);
  final CommandJoystick m_operatorController = new CommandJoystick(1);

  SendableChooser<String> m_autoSelect;
  // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve"));
  public IntakePivotSubsystem m_pivotSubsystem = new IntakePivotSubsystem();
  public IntakeRollerSubsystem m_rollerSubsystem = new IntakeRollerSubsystem();
  public ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  public ArmSubsystem m_armSubsystem = new ArmSubsystem();
  public ClawSubsystem m_clawSubsystem = new ClawSubsystem();
  public ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  public CageCatchSubsystem m_cageSubsystem = new CageCatchSubsystem();
  public Superstructure m_superstructure = new Superstructure(m_pivotSubsystem, m_rollerSubsystem, m_elevatorSubsystem, m_armSubsystem, m_clawSubsystem, m_climberSubsystem, m_cageSubsystem);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
      () -> m_driverController.getLeftY() * -1,
      () -> m_driverController.getLeftX() * -1)
      .withControllerRotationAxis(() -> -m_driverController.getRightX())
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

    public Trigger shouldAutoAlign = new Trigger(() -> m_operatorController.getThrottle() < 0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    addNamedCommands();
    configureBindings();
    updateAutoChooser();
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
    Command driveFieldOrientedAnglularVelocityKeyboard = m_swerveSubsystem.driveFieldOriented(driveAngularVelocityKeyboard);

    if (RobotBase.isSimulation()) {
      m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocityKeyboard);
    } else {
      m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    m_driverController.povUp().onTrue(m_swerveSubsystem.resetGyro());

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
                  Units.degreesToRadians(
                    180))));

    }
    //m_superstructure.setDefaultCommand(m_superstructure.stow());
    // //Bind different commands to buttons depending on whether or not the robot holds a coral
    m_driverController.leftTrigger().onTrue(new ConditionalCommand(
        m_superstructure.groundAlgae(), 
        m_superstructure.scoreLevel(ElevatorState.L4),
        () -> m_superstructure.getState() == RobotState.INTAKE));

    m_driverController.leftBumper().onTrue(new ConditionalCommand(
        m_superstructure.reefAlgae(), 
        m_superstructure.scoreLevel(ElevatorState.L3), 
        () -> m_superstructure.getState() == RobotState.INTAKE));

    m_driverController.a().onTrue(new ConditionalCommand(
        m_superstructure.scoreNet(), 
        m_superstructure.scoreLevel(ElevatorState.L2), 
        () -> m_superstructure.getState() == RobotState.INTAKE));

    m_driverController.y().onTrue(new ConditionalCommand(
        m_superstructure.scoreProcessor(), 
        m_superstructure.clawL1(), 
        () -> m_superstructure.getState() == RobotState.INTAKE));

     m_driverController.rightTrigger().onTrue(new ConditionalCommand(
        m_superstructure.groundCoral().andThen(m_superstructure.goToHandoff()), 
        m_superstructure.scoreLevel(ElevatorState.L4), 
        () -> m_superstructure.getState() == RobotState.INTAKE));

    m_driverController.rightBumper().onTrue(new ConditionalCommand(
        m_superstructure.intakeL1(), 
        m_superstructure.scoreLevel(ElevatorState.L3), 
        () -> m_superstructure.getState() == RobotState.INTAKE));

    m_driverController.b().whileTrue(new ConditionalCommand(
        m_climberSubsystem.runOut(false).alongWith(m_cageSubsystem.stop()), 
        new InstantCommand(), 
        () -> m_superstructure.getState() == RobotState.INTAKE));
    
    m_driverController.b().onTrue(new ConditionalCommand(
        new InstantCommand(),
        m_superstructure.scoreLevel(ElevatorState.L2),
        () -> m_superstructure.getState() == RobotState.INTAKE));

    m_driverController.x().onTrue(new ConditionalCommand(
        m_superstructure.goToHandoff().andThen(m_superstructure.handoff()), 
        /*m_armSubsystem.switchSides()*/new InstantCommand(), 
        () -> m_superstructure.getState() == RobotState.INTAKE));

    m_operatorController.button(1).onTrue(m_superstructure.cancelScoring());
    m_operatorController.button(2).whileTrue(m_climberSubsystem.runOut(true).alongWith(m_cageSubsystem.run(),  m_armSubsystem.switchSides(false).andThen(m_armSubsystem.setPosition(ArmState.ALGAE), m_elevatorSubsystem.runManual(false).until(() -> m_elevatorSubsystem.getPosition() < 0.05)), m_pivotSubsystem.setPosition(IntakeState.INTAKE), m_superstructure.switchModes(RobotState.INTAKE)));
    m_operatorController.button(3).whileTrue(m_elevatorSubsystem.runManual(true));
    m_operatorController.button(4).whileTrue(m_elevatorSubsystem.runManual(false));
    m_operatorController.button(5).whileTrue(m_pivotSubsystem.runDown());
    m_operatorController.button(6).onTrue(m_elevatorSubsystem.switchAlgae());
    m_operatorController.button(7).onTrue(m_superstructure.switchModes(RobotState.CORAL));
    //m_operatorController.button(7).onTrue(m_clawSubsystem.intake());
    // m_operatorController.button(9).whileTrue(m_elevatorSubsystem.sysIdQuasistatic(Direction.kForward));
    // m_operatorController.button(10).whileTrue(m_elevatorSubsystem.sysIdQuasistatic(Direction.kReverse));
    // m_operatorController.button(11).whileTrue(m_elevatorSubsystem.sysIdDynamic(Direction.kForward));
    // m_operatorController.button(12).whileTrue(m_elevatorSubsystem.sysIdDynamic(Direction.kReverse));
  }

   private void updateAutoChooser() {
    m_autoSelect = new SendableChooser<>();
    m_autoSelect.addOption("1 L4", "1 L4");
    m_autoSelect.addOption("Leave", "Leave");
    m_autoSelect.addOption("Nothing", "Nothing");
    SmartDashboard.putData(m_autoSelect);
  }

  private void addNamedCommands() {
    NamedCommands.registerCommand("L4", m_superstructure.goToLevel(ElevatorState.L4));
    NamedCommands.registerCommand("Score", m_superstructure.scoreCoralResetElev());
    NamedCommands.registerCommand("Algae", m_superstructure.reefAlgae());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_swerveSubsystem.getAutonomousCommand(m_autoSelect.getSelected());
  }

  public void setMotorBrake(boolean brake) {
    m_swerveSubsystem.setMotorBrake(brake);
  }
}