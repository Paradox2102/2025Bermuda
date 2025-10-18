// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.climber.CageCatchSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem.ClimberState;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem.IntakeState;
import frc.robot.subsystems.scoring.ArmSubsystem;
import frc.robot.subsystems.scoring.ClawSubsystem;
import frc.robot.subsystems.scoring.ElevatorSubsystem;
import frc.robot.subsystems.scoring.ArmSubsystem.ArmState;
import frc.robot.subsystems.scoring.ElevatorSubsystem.ElevatorState;

public class Superstructure extends SubsystemBase {
  public enum RobotState {
    INTAKE("Intake/Algae"),
    CORAL("Coral");

    private String m_name;
    
    RobotState(String name) {
      m_name = name;
    }

    public String getName() {
      return m_name;
    }
  }

  private RobotState m_state = RobotState.INTAKE;

  private IntakePivotSubsystem m_pivotSubsystem;
  private IntakeRollerSubsystem m_rollerSubsystem;
  private ElevatorSubsystem m_elevatorSubsystem;
  private ArmSubsystem m_armSubsystem;
  private ClawSubsystem m_clawSubsystem;
  private ClimberSubsystem m_climberSubsystem;
  private CageCatchSubsystem m_cageSubsystem;

  private boolean m_hasGamePiece = false;
  private ElevatorState m_scoringLevel;
  private boolean m_climberOut = false;

  /** Creates a new Superstructure. */
  public Superstructure(IntakePivotSubsystem pivot, IntakeRollerSubsystem rollers, ElevatorSubsystem elevator, ArmSubsystem arm, ClawSubsystem claw, ClimberSubsystem climber, CageCatchSubsystem cage) {
    m_pivotSubsystem = pivot;
    m_rollerSubsystem = rollers;
    m_elevatorSubsystem = elevator;
    m_armSubsystem = arm;
    m_clawSubsystem = claw;
    m_climberSubsystem = climber;
    m_cageSubsystem = cage;
  }

  public RobotState getState() {
    return m_state;
  }

  public SequentialCommandGroup stow() {
    return SequentialWithRequirements(
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          m_armSubsystem.setPosition(ArmState.STOW),
          m_pivotSubsystem.setPosition(IntakeState.STOW)), 
        m_elevatorSubsystem.setPosition(ElevatorState.STOW), 
        m_clawSubsystem.stop(), 
        m_cageSubsystem.stop(), 
        m_rollerSubsystem.stop()));
  }

  public SequentialCommandGroup goToLevel(ElevatorState state) {
    return SequentialWithRequirements(
      new SequentialCommandGroup(
        m_pivotSubsystem.setPosition(IntakeState.STOW)
        .alongWith(
          new ConditionalCommand(m_elevatorSubsystem.goUp(), 
          new InstantCommand(),
          () -> m_elevatorSubsystem.getSetPoint() == ElevatorState.HANDOFF.getHeight())),
        SetScoring(ElevatorState.STOW),
        m_armSubsystem.setPosition(state.getArmPos())
        .alongWith(
          m_elevatorSubsystem.setPosition(state))));
  }

  public SequentialCommandGroup scoreCoralResetElev() {
    return SequentialWithRequirements(
      new SequentialCommandGroup(
        m_clawSubsystem.stop(), 
        m_armSubsystem.scoreCoral()
        .alongWith(
          new WaitCommand(0.15)
          .andThen(
            RunForTime(
              m_elevatorSubsystem.scoreCoral(),
               1))),
      SwitchModes(RobotState.INTAKE),
      SetScoring(ElevatorState.STOW), 
      m_armSubsystem.setPosition(ArmState.STOW), 
      m_elevatorSubsystem.setPosition(ElevatorState.STOW)));
  }

  public ConditionalCommand scoreLevel(ElevatorState level, Boolean left) {
    return ConditionalWithRequirements(new ConditionalCommand(scoreCoralResetElev().andThen(SetScoring(ElevatorState.STOW)), goToLevel(level).andThen(SetScoring(level)), () -> m_scoringLevel == level));
  }

  public ConditionalCommand clawL1() {
    return ConditionalWithRequirements(new ConditionalCommand(m_clawSubsystem.eject().andThen(SwitchModes(RobotState.INTAKE).andThen(SetScoring(ElevatorState.STOW))), goToLevel(ElevatorState.L1).andThen(SetScoring(ElevatorState.L1)), () -> m_scoringLevel == ElevatorState.L1));
  }

  public ParallelDeadlineGroup groundCoral() {
    return DeadlineWithRequirements(new ParallelDeadlineGroup(m_rollerSubsystem.intake(), SetScoring(ElevatorState.STOW), new ParallelCommandGroup(m_elevatorSubsystem.setPosition(ElevatorState.HANDOFF).andThen(m_armSubsystem.setPosition(ArmState.HANDOFF)), m_pivotSubsystem.setPosition(IntakeState.INTAKE))));
  }

  public ConditionalCommand intakeL1() {
    return ConditionalWithRequirements(new ConditionalCommand(m_rollerSubsystem.eject().andThen(SetScoring(ElevatorState.STOW)), m_pivotSubsystem.setPosition(IntakeState.L1).andThen(SetScoring(ElevatorState.L1)), () -> m_scoringLevel == ElevatorState.L1));
  }

  public SequentialCommandGroup goToHandoff() {
    return SequentialWithRequirements(new SequentialCommandGroup(m_elevatorSubsystem.setPosition(ElevatorState.HANDOFF), m_pivotSubsystem.setPosition(IntakeState.HANDOFF), m_armSubsystem.setPosition(ArmState.HANDOFF)));
  }

  public SequentialCommandGroup handoff() {
    return SequentialWithRequirements(new ParallelDeadlineGroup(m_clawSubsystem.intake(), m_elevatorSubsystem.handoff(), m_rollerSubsystem.eject()).andThen(SwitchModes(RobotState.CORAL).alongWith(m_rollerSubsystem.stop(), m_elevatorSubsystem.setPosition(ElevatorState.HANDOFF))));
  }

  public ParallelDeadlineGroup groundAlgae() {
    return DeadlineWithRequirements(new ParallelDeadlineGroup(m_clawSubsystem.intake().andThen(m_clawSubsystem.setGamePiece(true)), SetScoring(ElevatorState.STOW), new ParallelCommandGroup(m_elevatorSubsystem.setPosition(ElevatorState.GROUND_ALGAE), m_pivotSubsystem.setPosition(IntakeState.STOW)).andThen(m_armSubsystem.switchSides(false).andThen(m_armSubsystem.setPosition(ArmState.GROUND_ALGAE)))));
  }

  public SequentialCommandGroup reefAlgae() {
    return SequentialWithRequirements(new SequentialCommandGroup(SetScoring(ElevatorState.STOW), goToLevel(m_elevatorSubsystem.getAlgaeLevel().getAsBoolean() ? ElevatorState.ALGAE_HIGH : ElevatorState.ALGAE_LOW), m_clawSubsystem.intake(), m_clawSubsystem.setGamePiece(true)));
  }

  public SequentialCommandGroup scoreAlgaeReset(boolean net) {
    return SequentialWithRequirements(new SequentialCommandGroup(RunForTime(new WaitCommand(0.1).andThen(m_clawSubsystem.eject()).alongWith(net ? m_elevatorSubsystem.goUp() : new InstantCommand()),1), m_clawSubsystem.setGamePiece(false), m_elevatorSubsystem.setPosition(ElevatorState.STOW).alongWith(m_armSubsystem.setPosition(ArmState.STOW))));
  }

  public ConditionalCommand scoreNet() {
    return ConditionalWithRequirements(new ConditionalCommand(scoreAlgaeReset(true).andThen(SetScoring(ElevatorState.STOW)), goToLevel(ElevatorState.NET).andThen(SetScoring(ElevatorState.NET)), () -> m_scoringLevel == ElevatorState.NET));
  }

  public ConditionalCommand scoreProcessor() {
    return ConditionalWithRequirements(new ConditionalCommand(scoreAlgaeReset(false).andThen(SetScoring(ElevatorState.STOW)), goToLevel(ElevatorState.PROCESSOR).andThen(SetScoring(ElevatorState.PROCESSOR)), () -> m_scoringLevel == ElevatorState.PROCESSOR));
  }

  public SequentialCommandGroup deployClimber() {
    return SequentialWithRequirements(new SequentialCommandGroup(new ParallelCommandGroup(m_pivotSubsystem.setPosition(IntakeState.INTAKE), m_armSubsystem.switchSides(false).andThen(m_armSubsystem.setPosition(ArmState.ALGAE_LOW)), m_elevatorSubsystem.setPosition(ElevatorState.STOW)), m_cageSubsystem.run().alongWith(m_climberSubsystem.setPosition(ClimberState.EXTEND))));
  }

  public SequentialCommandGroup climb() {
    return SequentialWithRequirements(new SequentialCommandGroup(m_climberSubsystem.setPosition(ClimberState.CLIMB), m_cageSubsystem.stop()));
  }

  public ConditionalCommand climbSequence() {
    return ConditionalWithRequirements(new ConditionalCommand(climb().andThen(ToggleClimbing(false)), deployClimber().andThen(ToggleClimbing(true)), () -> m_climberOut));
  }

  public SequentialCommandGroup cancelScoring() {
    return SequentialWithRequirements(new SequentialCommandGroup(SetScoring(ElevatorState.STOW), ToggleClimbing(false), SwitchModes(RobotState.INTAKE), m_armSubsystem.setPosition(ArmState.STOW), stow()));
  }

  private Command SwitchModes(RobotState state) {
    return Commands.runOnce(() -> {
      m_state = state;
      System.out.println("SwitchModes");
    });
  }

  private Command SetScoring(ElevatorState level){
    return Commands.runOnce(() -> {m_scoringLevel = level;});
  }

  private Command ToggleClimbing(boolean isClimbing){
    return Commands.runOnce(() -> {m_climberOut = isClimbing;});
  }

  public SequentialCommandGroup SequentialWithRequirements(SequentialCommandGroup group){
    group.addRequirements(this);
    return group;
  }

  public ParallelDeadlineGroup DeadlineWithRequirements(ParallelDeadlineGroup group) {
    group.addRequirements(this);
    return group;
  }

  public ConditionalCommand ConditionalWithRequirements(ConditionalCommand conditional){
    conditional.addRequirements(this);
    return conditional;
  }

  private ParallelDeadlineGroup RunForTime(Command command, double time) {
    return new ParallelDeadlineGroup(new WaitCommand(time), command, Commands.startEnd(() -> {
      System.out.println("runfortime start");
    }, () -> {
      System.out.println("runfortime end");
    }));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Control Mode", m_state.getName());
    m_hasGamePiece = m_rollerSubsystem.hasGamePiece() || m_clawSubsystem.hasGamePiece();
    SmartDashboard.putBoolean("Has Gamepiece", m_hasGamePiece);
    SmartDashboard.putBoolean("isclimbing", m_climberOut);
  }
}
