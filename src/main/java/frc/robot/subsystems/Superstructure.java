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
import edu.wpi.first.wpilibj2.command.Subsystem;
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
    return sequentialWithRequirements(
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          m_armSubsystem.setPosition(ArmState.STOW),
          m_pivotSubsystem.setPosition(IntakeState.STOW)), 
        m_elevatorSubsystem.setPosition(ElevatorState.STOW), 
        m_clawSubsystem.stop(), 
        m_cageSubsystem.stop(), 
        m_rollerSubsystem.stop()), this);
  }

  public SequentialCommandGroup goToLevel(ElevatorState state) {
    return sequentialWithRequirements(
      new SequentialCommandGroup(
        m_pivotSubsystem.setPosition(IntakeState.STOW)
        .alongWith(
          new ConditionalCommand(m_elevatorSubsystem.goUp(0.15), 
          new InstantCommand(),
          () -> m_elevatorSubsystem.getSetPoint() == ElevatorState.HANDOFF.getHeight())),
        setScoring(ElevatorState.STOW),
        m_armSubsystem.setPosition(state.getArmPos())
        .alongWith(
          m_elevatorSubsystem.setPosition(state))), this);
  }

  public SequentialCommandGroup goToAlgae(boolean high) {
    return sequentialWithRequirements(
      new SequentialCommandGroup(
        m_pivotSubsystem.setPosition(IntakeState.STOW)
        .alongWith(
          new ConditionalCommand(m_elevatorSubsystem.goUp(0.15), 
          new InstantCommand(),
          () -> m_elevatorSubsystem.getSetPoint() == ElevatorState.HANDOFF.getHeight())),
        setScoring(ElevatorState.STOW),
        m_armSubsystem.setPosition(ArmState.ALGAE)
        .alongWith(
          m_elevatorSubsystem.deployAlgae(high))), this);
  }

  public SequentialCommandGroup scoreCoralResetElev() {
    return sequentialWithRequirements(
      new SequentialCommandGroup(
        m_clawSubsystem.stop(), 
        m_armSubsystem.scoreCoral()
        .alongWith(
          new WaitCommand(0.15)
          .andThen(
            runForTime(
              m_elevatorSubsystem.scoreCoral(),
               0.75))),
      switchModes(RobotState.INTAKE),
      setScoring(ElevatorState.STOW), 
      m_armSubsystem.setPosition(ArmState.STOW), 
      m_elevatorSubsystem.setPosition(ElevatorState.HANDOFF)), this);
  }

  public ConditionalCommand scoreLevel(ElevatorState level) {
    return conditionalWithRequirements(new ConditionalCommand(scoreCoralResetElev().andThen(setScoring(ElevatorState.STOW)), goToLevel(level).andThen(setScoring(level)), () -> m_scoringLevel == level), this);
  }

  public ConditionalCommand clawL1() {
    return conditionalWithRequirements(new ConditionalCommand(runForTime(m_clawSubsystem.eject(), 0.5).andThen(switchModes(RobotState.INTAKE).andThen(setScoring(ElevatorState.STOW))), goToLevel(ElevatorState.L1).andThen(setScoring(ElevatorState.L1)), () -> m_scoringLevel == ElevatorState.L1), this);
  }

  public ParallelDeadlineGroup groundCoral() {
    return deadlineWithRequirements(new ParallelDeadlineGroup(m_rollerSubsystem.intake(), setScoring(ElevatorState.STOW), new ParallelCommandGroup(m_elevatorSubsystem.setPosition(ElevatorState.HANDOFF).andThen(m_armSubsystem.setPosition(ArmState.HANDOFF)), m_pivotSubsystem.setPosition(IntakeState.INTAKE))), this);
  }

  public ConditionalCommand intakeL1() {
    return conditionalWithRequirements(new ConditionalCommand(runForTime(m_rollerSubsystem.eject(), 0.5).andThen(setScoring(ElevatorState.STOW)), m_pivotSubsystem.setPosition(IntakeState.L1).andThen(setScoring(ElevatorState.L1)), () -> m_scoringLevel == ElevatorState.L1), this);
  }

  public SequentialCommandGroup goToHandoff() {
    return sequentialWithRequirements(new SequentialCommandGroup(m_elevatorSubsystem.setPosition(ElevatorState.HANDOFF), m_pivotSubsystem.setPosition(IntakeState.HANDOFF), m_armSubsystem.setPosition(ArmState.HANDOFF)), this);
  }

  public SequentialCommandGroup handoff() {
    return sequentialWithRequirements(new ParallelDeadlineGroup(m_clawSubsystem.intake(), m_elevatorSubsystem.handoff(), m_rollerSubsystem.eject()).andThen(switchModes(RobotState.CORAL).alongWith(m_rollerSubsystem.stop(), m_elevatorSubsystem.setPosition(ElevatorState.HANDOFF))), this);
  }

  public ParallelDeadlineGroup groundAlgae() {
    return deadlineWithRequirements(new ParallelDeadlineGroup(m_clawSubsystem.intake().andThen(m_clawSubsystem.setGamePiece(true)), setScoring(ElevatorState.STOW), new ParallelCommandGroup(m_elevatorSubsystem.setPosition(ElevatorState.GROUND_ALGAE), m_pivotSubsystem.setPosition(IntakeState.STOW)).andThen(m_armSubsystem.switchSides(false).andThen(m_armSubsystem.setPosition(ArmState.GROUND_ALGAE)))), this);
  }

  public SequentialCommandGroup reefAlgae(boolean high) {
    return sequentialWithRequirements(new SequentialCommandGroup(setScoring(ElevatorState.STOW), goToAlgae(high), m_clawSubsystem.eject(), m_clawSubsystem.setGamePiece(true)), this);
  }

  public ConditionalCommand scoreNet() {
    return conditionalWithRequirements(new ConditionalCommand(m_elevatorSubsystem.goUp(0.5).alongWith(m_armSubsystem.switchSides(false), new WaitCommand(0.125).andThen(runForTime(m_clawSubsystem.eject(), 1), m_clawSubsystem.setGamePiece(false))).andThen(setScoring(ElevatorState.STOW)), m_armSubsystem.switchSides(true).andThen(goToLevel(ElevatorState.NET), setScoring(ElevatorState.NET)), () -> m_scoringLevel == ElevatorState.NET), this);
  }

  public ConditionalCommand scoreProcessor() {
    return conditionalWithRequirements(new ConditionalCommand(runForTime(m_clawSubsystem.eject(), 0.5).andThen(m_armSubsystem.setPosition(ArmState.STOW)).andThen(setScoring(ElevatorState.STOW)), goToLevel(ElevatorState.PROCESSOR).andThen(setScoring(ElevatorState.PROCESSOR)), () -> m_scoringLevel == ElevatorState.PROCESSOR), this);
  }

  public SequentialCommandGroup deployClimber() {
    return sequentialWithRequirements(new SequentialCommandGroup(new ParallelCommandGroup(m_pivotSubsystem.setPosition(IntakeState.INTAKE), m_armSubsystem.switchSides(false).andThen(m_armSubsystem.setPosition(ArmState.ALGAE)), m_elevatorSubsystem.setPosition(ElevatorState.STOW)), m_cageSubsystem.run().alongWith(m_climberSubsystem.setPosition(ClimberState.EXTEND))), this);
  }

  public SequentialCommandGroup climb() {
    return sequentialWithRequirements(new SequentialCommandGroup(m_climberSubsystem.setPosition(ClimberState.CLIMB), m_cageSubsystem.stop()), this);
  }

  public ConditionalCommand climbSequence() {
    return conditionalWithRequirements(new ConditionalCommand(climb().andThen(toggleClimbing(false)), deployClimber().andThen(toggleClimbing(true)), () -> m_climberOut), this);
  }

  public SequentialCommandGroup cancelScoring() {
    return sequentialWithRequirements(new SequentialCommandGroup(setScoring(ElevatorState.STOW),m_clawSubsystem.setGamePiece(false), toggleClimbing(false), switchModes(RobotState.INTAKE), m_armSubsystem.setPosition(ArmState.STOW).deadlineFor(m_rollerSubsystem.eject()), stow()), this);
  }

  public Command switchModes(RobotState state) {
    return Commands.runOnce(() -> {
      m_state = state;
      System.out.println("SwitchModes");
    }, this);
  }

  private Command setScoring(ElevatorState level){
    return Commands.runOnce(() -> {m_scoringLevel = level;});
  }

  private Command toggleClimbing(boolean isClimbing){
    return Commands.runOnce(() -> {m_climberOut = isClimbing;});
  }

  public static SequentialCommandGroup sequentialWithRequirements(SequentialCommandGroup group, SubsystemBase req){
    group.addRequirements(req);
    return group;
  }

  public static ParallelDeadlineGroup deadlineWithRequirements(ParallelDeadlineGroup group, SubsystemBase req) {
    group.addRequirements(req);
    return group;
  }

  public static ConditionalCommand conditionalWithRequirements(ConditionalCommand conditional, SubsystemBase req){
    conditional.addRequirements(req);
    return conditional;
  }

  private ParallelDeadlineGroup runForTime(Command command, double time) {
    return new ParallelDeadlineGroup(new WaitCommand(time), command, Commands.startEnd(() -> {
      System.out.println("runfortime start");
    }, () -> {
      System.out.println("runfortime end");
    }));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putString("Control Mode", m_state.getName());
    m_hasGamePiece = m_rollerSubsystem.hasGamePiece() || m_clawSubsystem.hasGamePiece();
  //   SmartDashboard.putBoolean("Has Gamepiece", m_hasGamePiece);
  //   SmartDashboard.putBoolean("isclimbing", m_climberOut);
  }
}
