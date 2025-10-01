// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    return SequentialWithRequirements(new SequentialCommandGroup(new ParallelCommandGroup(m_armSubsystem.setPosition(ArmState.STOW), m_pivotSubsystem.setPosition(IntakeState.STOW)), m_elevatorSubsystem.setPosition(ElevatorState.STOW)));
  }

  public SequentialCommandGroup goToLevel(ElevatorState state) {
    return SequentialWithRequirements(new SequentialCommandGroup(m_pivotSubsystem.setPosition(IntakeState.STOW), m_elevatorSubsystem.setPosition(state).alongWith(m_armSubsystem.setPosition(state.getArmPos()))));
  }

  public SequentialCommandGroup scoreCoralResetElev() {
    return SequentialWithRequirements(new SequentialCommandGroup(RunForTime(m_armSubsystem.scoreCoral().alongWith(m_elevatorSubsystem.scoreCoral()), 1), SwitchModes(RobotState.INTAKE), m_elevatorSubsystem.setPosition(ElevatorState.STOW).alongWith(m_armSubsystem.setPosition(ArmState.STOW))));
  }

  public SequentialCommandGroup scoreLevel(ElevatorState level, Trigger trigger, Boolean left) {
    return SequentialWithRequirements(new SequentialCommandGroup(goToLevel(level), TriggerSequence(trigger), scoreCoralResetElev()));
  }

  public SequentialCommandGroup clawL1(Trigger trigger) {
    return SequentialWithRequirements(new SequentialCommandGroup(goToLevel(ElevatorState.L1), TriggerSequence(trigger), m_clawSubsystem.eject(), SwitchModes(RobotState.INTAKE), goToLevel(ElevatorState.STOW)));
  }

  public ParallelDeadlineGroup groundCoral() {
    return DeadlineWithRequirements(new ParallelDeadlineGroup(m_rollerSubsystem.intake(), new ParallelCommandGroup(m_elevatorSubsystem.setPosition(ElevatorState.STOW), m_armSubsystem.setPosition(ArmState.STOW), m_pivotSubsystem.setPosition(IntakeState.INTAKE))));
  }

  public SequentialCommandGroup intakeL1(Trigger trigger) {
    return SequentialWithRequirements(new SequentialCommandGroup(m_pivotSubsystem.setPosition(IntakeState.L1), TriggerSequence(trigger), m_rollerSubsystem.eject()));
  }

  public SequentialCommandGroup goToHandoff() {
    return SequentialWithRequirements(new SequentialCommandGroup(m_elevatorSubsystem.setPosition(ElevatorState.HANDOFF), new ParallelCommandGroup(m_armSubsystem.setPosition(ArmState.HANDOFF), m_pivotSubsystem.setPosition(IntakeState.HANDOFF))));
  }

  public ParallelDeadlineGroup handoff() {
    return DeadlineWithRequirements(new ParallelDeadlineGroup(m_clawSubsystem.intake().andThen(SwitchModes(RobotState.CORAL)), m_rollerSubsystem.eject()));
  }

  public ParallelDeadlineGroup groundAlgae() {
    return DeadlineWithRequirements(new ParallelDeadlineGroup(m_clawSubsystem.intake().andThen(m_clawSubsystem.setGamePiece(true)), new ParallelCommandGroup(m_elevatorSubsystem.setPosition(ElevatorState.GROUND_ALGAE), m_pivotSubsystem.setPosition(IntakeState.STOW)).andThen(m_armSubsystem.switchSides(false).andThen(m_armSubsystem.setPosition(ArmState.GROUND_ALGAE)))));
  }

  public SequentialCommandGroup scoreAlgaeReset() {
    return SequentialWithRequirements(new SequentialCommandGroup(RunForTime(m_clawSubsystem.eject(),1), m_clawSubsystem.setGamePiece(false), m_elevatorSubsystem.setPosition(ElevatorState.STOW).alongWith(m_armSubsystem.setPosition(ArmState.STOW))));
  }

  public SequentialCommandGroup deployClimber() {
    return SequentialWithRequirements(new SequentialCommandGroup(new ParallelCommandGroup(m_pivotSubsystem.setPosition(IntakeState.INTAKE), m_armSubsystem.setPosition(ArmState.STOW), m_elevatorSubsystem.setPosition(ElevatorState.STOW)), m_cageSubsystem.run(true), m_climberSubsystem.setPosition(ClimberState.EXTEND)));
  }

  public SequentialCommandGroup climb() {
    return SequentialWithRequirements(new SequentialCommandGroup(m_climberSubsystem.setPosition(ClimberState.CLIMB), m_cageSubsystem.stop()));
  }

  private Command SwitchModes(RobotState state) {
    return Commands.runOnce(() -> {
      m_state = state;
    });
  }

  public Command TriggerSequence(Trigger trigger) {
    return Commands.run(null).until(trigger);
  }

  public SequentialCommandGroup SequentialWithRequirements(SequentialCommandGroup group){
    group.addRequirements(this);
    return group;
  }

  public ParallelDeadlineGroup DeadlineWithRequirements(ParallelDeadlineGroup group) {
    group.addRequirements(this);
    return group;
  }

  private ParallelDeadlineGroup RunForTime(Command command, double time) {
    return new ParallelDeadlineGroup(new WaitCommand(time), command);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Control Mode", m_state.getName());
    m_hasGamePiece = m_rollerSubsystem.hasGamePiece() || m_clawSubsystem.hasGamePiece();
    SmartDashboard.putBoolean("Has Gamepiece", m_hasGamePiece);
  }
}
