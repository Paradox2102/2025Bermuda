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

  public SequentialCommandGroup goToLevel(ElevatorState state) {
    return new SequentialCommandGroup(m_pivotSubsystem.setPosition(IntakeState.STOW).until(m_pivotSubsystem.atPosition), m_elevatorSubsystem.setPosition(state).alongWith(m_armSubsystem.setPosition(state.getArmPos())));
  }

  public SequentialCommandGroup scoreResetElev() {
    return new SequentialCommandGroup(RunForTime(m_armSubsystem.scoreReef().alongWith(m_elevatorSubsystem.scoreCoral()), 1), m_elevatorSubsystem.setPosition(ElevatorState.STOW).alongWith(m_armSubsystem.setPosition(ArmState.STOW)));
  }

  public ParallelDeadlineGroup groundCoral() {
    return new ParallelDeadlineGroup(m_rollerSubsystem.intake(), new ParallelCommandGroup(m_elevatorSubsystem.setPosition(ElevatorState.STOW), m_armSubsystem.setPosition(ArmState.STOW), m_pivotSubsystem.setPosition(IntakeState.INTAKE)));
  }

  private Command SwitchModes(RobotState state) {
    return Commands.runOnce(() -> {
      m_state = state;
    }, this);
  }

  private Command TriggerSequence(Trigger trigger, SubsystemBase subsystem) {
    return Commands.run(null, subsystem).until(trigger);
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
