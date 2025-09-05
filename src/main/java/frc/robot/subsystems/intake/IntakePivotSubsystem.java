// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivotSubsystem extends SubsystemBase {
  public enum IntakeState {
    STOW(0),
    HANDOFF(0),
    INTAKE(0),
    L1(0);

    private double m_angle;
    
    IntakeState(double angle) {
      m_angle = angle;
    }

    public double getAngle() {
      return m_angle;
    }
  }
  /** Creates a new IntakePivot. */
  public IntakePivotSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
