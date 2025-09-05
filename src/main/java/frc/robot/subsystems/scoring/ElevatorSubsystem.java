// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.scoring;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  public enum ElevatorState {
    STOW(0, 0),
    HANDOFF(0, 0),
    L2(0,0),
    L3(0,0),
    L4(0,0),
    GROUND_ALGAE(0,0),
    ALGAE_LOW(0,0),
    ALGAE_HIGH(0,0),
    PROCESSOR(0,0),
    NET(0,0),
    LOLLIPOP(0, 0);

    private double m_armAngle;
    private double m_height;
    
    ElevatorState(double height, double angle) {
      m_height = height;
      m_armAngle = angle;
    }

    public double getAngle() {
      return m_armAngle;
    }

    public double getHeight() {
      return m_height;
    }
  }
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
