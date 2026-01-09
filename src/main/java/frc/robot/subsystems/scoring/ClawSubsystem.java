// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.scoring;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase {
  private SparkFlex m_clawMotor = new SparkFlex(CANIDConstants.claw, MotorType.kBrushless);
  private RelativeEncoder m_encoder = m_clawMotor.getEncoder();
  private SparkClosedLoopController m_pid;
  private boolean m_isAlgae = false;

  public final Trigger pickCoralAlgae = new Trigger(
      () -> Math.abs(getCurrentDraw()) > ClawConstants.k_stallCurrent && getSpeed() < (ClawConstants.k_inSpeed - ClawConstants.k_slowSpeed))
      .debounce(0.1, DebounceType.kRising);
  private boolean m_hasGamePiece = false;

  /** Creates a new RollerSubsystem. */
  public ClawSubsystem() {
    m_clawMotor.configure(ClawConstants.clawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_pid = m_clawMotor.getClosedLoopController();
  }

  public double getSpeed() {
    return m_encoder.getVelocity();
  }

  public double getCurrentDraw() {
    return m_clawMotor.getOutputCurrent();
  }

  public boolean hasGamePiece() {
    return m_hasGamePiece;
  }


  public void run(boolean in) {
    m_pid.setReference(in ? ClawConstants.k_inSpeed : ClawConstants.k_outSpeed, ControlType.kVelocity);
  }

  public Command stop(){
    return Commands.runOnce(() -> m_pid.setReference(0, ControlType.kVoltage), this);
  }

  public Command intake() {
    return Commands.runEnd(() -> {
      run(true);
    }, () -> {
      stop();
    }, this).until(pickCoralAlgae);
  }

  public Command eject() {
    return Commands.run(() -> {
      m_clawMotor.setVoltage(-3);;
    },this);
  }

  public Command deAlgae() {
    return Commands.runEnd(() -> {
      run(false);
    }, () -> {
      stop();
    }, this);
  }

  public Command setGamePiece(boolean algae){
    return Commands.runOnce(() -> {
      m_isAlgae = algae;
    }, this);
  }

  public boolean isAlgae() {
    return m_isAlgae;
  }

  public Command hold() {
    return Commands.runOnce(
      () -> {
        if(isAlgae()){
          m_clawMotor.setVoltage(0);
        } else {
          m_clawMotor.setVoltage(0);
        }
      }, this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Claw Speed", getSpeed());
    // SmartDashboard.putNumber("Claw Current", getCurrentDraw());
  }
}
