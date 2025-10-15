// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.scoring;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.scoring.ArmSubsystem.ArmState;

public class ElevatorSubsystem extends SubsystemBase {
  public enum ElevatorState {
    STOW(0, ArmState.STOW, "Stow"),
    HANDOFF(0.7025, ArmState.HANDOFF, "Handoff"),
    L1(0.4, ArmState.L1, "L1"),
    L2(0.35, ArmState.L2, "L2"),
    L3(0.65, ArmState.L3, "L3"),
    L4(1.35, ArmState.L4, "L4"),
    GROUND_ALGAE(0, ArmState.GROUND_ALGAE, "Algae Ground"),
    ALGAE_LOW(0.3, ArmState.ALGAE_LOW, "Algae Low"),
    ALGAE_HIGH(0.5, ArmState.ALGAE_HIGH, "Algae High"),
    PROCESSOR(0, ArmState.PROCESSOR, "Processor"),
    NET(1.42, ArmState.NET, "Net"),
    LOLLIPOP(0, ArmState.LOLLIPOP, "Lollipop");

    private double m_height;
    private ArmState m_armPos;
    private String m_name;
    
    ElevatorState(double height, ArmState armPos, String name) {
      m_height = height;
      m_armPos = armPos;
      m_name = name;
    }

    public double getHeight() {
      return m_height;
    }

    public ArmState getArmPos() {
      return m_armPos;
    }

    public String getName() {
      return m_name;
    }
  }

  private SparkFlex m_leadMotor = new SparkFlex(Constants.CANIDConstants.elev_leader, MotorType.kBrushless);
  private SparkFlex m_followMotor = new SparkFlex(Constants.CANIDConstants.elev_follower, MotorType.kBrushless);
  private SparkSim m_motorSim = new SparkSim(m_leadMotor, DCMotor.getNeoVortex(2));

  private ElevatorState m_state = ElevatorState.STOW;

  //kv and ka calculated from reca.lc
  private ElevatorSim m_elevatorSim = new ElevatorSim(ElevatorConstants.k_v, ElevatorConstants.k_a, DCMotor.getNeoVortex(2), 0, 1.42, true, m_state.getHeight());

  private ProfiledPIDController m_pid = new ProfiledPIDController(ElevatorConstants.k_p, ElevatorConstants.k_i, ElevatorConstants.k_d, new Constraints(ElevatorConstants.k_maxVel, ElevatorConstants.k_maxAccel), 0.02);
  private ElevatorFeedforward m_feedforward = new ElevatorFeedforward(ElevatorConstants.k_s, ElevatorConstants.k_g, ElevatorConstants.k_v, ElevatorConstants.k_a);
  private double m_output = 0;

  private double m_simHeight = 0;

  private RelativeEncoder m_encoder = m_leadMotor.getEncoder();
  private DigitalInput m_limitSwitch = new DigitalInput(0);
  private double m_setPoint = m_state.getHeight();
  
  public Trigger atPosition = new Trigger(
    () -> Math.abs(getPosition() - m_state.getHeight()) < ElevatorConstants.k_deadzone);

  //TODO: change to atBottom
  public Trigger atBottom = new Trigger(
    () -> m_limitSwitch.get());

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    m_leadMotor.configure(ElevatorConstants.elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_followMotor.configure(ElevatorConstants.followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_pid.reset(0,0);
    //m_pid.setIZone(ElevatorConstants.k_izone);
  }

  public double getPosition() {
    if(RobotBase.isReal()){
      return m_encoder.getPosition();
    } else {
      return m_simHeight;
    }
  }

  public double getVelocity() {
    if(RobotBase.isReal()){
      return m_encoder.getVelocity();
    } else {
      return m_elevatorSim.getVelocityMetersPerSecond();
    }
  }

  public Command setPosition(ElevatorState pos) {
    return Commands.run(() -> {
      m_state = pos;
      m_pid.reset(getPosition(),getVelocity());
      m_setPoint = m_state.getHeight();
    }, this).until(atPosition);
  }

  public Command runUp(){
    return Commands.runEnd(() -> {
      m_leadMotor.setVoltage(6);
    }, ()-> {
      m_leadMotor.setVoltage(0);
    }, this);
  }

  public Command scoreCoral() {
    return Commands.runOnce(() -> {
      m_setPoint = m_state.getHeight() - (m_state == ElevatorState.L4 ? 2*ElevatorConstants.k_dunkHeight : ElevatorConstants.k_dunkHeight);
      System.out.println("Elevator Score");
    }, this);
  }

  public Command reset() {
    return Commands.runOnce(() -> {m_state = ElevatorState.STOW;}, this);
  }

  public double getSetPoint() {
    return m_setPoint;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_pid.setGoal(getSetPoint());
    double pid = m_pid.calculate(getPosition());
    m_output = pid + m_feedforward.calculate(m_pid.getSetpoint().velocity);
    m_leadMotor.setVoltage(m_output);
    SmartDashboard.putNumber("Elev Height", getPosition());
    SmartDashboard.putNumber("Output", m_output);
    SmartDashboard.putNumber("Feedforward", m_feedforward.calculate(m_pid.getSetpoint().velocity));
    SmartDashboard.putNumber("Pid", pid);
    SmartDashboard.putString("level", m_state.getName());
    SmartDashboard.putNumber("target vel", m_pid.getSetpoint().velocity);
    SmartDashboard.putNumber("velocity", getVelocity());
  }

  public void simulationPeriodic() {
    m_elevatorSim.setInput(m_motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    m_elevatorSim.update(0.02);
    m_motorSim.iterate(
      Units.radiansPerSecondToRotationsPerMinute(m_elevatorSim.getVelocityMetersPerSecond()),
      RoboRioSim.getVInVoltage(), 0.02);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
    m_simHeight = m_elevatorSim.getPositionMeters();
  }
}
