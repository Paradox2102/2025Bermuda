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

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

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

  private SparkFlex m_leadMotor = new SparkFlex(Constants.CANIDConstants.elev_leader, MotorType.kBrushless);
  private SparkFlex m_followMotor = new SparkFlex(Constants.CANIDConstants.elev_follower, MotorType.kBrushless);
  private SparkSim m_motorSim = new SparkSim(m_leadMotor, DCMotor.getNeoVortex(2));

  //kv and ka calculated from reca.lc
  private ElevatorSim m_elevatorSim = new ElevatorSim(ElevatorConstants.k_v, ElevatorConstants.k_a, DCMotor.getNeoVortex(2), 0, 1.42, true, 0);

  private ProfiledPIDController m_pid = new ProfiledPIDController(ElevatorConstants.k_p, ElevatorConstants.k_i, ElevatorConstants.k_d, new Constraints(ElevatorConstants.k_maxVel, ElevatorConstants.k_maxAccel));
  private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(ElevatorConstants.k_g, ElevatorConstants.k_v, ElevatorConstants.k_a);
  private double m_output = 0;

  private double m_simHeight = 0;

  private RelativeEncoder m_encoder = m_leadMotor.getEncoder();
  private DigitalInput m_limitSwitch = new DigitalInput(0);

  private ElevatorState m_state = ElevatorState.STOW;
  
  public Trigger atPosition = new Trigger(
    () -> Math.abs(getPosition() - m_state.getHeight()) < ElevatorConstants.k_deadzone);

  public Trigger limit = new Trigger(
    () -> m_limitSwitch.get());

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    m_leadMotor.configure(ElevatorConstants.elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_followMotor.configure(ElevatorConstants.followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getPosition() {
    return m_encoder.getPosition();
  }

  public double getVelocity() {
    return m_encoder.getVelocity();
  }

  public Command setPosition(ElevatorState pos) {
    return Commands.runOnce(() -> {m_state = pos;}, this);
  }

  public ElevatorState getSetPoint() {
    return m_state;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_pid.setGoal(m_state.getHeight());
    if(RobotBase.isReal()) {
      m_output = m_pid.calculate(getPosition()) + 
        (m_feedforward.calculate(m_pid.getSetpoint().velocity) / RobotController.getBatteryVoltage());
    } else {
      m_output = m_pid.calculate(getPosition()) + 
        (m_feedforward.calculate(m_pid.getSetpoint().velocity) / RoboRioSim.getVInVoltage());
    }
    m_leadMotor.set(m_output);
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
