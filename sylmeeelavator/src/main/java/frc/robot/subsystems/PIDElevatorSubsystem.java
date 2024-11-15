package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ElevatorConstants;

public class PIDElevatorSubsystem extends PIDSubsystem {

  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();

  private static final double Kp = 7;
  private static final double Ki = 0;
  private static final double Kd = 0;

  /** The shooter subsystem for the robot. */
  public PIDElevatorSubsystem() {
    super(new PIDController(Kp, Ki, Kd));
    getController().setTolerance(ElevatorConstants.kTurnRateToleranceDegPerS);
    //m_shooterEncoder.setDistancePerPulse(ShooterConstants.kEncoderDistancePerPulse);
    setSetpoint(0.01);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    m_elevator.setMotorSpeed(output);
  }

  @Override
  public double getMeasurement() {
    return m_elevator.getCurrentPosition();
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  public void goToFloor1() {
    setSetpoint(0.1);
  }

  public void goToFloor2() {
    setSetpoint(0.18);
  }

//   public void runFeeder() {
//     m_feederMotor.set(ShooterConstants.kFeederSpeed);
//   }

//   public void stopFeeder() {
//     m_feederMotor.set(0);
//   }
}