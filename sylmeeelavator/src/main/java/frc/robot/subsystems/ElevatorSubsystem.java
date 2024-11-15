// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.SnapToTargetPosition;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private CANSparkMax m_motor = new CANSparkMax(ElevatorConstants.kCanSparkMAXPort, MotorType.kBrushless);
  private DigitalInput m_homingSwitch = new DigitalInput(ElevatorConstants.kHomeSwitchDIO);

  private double encoderOffset = 0;
  private double setPoint = 0;

  //private SnapToTargetPosition m_PIDhandler;

  public ElevatorSubsystem() {
    
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command logSomethingCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          System.out.println(m_homingSwitch.get());
        });
  }

  public void setMotorSpeed(double val) {
    System.out.println(getCurrentPosition());

    if (isLimitSwitchHit()) {
      encoderOffset = m_motor.getEncoder().getPosition();
    }

    if (isLimitSwitchHit() && val < 0) {
      m_motor.set(0);

    } else if (getCurrentPosition() >= 0.19 && val > 0 ) {
      m_motor.set(0);

    } else {
      m_motor.set(val);
      
    }

  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean isLimitSwitchHit() {
    // Query some boolean state, such as a digital sensor.
    return m_homingSwitch.get();
  }

  public double getCurrentPosition() {
    return m_motor.getEncoder().getPosition() - encoderOffset;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // runOnce(new SnapToTargetPosition(setPoint, this));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
