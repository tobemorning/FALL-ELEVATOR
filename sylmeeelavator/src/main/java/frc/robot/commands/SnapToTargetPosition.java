// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController; 
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

/** An example command that uses an example subsystem. */
public class SnapToTargetPosition extends ProfiledPIDCommand {
  /**
   * Turns to robot to the specified angle using a motion profile.
   *
   * @param targetEncoderValue The angle to turn to
   * @param elevator The drive subsystem to use
   */
  private static final double Kp = 5;
  private static final double Ki = 0;
  private static final double Kd = 0;
  
  public SnapToTargetPosition(double targetEncoderValue, ElevatorSubsystem elevator) {
    super(
        new ProfiledPIDController(
            Kp, Ki, Kd,
            new TrapezoidProfile.Constraints(
                ElevatorConstants.kMaxTurnRateDegPerS,
                ElevatorConstants.kMaxTurnAccelerationDegPerSSquared)),
        // Close loop on heading
        elevator::getCurrentPosition,
        // Set reference to target
        targetEncoderValue,
        // Pipe output to turn robot
        (output, setpoint) -> elevator.setMotorSpeed(output),
        // Require the drive
        elevator);

    // Set the controller to be continuous (because it is an angle controller)
    // getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the referexnce
    getController().disableContinuousInput();

    getController().setTolerance(ElevatorConstants.kTurnToleranceDeg, ElevatorConstants.kTurnRateToleranceDegPerS);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atGoal();
  }
}
