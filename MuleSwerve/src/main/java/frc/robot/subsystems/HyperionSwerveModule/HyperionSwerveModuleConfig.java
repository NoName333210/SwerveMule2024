// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.HyperionSwerveModule;

import com.ctre.phoenix.motorcontrol.InvertType;

/** Defines configuration of a single module */
public class HyperionSwerveModuleConfig {

  /** Driving motor CAN id */
  public final int m_driveMotorId;

  /** Turning motor CAN id */
  public final int m_turnMotorId;

  /** Analog encoder value when module is at angle 0 */
  public final int m_analogZero;

  /** Analog channel of the module angle encoder */
  public final int m_turnEncoderChannel;

  /** Inversion of the drive motor */
  public final InvertType m_driveMotorInversion;

  /** Is the drive motor sensor out of phase */
  public final boolean m_driveSensorInvertPhase;

  /**
   * Constructs a configuration
   *
   * @param driveMotorId drive motor CAN id
   * @param turnMotorId turn motor CAN id
   * @param analogZero analog encoder value when module is at angle 0
   * @param driveMotorInversion drive motor inversion
   * @param driveSensorInvertPhase is the drive motor sensor phase inverted
   */
  public HyperionSwerveModuleConfig(
      int driveMotorId,
      int turnMotorId,
      int turnEncoderChannel,
      int analogZero,
      InvertType driveMotorInversion,
      boolean driveSensorInvertPhase) {

    m_driveMotorId = driveMotorId;
    m_turnMotorId = turnMotorId;
    m_turnEncoderChannel = turnEncoderChannel;
    m_analogZero = analogZero;
    m_driveMotorInversion = driveMotorInversion;
    m_driveSensorInvertPhase = driveSensorInvertPhase;
  }
}
