// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.HyperionSwerveModule.HyperionSwerveModuleFactory;
import java.util.function.DoubleSupplier;

public class DriveTrain extends SubsystemBase {

  public enum Mode {
    New,
    Last,
    Disabled
  }

  // Subsystem parameters
  public static final double kMaxModuleSpeed = 4.0;

  public static final double kMaxSpeedX = 4.0;
  public static final double kMaxSpeedY = 4.0;
  public static final double kMaxSpeedRot = 360.0;

  public static final double kMaxAccTrans = 2.0 * kMaxModuleSpeed;
  public static final double kMaxAccRot = 4.0 * kMaxSpeedRot;

  public static final double kJoystickDeadband = 0.15;

  public static final double kHoloKP = 2.0;
  public static final double kHoloKI = 0.0;
  public static final double kHoloKD = 0.0;

  public static final double kRotKP = 8.0;
  public static final double kRotKI = 0.0;
  public static final double kRotKD = 0.0;

  public double filteredX = 0;
  public static final double XScoringPos = 2;
  public static final double minYScoringPos = 0.5;
  public static final double maxYScoringPos = 5;
  public static final double scoringGridIncrements = (maxYScoringPos - minYScoringPos) / 8;
  public double YScoringPos = 2.75;

  // Member objects
  private final SwerveModuleFactory m_moduleFactory = new HyperionSwerveModuleFactory();
  private final SwerveModule[] m_modules = m_moduleFactory.createModules();
  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(m_moduleFactory.getLocations());

  Accelerometer m_accelerometer = new BuiltInAccelerometer();

  private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(kMaxAccTrans);
  private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(kMaxAccTrans);
  private final SlewRateLimiter m_zLimiter = new SlewRateLimiter(kMaxAccRot);

  private Field2d m_field = new Field2d();

  LinearFilter m_xAccel = LinearFilter.movingAverage(30);

  /** Creates a new DriveTrain. */
  public DriveTrain(){

    // Run path planning server
    SmartDashboard.putData("field", m_field);
  }

  @Override
  public void periodic() {
    // Call module periodic
    filteredX = m_xAccel.calculate(m_accelerometer.getX());

    for (final var module : m_modules) {
      module.periodic();
    }
  }

  /**
   * Drives the robot in closed-loop velocity
   *
   * @param xSpeed Velocity along x (m/s)
   * @param ySpeed Velocity along y (m/s)
   * @param rotSpeed Rotation speed around x (deg/s)
   * @param fieldRelative Velocity are field relative
   */
  private void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {
    rotSpeed = Math.toRadians(rotSpeed);
    var moduleStates = m_kinematics.toSwerveModuleStates( new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));

    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, kMaxModuleSpeed);
    for (int i = 0; i < m_modules.length; ++i) {
      m_modules[i].setDesiredState(moduleStates[i]);
      String module_state_name = String.format("module_state%d ", i);
      SmartDashboard.putNumber(module_state_name + "rotation", moduleStates[i].angle.getDegrees());
      SmartDashboard.putNumber(module_state_name + "speed", moduleStates[i].speedMetersPerSecond);
    }
  }

  /**
   * Drives the robot in closed-loop velocity
   *
   * @param moduleStates Swerve modules states
   */
  public void drive(SwerveModuleState[] moduleStates) {
    for (int i = 0; i < m_modules.length; ++i) {
      m_modules[i].setDesiredState(moduleStates[i]);
    }
  }

  /**
   * Scales a joystick input to make it suitable as a velocity value
   *
   * @param valueSupplier Raw joystick value supplier (-1, 1)
   * @param maxValue Maximum value in physical units
   * @param rateLimiter Slew rate limiter to limit rate of change
   * @return Scaled joystick value (physical units)
   */
  private static double scaleJoystickInput(
      DoubleSupplier valueSupplier, double maxValue, SlewRateLimiter rateLimiter) {

    double rawValue = valueSupplier.getAsDouble();
    double value =
        Math.abs(rawValue) > kJoystickDeadband ? rawValue * rawValue * Math.signum(rawValue) : 0.0;

    return rateLimiter.calculate(value * maxValue);
  }

  /**
   * Command that drives using axis from the provided suppliers
   *
   * @param xSpeed Velocity along x (-1, 1) supplier
   * @param ySpeed Velocity along y (-1, 1) supplier
   * @param rotSpeed Rotation speed around x (-1, 1) supplier
   * @param fieldRelative Velocity are field relative supplier
   * @return run command that runs forever
   */
  public Command driveCommand(
      DoubleSupplier xSpeed,
      DoubleSupplier ySpeed,
      DoubleSupplier rotSpeed,
      boolean fieldRelative) {

    return this.runOnce(
            () -> {
              this.m_xLimiter.reset(0.0);
              this.m_yLimiter.reset(0.0);
              this.m_zLimiter.reset(0.0);
            })
        .andThen(
            this.run(
                () ->
                    this.drive(
                        DriveTrain.scaleJoystickInput(xSpeed, kMaxSpeedX, m_xLimiter),
                        DriveTrain.scaleJoystickInput(ySpeed, kMaxSpeedY, m_yLimiter),
                        DriveTrain.scaleJoystickInput(rotSpeed, kMaxSpeedRot, m_zLimiter),
                        fieldRelative)));
  }

  public Command balance() {
    return this.run(() -> drive(0.6, 0, 0, true))
        .until(this::inAngle)
        .andThen(this.run(() -> drive(0.3, 0, 0, true)))
        .until(this::parallel)
        .andThen(this.runOnce(() -> drive(0, 0, 0, true)));
  }

  public boolean inAngle() {
    return filteredX > 0.3;
  }

  public boolean parallel() {
    return filteredX < 0.3 && filteredX > -0.3;
  }

  public Command stop() {
    return this.runOnce(() -> this.driveWithSpeed(0.0, 0.0, 0.0));
  }

  /**
   * @param toTheRight true means robot goes right, flips automatically with the color
   */
  public Command moveScorePosition(boolean toTheRight) {
    return this.runOnce(
        () -> {
          boolean temp = DriverStation.getAlliance() == Alliance.Red ? !toTheRight : toTheRight;
          this.incrementScorePosition(temp);
        });
  }

  /**
   * @param direction true means robot goes right
   */
  public void incrementScorePosition(boolean direction) {
    if (direction) YScoringPos += 0.5625;
    else YScoringPos -= 0.5625;
    System.out.println("incrementing" + YScoringPos);
    // clamps around min and max value to insure we don't run into the walls
    YScoringPos = MathUtil.clamp(YScoringPos, minYScoringPos, maxYScoringPos);
    double gridPos = ((YScoringPos - minYScoringPos) / 0.5625) + 1;
    if (DriverStation.getAlliance() == Alliance.Blue) gridPos = 10 - gridPos;
    SmartDashboard.putNumber("Grid Position (left to right)", gridPos);
    System.out.println(gridPos);
  }

  public Command driveWithSpeed(double speedX, double speedY, double rotation) {
    return this.run(() -> this.drive(speedX, speedY, rotation, true));
  }

  public Command driveWithSpeedWithTimeout(
      double speedX, double speedY, double rotation, double time) {
    return this.run(() -> this.drive(speedX, speedY, rotation, true)).withTimeout(time);
  }

}
