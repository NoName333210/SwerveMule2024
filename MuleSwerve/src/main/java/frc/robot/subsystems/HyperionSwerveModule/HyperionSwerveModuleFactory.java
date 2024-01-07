// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.HyperionSwerveModule;

import static frc.robot.Constants.HyperionSwerveModule.*;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveModuleFactory;

/** Implements the swerve factory for Hyperion custom module. */
public class HyperionSwerveModuleFactory implements SwerveModuleFactory {

  @Override
  public SwerveModule[] createModules() {
    var modules = new HyperionSwerveModule[kConfigs.length];
    for (int i = 0; i < modules.length; ++i) {
      modules[i] = new HyperionSwerveModule(kConfigs[i]);
    }
    return modules;
  }

  @Override
  public Translation2d[] getLocations() {
    return kLocations;
  }
}
