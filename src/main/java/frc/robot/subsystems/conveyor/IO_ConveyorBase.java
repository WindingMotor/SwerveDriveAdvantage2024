// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.conveyor;

import org.littletonrobotics.junction.AutoLog;

public interface IO_ConveyorBase {

  @AutoLog
  public static class ConveyorInputs {

    public double intakeMotorSpeed = 0.0;
    public double indexerMotorSpeed = 0.0;

    public double intakeMotorCurrent = 0.0;
    public double indexerMotorCurrent = 0.0;

    public boolean intakeSensorState = false;

    public boolean indexerInitalSensorState = false;
    public boolean indexerFinalSensorState = false;

    public boolean shooterFlag;
  }

  /**
   * Updates the inputs with the current values.
   *
   * @param inputs The inputs to update.
   */
  void updateInputs(ConveyorInputs inputs);

  void update(double intakeSpeed, double indexerSpeed);

  void stop();
}
