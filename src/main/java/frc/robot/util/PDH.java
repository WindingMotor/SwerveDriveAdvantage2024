// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class PDH {

  PowerDistribution PDH;

  public PDH() {
    PDH = new PowerDistribution(1, ModuleType.kRev);
    PDH.setSwitchableChannel(true);
  }

  public void setSwitchableChannel(boolean switchable) {
    PDH.setSwitchableChannel(switchable);
  }
}
