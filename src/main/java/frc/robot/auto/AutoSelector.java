// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoSelector {

	private SendableChooser<Command> autoSelector;

	public AutoSelector() {

		// Build the auto selector from PathPlanner auto files and set the default auto to A1_D1_SM
		autoSelector = AutoBuilder.buildAutoChooser("A1_D1_SM");

		// Not sure why this is needed, the auto selector should apply all PathPlannerAuto's options by
		// default
		// autoSelector.addOption("A1_D1_SM", getSelectedAuto());

		SmartDashboard.putData("Auto", autoSelector);
	}

	/**
	 * Gets the command of the selected auto.
	 *
	 * @command The command of the selected auto
	 */
	public Command getSelectedAuto() {
		return autoSelector.getSelected();
	}

	/**
	 * Gets the starting pose of the selected auto.
	 *
	 * @return The starting pose of the selected auto
	 */
	public Pose2d getStartingPose() {
		return PathPlannerAuto.getStaringPoseFromAutoFile(autoSelector.getSelected().getName());
	}
}
