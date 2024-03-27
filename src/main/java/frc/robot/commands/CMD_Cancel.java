// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class CMD_Cancel extends Command {

	private final Supplier<Boolean> cancel;
	private boolean isCommandDone = false;

	public CMD_Cancel(Supplier<Boolean> cancel) {
		this.cancel = cancel;
	}

	@Override
	public void initialize() {
		isCommandDone = false;
	}

	@Override
	public void execute() {
		if (cancel.get()) {
			isCommandDone = true;
		}
	}

	@Override
	public boolean isFinished() {
		if (isCommandDone) {
			return true;
		} else {
			return false;
		}
	}
}
