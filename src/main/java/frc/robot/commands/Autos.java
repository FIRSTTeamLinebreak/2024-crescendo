package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
    public static Command noOp() {
        return Commands.run(() -> {});
    }
}
