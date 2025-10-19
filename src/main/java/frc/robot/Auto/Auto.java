package frc.robot.Auto;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Drivetrain.Drivetrain;

public class Auto {
    public static Drivetrain drivetrain = Drivetrain.getInstance();

    public static Command getAuto(){
        return new SequentialCommandGroup(
            drivetrain.drive(() -> 0.2, () -> 0.0).withTimeout(Seconds.of(1)).andThen(drivetrain.drive(() -> 0.0, () -> 0.0))
        );
    }
}
