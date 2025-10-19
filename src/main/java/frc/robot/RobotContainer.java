// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Auto.Auto;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Elevator.Elevator;
import frc.utils.ReefHeight;

public class RobotContainer {
  public Drivetrain drivetrain = Drivetrain.getInstance();
  public Elevator elevator = Elevator.getInstance();
  public Joystick joystick = new Joystick(0);
  public Joystick elevatorJoystick = new Joystick(1);
  public Telemetry telemetry = new Telemetry();

  public RobotContainer() {

    // 設定預設命令, 搖桿的ID設定要看Driver的習慣來設定
    drivetrain.setDefaultCommand(drivetrain.drive(
      () -> -joystick.getRawAxis(1),  // 前後
      () -> joystick.getRawAxis(0)) // 左右
      );
    configureBindings();
  }

  private void configureBindings() {
    //所有的按鈕都可以根據driver的習慣來改
    new Trigger(() -> joystick.getRawButton(1)) 
      .onTrue(elevator.elevate(ReefHeight.L1)); //升到L1
    new Trigger(() -> joystick.getRawButton(2))
      .onTrue(elevator.elevate(ReefHeight.L2)); //升到L2
    new Trigger(() -> joystick.getRawButton(3))
      .onTrue(elevator.elevate(ReefHeight.L3)); //升到L3
    new Trigger(() -> joystick.getRawButton(5))
      .whileTrue(elevator.shooters.shootCoral(true));
    new Trigger(() -> joystick.getRawButton(6))
      .whileTrue(elevator.shooters.shootCoral(false));
    new Trigger(() -> joystick.getRawAxis(7) > 0.5)
      .whileTrue(elevator.shooters.shootAlgae(true));
    new Trigger(() -> joystick.getRawAxis(8) > 0.5)
      .whileTrue(elevator.shooters.shootAlgae(false));
    new Trigger(() -> joystick.getRawButton(9))
      .onTrue(elevator.seedPosition());
  }

  public Command getAutonomousCommand() {
    return Auto.getAuto();
  }
}
