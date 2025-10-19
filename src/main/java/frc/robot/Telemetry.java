package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Drivetrain.Drivetrain;

public class Telemetry extends SubsystemBase{
    public Drivetrain drivetrain = Drivetrain.getInstance();
    public StructPublisher<Pose2d> RobotPose; //機器人位置
    public StructPublisher<ChassisSpeeds> RobotSpeeds;//機器人速度
    public DoubleArrayPublisher MotorSpeeds, MotorCurrents;//馬達速度與電流
    public DoublePublisher BatteryVoltage, MatchTime;//電池電壓與比賽時間

    public static Telemetry telemetry;

    public Telemetry() {
        RobotPose = NetworkTableInstance.getDefault().getStructTopic("Drivetrain/RobotPose", Pose2d.struct).publish();
        RobotSpeeds = NetworkTableInstance.getDefault().getStructTopic("Drivetrain/RobotSpeeds", ChassisSpeeds.struct).publish();
        MotorSpeeds = NetworkTableInstance.getDefault().getDoubleArrayTopic("Drivetrain/MotorSpeeds").publish();
        MotorCurrents = NetworkTableInstance.getDefault().getDoubleArrayTopic("Drivetrain/MotorCurrents").publish();

        BatteryVoltage = NetworkTableInstance.getDefault().getDoubleTopic("Utils/BatteryVoltage").publish();
        MatchTime = NetworkTableInstance.getDefault().getDoubleTopic("Utils/MatchTime").publish();
    }

    @Override
    public void periodic(){
        RobotPose.set(drivetrain.PoseEstimator.getEstimatedPosition());
        RobotSpeeds.set(drivetrain.getChassisSpeeds());
        MotorSpeeds.set(new double[]{
            drivetrain.LeftMotor.getEncoder().getVelocity(),
            drivetrain.RightMotor.getEncoder().getVelocity(),
        });
        MotorCurrents.set(new double[]{
            drivetrain.LeftMotor.getOutputCurrent(),
            drivetrain.LeftBackMotor.getOutputCurrent(),
            drivetrain.RightMotor.getOutputCurrent(),
            drivetrain.RightBackMotor.getOutputCurrent()
        });

        BatteryVoltage.set(RobotController.getBatteryVoltage());
        MatchTime.set(DriverStation.getMatchTime());
    }

    @Override
    public void simulationPeriodic(){
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(drivetrain.SimSystem.getCurrentDrawAmps()));
    }

    public static Telemetry getInstance(){
        if(telemetry == null) telemetry = new Telemetry();
        
        return telemetry;
    }
}
