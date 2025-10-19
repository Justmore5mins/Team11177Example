package frc.robot;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.urcl.URCL;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Elevator.Elevator;

public class Telemetry extends SubsystemBase{
    public Drivetrain drivetrain = Drivetrain.getInstance();
    public Elevator elevator = Elevator.getInstance();
    public StructPublisher<Pose2d> RobotPose; //機器人位置
    public StructPublisher<ChassisSpeeds> RobotSpeeds;//機器人速度
    public DoubleArrayPublisher MotorSpeeds, MotorCurrents;//馬達速度與電流
    public DoublePublisher BatteryVoltage, MatchTime;//電池電壓與比賽時間
    public StringPublisher DrivetrainNowDoing, ElevatorNowDoing; //目前子系統正在做的事

    public static Telemetry telemetry;
    private boolean isDataLogStarted = false;

    public Telemetry() {
        RobotPose = NetworkTableInstance.getDefault().getStructTopic("Drivetrain/RobotPose", Pose2d.struct).publish();
        RobotSpeeds = NetworkTableInstance.getDefault().getStructTopic("Drivetrain/RobotSpeeds", ChassisSpeeds.struct).publish();
        MotorSpeeds = NetworkTableInstance.getDefault().getDoubleArrayTopic("Drivetrain/MotorSpeeds").publish();
        MotorCurrents = NetworkTableInstance.getDefault().getDoubleArrayTopic("Drivetrain/MotorCurrents").publish();

        BatteryVoltage = NetworkTableInstance.getDefault().getDoubleTopic("Utils/BatteryVoltage").publish();
        MatchTime = NetworkTableInstance.getDefault().getDoubleTopic("Utils/MatchTime").publish();
        DrivetrainNowDoing = NetworkTableInstance.getDefault().getStringTopic("NowDoing/Drivetrain").publish();
        ElevatorNowDoing = NetworkTableInstance.getDefault().getStringTopic("NowDoing/Elevator").publish();
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
        DrivetrainNowDoing.set(drivetrain.NowDoing);
        ElevatorNowDoing.set(elevator.NowDoing);

        if(!isDataLogStarted){
            DataLog();
            isDataLogStarted = true;
        }
    }

    @Override
    public void simulationPeriodic(){
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(drivetrain.SimSystem.getCurrentDrawAmps()));
    }

    //用AdvantageKit來記錄資料
    public void DataLog(){
        Logger.recordOutput("Drivetrain/RobotPose", drivetrain.PoseEstimator.getEstimatedPosition());
        Logger.recordOutput("Drivetrain/ChassisSpeeds", drivetrain.getChassisSpeeds());
        Logger.recordMetadata("NowDoing/Drivetrain", drivetrain.NowDoing);
        Logger.recordOutput("NowDoing/Elevator", elevator.NowDoing);
        Logger.registerURCL(URCL.startExternal());
        Logger.setReplaySource(new WPILOGReader("home/lvuser/logs/%s.wplog".formatted(LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss")))));
    }

    public static Telemetry getInstance(){
        if(telemetry == null) telemetry = new Telemetry();
        
        return telemetry;
    }
}
