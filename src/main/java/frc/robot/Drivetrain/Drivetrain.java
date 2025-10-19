package frc.robot.Drivetrain;

import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase{
    public SparkMax LeftMotor, RightMotor, LeftBackMotor, RightBackMotor;
    public RelativeEncoder LeftEncoder, RightEncoder;
    public SparkClosedLoopController LeftPID, RightPID;
    private SparkMaxConfig LeftConfig, RightConfig, LeftBackConfig, RightBaackConfig;

    public AHRS gyro;
    public DifferentialDrivePoseEstimator PoseEstimator;

    public static Drivetrain drivetrain;
    public String NowDoing = "Idle";

    //Simulation
    public SparkMaxSim LeftSim, RightSim;
    public SparkRelativeEncoderSim LeftEncoderSim, RightEncoderSim;
    public DifferentialDrivetrainSim SimSystem;

    private Drivetrain(){
        //把馬達實例化
        LeftMotor = new SparkMax(Constants.LeftIDs[0], MotorType.kBrushless);
        LeftBackMotor = new SparkMax(Constants.LeftIDs[1], MotorType.kBrushless);
        RightMotor = new SparkMax(Constants.RightIDs[0], MotorType.kBrushless);
        RightBackMotor = new SparkMax(Constants.RightIDs[1], MotorType.kBrushless);

        //還有一點跟馬達有關的東西
        LeftEncoder = LeftMotor.getEncoder();
        RightEncoder = RightMotor.getEncoder();
        LeftPID = LeftMotor.getClosedLoopController();
        RightPID = RightMotor.getClosedLoopController();

        //其他東西的實例化
        gyro = new AHRS(NavXComType.kMXP_SPI);
        PoseEstimator = new DifferentialDrivePoseEstimator(Constants.kinematics, gyro.getRotation2d(), getPosition().leftMeters, getPosition().rightMeters, Constants.InitialPose);

        LeftConfig = new SparkMaxConfig();
        RightConfig = new SparkMaxConfig();
        LeftBackConfig = new SparkMaxConfig();
        RightBaackConfig = new SparkMaxConfig();

        LeftConfig
            .idleMode(IdleMode.kBrake) //設定空轉模式
            .inverted(false) //設定馬達方向
            .voltageCompensation(12) //設定電壓補償
            .smartCurrentLimit(Constants.SlipCurrent); //設定電流限制
        LeftConfig.encoder
            .positionConversionFactor(Constants.PositionConvertionFactor) //把馬達的輸出量變成我們要的單位
            .velocityConversionFactor(Constants.VelocityConvertionFactor); //把馬達的輸出量變成我們要的單位
        LeftConfig.closedLoop.apply(Constants.LeftPID); //設定PID

        LeftBackConfig
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .voltageCompensation(12)
            .smartCurrentLimit(Constants.SlipCurrent);
        LeftBackConfig.follow(LeftMotor); //設定跟隨主馬達

        RightConfig
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .voltageCompensation(12)
            .smartCurrentLimit(Constants.SlipCurrent);
        RightConfig.encoder
            .positionConversionFactor(Constants.PositionConvertionFactor)
            .velocityConversionFactor(Constants.VelocityConvertionFactor);
        RightConfig.closedLoop.apply(Constants.RightPID);
        RightBaackConfig
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .voltageCompensation(12)
            .smartCurrentLimit(Constants.SlipCurrent);
        RightBaackConfig.follow(RightMotor);

        //應用設定
        LeftMotor.configure(LeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); // kResetSafeParameters: 把馬達回復到出廠設定(除了CAN ID) kPersistParameters: 把設定存到馬達裡面
        LeftBackMotor.configure(LeftBackConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        RightMotor.configure(RightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        RightBackMotor.configure(RightBaackConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
        if(RobotBase.isSimulation()) simInit();
        SmartDashboard.putData("Drivetrain", this);
    }

    /**
     * 取得位置
     * @return {@link #DifferentialDriveWheelPositions} 位置
     */
    public DifferentialDriveWheelPositions getPosition(){
        return (RobotBase.isReal()) ? 
        new DifferentialDriveWheelPositions(
            LeftEncoderSim.getPosition(),
            RightEncoderSim.getPosition()) : 
        new DifferentialDriveWheelPositions(
            LeftEncoder.getPosition(),
            RightEncoder.getPosition());
    }

    /**
     * 取得速度
     * @return {@link #DifferentialDriveWheelSpeeds} 速度
     */
    public DifferentialDriveWheelSpeeds getSpeeds(){
        return (RobotBase.isSimulation()) ? 
        new DifferentialDriveWheelSpeeds(
            LeftEncoderSim.getVelocity(),
            RightEncoderSim.getVelocity()) : 
        new DifferentialDriveWheelSpeeds(
            LeftEncoder.getVelocity(),
            RightEncoder.getVelocity());
    }

    /**
     * 取得底盤速度
     * @return {@link #ChassisSpeeds} 底盤速度
     */
    public ChassisSpeeds getChassisSpeeds(){
        return Constants.kinematics.toChassisSpeeds(getSpeeds());
    }

    /**
     * 直接設定馬達轉速
     * @param LeftSpeed 左馬達轉速(公尺/秒)
     * @param RightSpeed 右馬達轉速(公尺/秒)
     */
    public void drive(double LeftSpeed, double RightSpeed){
        if(RobotBase.isReal()){
            LeftMotor.set(LeftSpeed/(5676*Constants.VelocityConvertionFactor)); //設定馬達轉速
            RightMotor.set(RightSpeed/(5676*Constants.VelocityConvertionFactor)); //5676是馬達的最大RPM

            // LeftPID.setReference(LeftSpeed, ControlType.kVelocity);
            // RightPID.setReference(RightSpeed, ControlType.kVelocity);
        }else{
            LeftSim.setAppliedOutput(LeftSpeed/(5676*Constants.VelocityConvertionFactor));
            RightSim.setAppliedOutput(RightSpeed/(5676*Constants.VelocityConvertionFactor));
        }
    }

    /**
     * 使用供應器來驅動
     * @param throttle 前進/後退
     * @param turn 轉向
     * @return 命令
     */
    public Command drive(Supplier<Double> throttle, Supplier<Double> turn){
        return this.run(() -> {
            double leftSpeed = throttle.get() + turn.get();
            double rightSpeed = throttle.get() - turn.get();
            drive(leftSpeed, rightSpeed);
            NowDoing = "drive";
        }).withName(String.format("drive with throttle %.2f turn %.2f", throttle.get(), turn.get())).andThen(runOnce(() -> NowDoing = "Idle"));
    }

    public void simInit(){
        //Simulation
        LeftSim = new SparkMaxSim(LeftMotor, DCMotor.getNEO(2)); //NEO2個馬達
        RightSim = new SparkMaxSim(RightMotor, DCMotor.getNEO(2));//NEO2個馬達
        LeftEncoderSim = LeftSim.getRelativeEncoderSim();//取得編碼器模擬器
        RightEncoderSim = RightSim.getRelativeEncoderSim();//取得編碼器模擬器
        SimSystem = new DifferentialDrivetrainSim( 
            DCMotor.getNEO(2), 
            Constants.GearRatio, 
            Constants.MOI, 
            Constants.RobotMass, 
            Constants.WheelCirc/Math.PI/2, 
            Constants.kinematics.trackWidthMeters, 
            null); //建立模擬系統(各個東西的數值請自己按照實際情況調)
}

    @Override
    public void simulationPeriodic() {
        //設定模擬系統的輸入
        SimSystem.setInputs(
            LeftSim.getAppliedOutput() * 12,
            RightSim.getAppliedOutput() * 12
        );
        SimSystem.update(0.02);
    
        // 更新模擬編碼器
        LeftEncoderSim.setPosition(SimSystem.getLeftPositionMeters());
        RightEncoderSim.setPosition(SimSystem.getRightPositionMeters());
        LeftEncoderSim.setVelocity(SimSystem.getLeftVelocityMetersPerSecond());
        RightEncoderSim.setVelocity(SimSystem.getRightVelocityMetersPerSecond());
    
        // 更新模擬陀螺儀(因為陀螺儀方向跟我們定義的方向相反所以要乘-1)
        gyro.setAngleAdjustment(-SimSystem.getHeading().getDegrees());
    }

    @Override
    public void periodic(){
        PoseEstimator.update(gyro.getRotation2d(), getPosition()); //更新位置
    }

    //防止多重實例化
    public static Drivetrain getInstance(){
        if(drivetrain == null) drivetrain = new Drivetrain();
        
        return drivetrain;
    }
}
