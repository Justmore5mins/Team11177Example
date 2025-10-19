package frc.robot.Vision;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{
    public PhotonCamera LocalizationCamera;
    public PhotonCameraSim CamerSim;
    public PhotonPoseEstimator PoseEstimator;
    public StructArrayPublisher<Pose3d> PosePublisher;
    private static Vision instance = null;

    private Vision(){
        LocalizationCamera = new PhotonCamera(Constants.LocalizationCameraName);
        CamerSim = new PhotonCameraSim(LocalizationCamera,SimCameraProperties.LL2_1280_720());
        PoseEstimator = new PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark),
            Constants.strategy,
            new Transform3d(Constants.CameraPlace.toMatrix()) // TODO: 這邊要改成我們的機器人相對於攝影機的轉換矩陣
        );
        PosePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("Vision/Targets", Pose3d.struct).publish();
    }

    public Pose2d getPose(){
        try{
            if(!LocalizationCamera.isConnected()) throw new RuntimeException("bruh"); //沒有連接到相機
            var results = LocalizationCamera.getAllUnreadResults();//取得所有未讀取的結果
            var res = PoseEstimator.update(results.get(results.size() - 1)).orElseThrow();//用最新的結果更新定位器
            List<Pose3d> t = new ArrayList<Pose3d>();//準備一個清單來存放所有目標的位置
            res.targetsUsed.forEach((tar) -> t.add(res.estimatedPose.transformBy(tar.getBestCameraToTarget())));//把每個目標的位置都加進清單
            PosePublisher.accept(t.toArray(Pose3d[]::new));//把目標位置發布到NetworkTables
            return res.estimatedPose.toPose2d();//回傳機器人位置
        }catch(Exception e){//因為太懶所以直接用exception統一打包
            return null;
        }
    }
    @Override
    public void periodic(){//雖然沒有常態性的餵回去pose estimator 不過NT會常態性更新
        getPose();
    }

    public static Vision getInstance() {
        if (instance == null) instance = new Vision();
        
        return instance;
    }
}
