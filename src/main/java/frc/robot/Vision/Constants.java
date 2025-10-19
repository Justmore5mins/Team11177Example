package frc.robot.Vision;

import static edu.wpi.first.units.Units.Centimeters;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

// TODO: 這邊也是
public class Constants {
    public static final String LocalizationCameraName = "LocalizationCamera"; //這個名字要跟PhotonVision GUI剛進去的右上角的那個名字要一樣(可以自己改)
    public static final PoseStrategy strategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR; //要用的定位策略
    public static final Pose3d CameraPlace = new Pose3d(Centimeters.of(0), Centimeters.of(0),Centimeters.of(0), Rotation3d.kZero); //相對於機器底部中心的座標
}
