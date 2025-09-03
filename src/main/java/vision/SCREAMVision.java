package vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class SCREAMVision {
    
    public static class SCREAMVisionConfiguration{

        public boolean enabled = true;
        public boolean simEnabled = true;
        public boolean forceSimulation = false;

        public boolean rejectHeading = true;

        public boolean simDrawWireframe = false;

        public LimelightVision.Limelight[] limelights = null;

        public AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        public double xyStdBaseline = 0.93;
        public double thetaStdBaseline = 12.5;
    }

    public SCREAMVision(SCREAMVisionConfiguration config){
        
    }

}
