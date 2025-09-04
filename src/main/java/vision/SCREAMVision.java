package vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.RobotBase;
import vision.LimelightVision.Limelight;

public class SCREAMVision {
    
    public static class SCREAMVisionConfiguration{

        public boolean enabled = true;
        public boolean simEnabled = true;
        public boolean forceSimulation = false;

        public boolean rejectHeading = true;

        public boolean simDrawWireframe = false;

        public Limelight[] limelights = null;

        public AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        public double xyStdBaseline = 0.93;
        public double thetaStdBaseline = 12.5;
    }

    protected final SCREAMVisionConfiguration config;
    protected final Limelight[] limelights;

    public SCREAMVision(SCREAMVisionConfiguration config){
        this.config = config;
        this.limelights = config.limelights;

        if(shouldSimulate()){

        }
    }

    public boolean shouldSimulate(){
        return (config.simEnabled && RobotBase.isSimulation()) || config.forceSimulation;
    }

    public void initializeSimulation(){

    }

}
