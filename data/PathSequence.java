package com.SCREAMLib.data;

import java.util.ArrayList;
import java.util.List;

import com.SCREAMLib.util.AllianceFlipUtil;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PathSequence {
    ArrayList<PathPlannerPath> list = new ArrayList<PathPlannerPath>();
    String[] pathNames;
    int index = 0;
    
    public PathSequence(String... pathNames){
        this.pathNames = pathNames;
        if(pathNames.length == 0){
            list.add(new PathPlannerPath(List.of(), new PathConstraints(0, 0, 0, 0), new GoalEndState(0, new Rotation2d())));
        } else {
            for(String pathName : pathNames){
                list.add(getPath(pathName));
            }
        }
    }

    private static PathPlannerPath getPath(String pathName){
        return PathPlannerPath.fromPathFile(pathName);
    }

    private static Command getPathCommand(PathPlannerPath path){
        return AutoBuilder.followPath(path);
    }

    private static Pose2d getPathStartingPose(PathPlannerPath path){
        return AllianceFlipUtil.MirroredPose2d(path.getPreviewStartingHolonomicPose());
    }

    public Pose2d getStartingPose(){
        return getPathStartingPose(list.get(0));
    }

    public int getSize(){
        return list.size();
    }

    public Command getAll(){
        Command[] commands = new Command[list.size()];
        for(int i = 0; i < list.size(); i++){
            commands[i] = getPathCommand(list.get(i));
        }
        return new SequentialCommandGroup(commands);
    }

    public Command getIndex(int index){
        try {
            return getPathCommand(list.get(index));
        } catch (IndexOutOfBoundsException e) {
            DriverStation.reportWarning("[Auto] No path at specified index " + index + ". Last supplied path: " + pathNames[pathNames.length], null);
            return new InstantCommand();
        }
    }

    public PathPlannerPath getPath(int index){
        try {
            return list.get(index);
        } catch (IndexOutOfBoundsException e) {
            DriverStation.reportWarning("[Auto] No path at specified index " + index + ". Last supplied path: " + pathNames[pathNames.length], null);
            return PathPlannerPath.fromPathFile("DoNothing");
        }
    }

    public PathSequence withAdditional(String... pathNames){
        for(String pathName : pathNames){
            list.add(getPath(pathName));
        }
        return this;
    }
}
