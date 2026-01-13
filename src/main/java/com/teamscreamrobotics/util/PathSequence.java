package com.teamscreamrobotics.util;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
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
            try {
                list.add(PathPlannerPath.fromPathFile("DoNothing"));
            } catch (FileVersionException | IOException | ParseException e) {
                e.printStackTrace();
            }
        } else {
            for(String pathName : pathNames){
                list.add(getPath(pathName).get());
            }
        }
        
    }

    private static Optional<PathPlannerPath> getPath(String pathName){
        try {
            return Optional.of(PathPlannerPath.fromPathFile(pathName));
        } catch (Exception e){
            e.printStackTrace();
            return Optional.empty();
        }
    }

    @SuppressWarnings("unused")
    private static boolean isRunningPath = false;
    private static Command getPathCommand(PathPlannerPath path){
        return AutoBuilder.followPath(path).beforeStarting(() -> isRunningPath = true).finallyDo(() -> isRunningPath = false);
    }

    private static Pose2d getPathStartingPose(PathPlannerPath path){
        return AllianceFlipUtil.MirroredPose2d(path.getStartingHolonomicPose().get());
    }

    public Pose2d getStartingPose(){
        return getPathStartingPose(list.get(0));
    }

    public Command getStart(){
        return getPathCommand(list.get(0));
    }

    public Command getEnd(){
        return getPathCommand(list.get(list.size()-1));
    }

    public static int getSize(PathSequence pathSequence){
        return pathSequence.list.size()-1;
    }

    public Command getNext(){
        try {
            index ++;
            return getPathCommand(list.get(index));
        } catch (IndexOutOfBoundsException e) {
            DriverStation.reportWarning("[Auto] No additional paths. Last supplied path: " + pathNames[pathNames.length-1], true);
            return new InstantCommand();
        }
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

    public Optional<PathPlannerPath> getPath(int index){
        try {
            return Optional.of(list.get(index));
        } catch (IndexOutOfBoundsException e) {
            DriverStation.reportWarning("[Auto] No path at specified index " + index + ". Last supplied path: " + pathNames[pathNames.length], null);
            try {
                return Optional.of(PathPlannerPath.fromPathFile("DoNothing"));
            } catch (FileVersionException | IOException | ParseException e1) {
                e1.printStackTrace();
                Optional.empty();
            }
            return Optional.empty();
        }
    }

    public PathSequence withAdditional(String... pathNames){
        for(String pathName : pathNames){
            list.add(getPath(pathName).get());
        }
        return this;
    }
}
