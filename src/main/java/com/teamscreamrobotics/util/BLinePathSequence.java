package com.teamscreamrobotics.util;

import java.security.InvalidParameterException;
import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

public class BLinePathSequence {
    private final FollowPath.Builder builder;
    private final ArrayList<Path> list = new ArrayList<Path>();
    private final String[] pathNames;
    private final FlipType flipType;
    private int index = 0;

    public enum FlipType{
        RotatedField,
        MirroredField;
    }
    
    public BLinePathSequence(FollowPath.Builder builder, FlipType flipType, String... pathNames){
        this.flipType = flipType;
        this.pathNames = pathNames;
        if(flipType == FlipType.RotatedField){
            this.builder = builder.withDefaultShouldFlip();
        } else {
            this.builder = builder.withShouldMirror(() -> true);
        }
        if(pathNames.length == 0){
            throw new InvalidParameterException("Cannot create path sequence of length 0");
        } else {
            for(String pathName : pathNames){
                Path path = getPath(pathName)
    .orElseThrow(() -> new RuntimeException("Failed to load path: " + pathName));
list.add(path);
            }
        }
    }

    private Optional<Path> getPath(String pathName){
        try {
            Path path = new Path(pathName);
            if(AllianceFlipUtil.shouldFlip().getAsBoolean()){
                if(flipType == FlipType.RotatedField){
                    path.flip();
                } else {
                    path.mirror();
                }
            }
            return Optional.of(path);
        } catch (Exception e){
            e.printStackTrace();
            return Optional.empty();
        }
    }

    public static boolean isRunningPath = false;

    public static boolean isRunningPath(){
        return isRunningPath;
    }

    private Command getPathCommand(Path path){
        return builder.build(path).beforeStarting(() -> isRunningPath = true).finallyDo(() -> isRunningPath = false);
    }

    private Pose2d getPathStartingPose(Path path){
        return path.getStartPose();
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

    public int getSize(){
        return list.size();
    }

    public Command getNext(){
        try {
            if (index + 1 >= list.size()) {
    DriverStation.reportWarning("[Auto] No additional paths. Last supplied path: " + pathNames[pathNames.length-1], true);
    return new InstantCommand();
}
index++;
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
            DriverStation.reportWarning("[Auto] No path at specified index " + index + ". Last supplied path: " + pathNames[pathNames.length - 1], true);
            return new InstantCommand();
        }
    }

    public Optional<Path> getPath(int index){
        try {
            return Optional.of(list.get(index));
        } catch (IndexOutOfBoundsException e) {
            DriverStation.reportWarning("[Auto] No path at specified index " + index + ". Last supplied path: " + pathNames[pathNames.length - 1], null);
            return Optional.empty();
        }
    }

    public BLinePathSequence withAdditional(String... pathNames){
        for(String pathName : pathNames){
            list.add(getPath(pathName).get());
        }
        return this;
    }
}
