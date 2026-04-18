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

public class PPPathSequence {
    private final ArrayList<PathPlannerPath> list = new ArrayList<>();
    private final String[] pathNames;
    private int index = 0;

    public PPPathSequence(String... pathNames){
        this.pathNames = pathNames;

        if (pathNames.length == 0) {
            loadFallbackPath();
        } else {
            for (String pathName : pathNames) {
                PathPlannerPath path = getPath(pathName)
                    .orElseThrow(() -> new RuntimeException("Failed to load path: " + pathName));
                list.add(path);
            }
        }
    }

    private void loadFallbackPath() {
        try {
            list.add(PathPlannerPath.fromPathFile("DoNothing"));
        } catch (FileVersionException | IOException | ParseException e) {
            DriverStation.reportError("[Auto] Failed to load fallback path DoNothing", e.getStackTrace());
        }
    }

    private static Optional<PathPlannerPath> getPath(String pathName){
        try {
            return Optional.of(PathPlannerPath.fromPathFile(pathName));
        } catch (Exception e){
            DriverStation.reportError("[Auto] Failed to load path: " + pathName, e.getStackTrace());
            return Optional.empty();
        }
    }

    private static boolean isRunningPath = false;

    private Command getPathCommand(PathPlannerPath path){
        return AutoBuilder.followPath(path)
            .beforeStarting(() -> isRunningPath = true)
            .finallyDo(() -> isRunningPath = false);
    }

    private Pose2d getPathStartingPose(PathPlannerPath path){
        return AllianceFlipUtil.MirroredPose2d(
            path.getStartingHolonomicPose().orElse(new Pose2d())
        );
    }

    public Pose2d getStartingPose(){
        return getPathStartingPose(list.get(0));
    }

    public Command getStart(){
        return getPathCommand(list.get(0));
    }

    public Command getEnd(){
        return getPathCommand(list.get(list.size() - 1));
    }

    public static int getSize(PPPathSequence pathSequence){
        return pathSequence.list.size();
    }

    public Command getNext(){
        if (index + 1 >= list.size()) {
            DriverStation.reportWarning(
                "[Auto] No additional paths. Last supplied path: " +
                pathNames[pathNames.length - 1],
                true
            );
            return new InstantCommand();
        }

        index++;
        return getPathCommand(list.get(index));
    }

    public Command getAll(){
        Command[] commands = new Command[list.size()];
        for (int i = 0; i < list.size(); i++){
            commands[i] = getPathCommand(list.get(i));
        }
        return new SequentialCommandGroup(commands);
    }

    public Command getIndex(int index){
        if (index < 0 || index >= list.size()) {
            DriverStation.reportWarning(
                "[Auto] No path at specified index " + index +
                ". Last supplied path: " + pathNames[pathNames.length - 1],
                true
            );
            return new InstantCommand();
        }
        return getPathCommand(list.get(index));
    }

    public Optional<PathPlannerPath> getPath(int index){
        if (index < 0 || index >= list.size()) {
            DriverStation.reportWarning(
                "[Auto] No path at specified index " + index +
                ". Last supplied path: " + pathNames[pathNames.length - 1],
                true
            );
            return Optional.empty();
        }
        return Optional.of(list.get(index));
    }

    public PPPathSequence withAdditional(String... pathNames){
        for (String pathName : pathNames){
            getPath(pathName).ifPresent(list::add);
        }
        return this;
    }
}