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
import frc.robot.lib.BLine.FlippingUtil.FieldSymmetry;

public class BLinePathSequence {
    private final FollowPath.Builder builder;
    private final ArrayList<PathEntry> entries = new ArrayList<>();
    private final String[] pathNames;
    private final FieldSymmetry fieldSymmetry;
    private int index = -1;
    private String name;

    private static class PathEntry {
        final String name;
        final boolean extraMirror;

        PathEntry(String name, boolean extraMirror) {
            this.name = name;
            this.extraMirror = extraMirror;
        }
    }

    /**
     * Loads and prepares a sequence of BLine paths for autonomous use.
     * Alliance flipping is deferred to retrieval time so the correct alliance
     * is used even if the object is constructed before the DS reports alliance.
     *
     * @param builder    the {@link FollowPath.Builder} used to build each path-following command
     * @param flipType   how paths should be mirrored for the red alliance — see {@link FlipType}
     * @param pathNames  one or more BLine path names to load, in order
     * @throws InvalidParameterException if no path names are supplied
     */
    public BLinePathSequence(FollowPath.Builder builder, FieldSymmetry fieldSymmetry, String... pathNames){
        this.builder = builder;
        this.fieldSymmetry = fieldSymmetry;
        this.pathNames = pathNames;

        if (pathNames.length == 0) {
            throw new InvalidParameterException("Cannot create path sequence of length 0");
        }

        for (String pathName : pathNames){
            entries.add(new PathEntry(pathName, false));
        }
    }

    private BLinePathSequence(BLinePathSequence source, boolean applyMirror) {
        this.builder = source.builder;
        this.fieldSymmetry = source.fieldSymmetry;
        this.pathNames = source.pathNames;
        this.name = source.name;

        for (PathEntry entry : source.entries) {
            this.entries.add(new PathEntry(entry.name, entry.extraMirror != applyMirror));
        }
    }

    private Optional<Path> loadPath(PathEntry entry){
        try {
            Path path = new Path(entry.name);

            if(AllianceFlipUtil.shouldFlip().getAsBoolean()){
                path.flip();
            }

            if (entry.extraMirror) {
                path.mirror();
            }

            return Optional.of(path);
        } catch (Exception e){
            DriverStation.reportError("[Auto] Failed to load path: " + entry.name, e.getStackTrace());
            return Optional.empty();
        }
    }

    private static boolean isRunningPath = false;

    /** Returns {@code true} while any path command from this class is executing. */
    public static boolean isRunningPath(){
        return isRunningPath;
    }

    private Command getPathCommand(PathEntry entry){
        Optional<Path> path = loadPath(entry);
        if (path.isEmpty()) {
            return new InstantCommand();
        }
        return builder.build(path.get())
            .beforeStarting(() -> isRunningPath = true)
            .finallyDo(() -> isRunningPath = false);
    }

    /** Returns the starting pose of the first path — use this to reset odometry before auto. */
    public Pose2d getStartingPose(){
        return loadPath(entries.get(0))
            .orElseThrow(() -> new RuntimeException("Failed to load path: " + entries.get(0).name))
            .getStartPose();
    }

    /** Returns a command that follows the first path in the sequence. */
    public Command getStart(){
        if(index == -1){
            index = 0;
        }
        return getPathCommand(entries.get(0));
    }

    /** Returns a command that follows the last path in the sequence. */
    public Command getEnd(){
        return getPathCommand(entries.get(entries.size() - 1));
    }

    /** Returns the total number of paths in this sequence. */
    public int getSize(){
        return entries.size();
    }

    /**
     * Advances the internal cursor and returns a command for the next path.
     * Starts at index 1 on first call (use {@link #getStart()} for index 0).
     * Returns a no-op {@link InstantCommand} and logs a DS warning if already at the last path.
     */
    public Command getNext(){
        if (index + 1 >= entries.size()) {
            DriverStation.reportWarning(
                "[Auto] No additional paths. Last supplied path: " +
                pathNames[pathNames.length - 1],
                true
            );
            return new InstantCommand();
        }

        index++;
        return getPathCommand(entries.get(index));
    }

    /** Returns a {@link SequentialCommandGroup} that runs all paths in order. */
    public Command getAll(){
        Command[] commands = new Command[entries.size()];
        for (int i = 0; i < entries.size(); i++){
            commands[i] = getPathCommand(entries.get(i));
        }
        return new SequentialCommandGroup(commands);
    }

    /**
     * Returns a command for the path at {@code index}.
     * Returns a no-op {@link InstantCommand} and logs a DS warning if out of bounds.
     *
     * @param index zero-based path index
     */
    public Command getIndex(int index){
        if (index < 0 || index >= entries.size()) {
            DriverStation.reportWarning(
                "[Auto] No path at specified index " + index +
                ". Last supplied path: " + pathNames[pathNames.length - 1],
                true
            );
            return new InstantCommand();
        }
        return getPathCommand(entries.get(index));
    }

    /**
     * Returns the raw {@link Path} at {@code index}, or {@link Optional#empty()} if out of bounds.
     * The path is flipped according to the current alliance at call time.
     * Use this when you need direct path access rather than a follow command.
     *
     * @param index zero-based path index
     */
    public Optional<Path> getPath(int index){
        if (index < 0 || index >= entries.size()) {
            DriverStation.reportWarning(
                "[Auto] No path at specified index " + index +
                ". Last supplied path: " + pathNames[pathNames.length - 1],
                true
            );
            return Optional.empty();
        }
        return loadPath(entries.get(index));
    }

    /**
     * Appends additional paths to the end of this sequence.
     *
     * @param pathNames one or more BLine path names to append
     * @return {@code this}, for chaining
     */
    public BLinePathSequence withAdditional(String... pathNames){
        for (String pathName : pathNames){
            entries.add(new PathEntry(pathName, false));
        }
        return this;
    }

    /**
     * Returns a new {@link BLinePathSequence} with all paths mirrored across the field
     * width centerline ({@code y -> fieldSizeY - y}), regardless of alliance.
     * Use this when reusing a path designed for one half of the field on the other
     * (e.g. top wall path -> bottom wall path).
     *
     * @return a new {@link BLinePathSequence} with mirrored paths
     */
    public BLinePathSequence mirror() {
        return new BLinePathSequence(this, true);
    }

    /**
     * Sets the name of this path sequence.
     *
     * @param name the name to assign to this sequence
     * @return {@code this}, for chaining
     */
    public BLinePathSequence withName(String name) {
        this.name = name;
        return this;
    }

    /**
     * Returns the name of this path sequence.
     *
     * @return the sequence name, or {@code null} if no name has been set
     */
    public String getName() {
        return name;
    }
}