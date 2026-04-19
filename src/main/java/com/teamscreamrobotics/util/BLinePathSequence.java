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
    private final ArrayList<Path> list = new ArrayList<>();
    private final String[] pathNames;
    private final FlipType flipType;
    private int index = 0;

    /**
     * Defines how a path should be transformed to produce its field-symmetric equivalent.
     *
     * <p>BLine provides two underlying operations:
     * <ul>
     *   <li>{@link Path#flip()} - transforms to the opposite alliance side (blue -> red)</li>
     *   <li>{@link Path#mirror()} - reflects across the field width centerline (y -> fieldSizeY - y)</li>
     * </ul>
     *
     * <ul>
     *   <li>{@link #RotatedField} - Use for rotationally symmetric fields (e.g. Rebuilt 2026).
     *       Calls {@link Path#flip()} only, which rotates 180 degrees around the field center.</li>
     *   <li>{@link #SymmetricField} - Use for bilaterally symmetric fields where blue and red are
     *       mirrored across the length midline (y axis). Calls both {@link Path#flip()} and
     *       {@link Path#mirror()}, transforming both alliance side and field half.</li>
     */
    public enum FlipType {
        RotatedField,
        SymmetricField,
    }

    /**
     * Loads and prepares a sequence of BLine paths for autonomous use.
     *
     * @param builder    the {@link FollowPath.Builder} used to build each path-following command
     * @param flipType   how paths should be mirrored for the red alliance — see {@link FlipType}
     * @param pathNames  one or more BLine path names to load, in order
     * @throws InvalidParameterException if no path names are supplied
     * @throws RuntimeException          if any path fails to load
     */
    public BLinePathSequence(FollowPath.Builder builder, FlipType flipType, String... pathNames){
        this.flipType = flipType;
        this.pathNames = pathNames;

        if(flipType == FlipType.SymmetricField){
            this.builder = builder.withDefaultShouldFlip().withShouldMirror(AllianceFlipUtil.shouldFlip()::getAsBoolean);
        } else {
            this.builder = builder.withDefaultShouldFlip();
        }

        if (pathNames.length == 0) {
            throw new InvalidParameterException("Cannot create path sequence of length 0");
        }

        for (String pathName : pathNames){
            Path path = getPath(pathName)
                .orElseThrow(() -> new RuntimeException("Failed to load path: " + pathName));
            list.add(path);
        }
    }

    private Optional<Path> getPath(String pathName){
        try {
            Path path = new Path(pathName);

            if (AllianceFlipUtil.shouldFlip().getAsBoolean()) {
                if(flipType == FlipType.SymmetricField){
                    path.flip();
                    path.mirror();
                } else {
                    path.flip();
                }
            }

            return Optional.of(path);
        } catch (Exception e){
            DriverStation.reportError("[Auto] Failed to load path: " + pathName, e.getStackTrace());
            return Optional.empty();
        }
    }

    private static boolean isRunningPath = false;

    /** Returns {@code true} while any path command from this class is executing. */
    public static boolean isRunningPath(){
        return isRunningPath;
    }

    private Command getPathCommand(Path path){
        return builder.build(path)
            .beforeStarting(() -> isRunningPath = true)
            .finallyDo(() -> isRunningPath = false);
    }

    private Pose2d getPathStartingPose(Path path){
        return path.getStartPose();
    }

    /** Returns the starting pose of the first path — use this to reset odometry before auto. */
    public Pose2d getStartingPose(){
        return getPathStartingPose(list.get(0));
    }

    /** Returns a command that follows the first path in the sequence. */
    public Command getStart(){
        return getPathCommand(list.get(0));
    }

    /** Returns a command that follows the last path in the sequence. */
    public Command getEnd(){
        return getPathCommand(list.get(list.size() - 1));
    }

    /** Returns the total number of paths in this sequence. */
    public int getSize(){
        return list.size();
    }

    /**
     * Advances the internal cursor and returns a command for the next path.
     * Starts at index 1 on first call (use {@link #getStart()} for index 0).
     * Returns a no-op {@link InstantCommand} and logs a DS warning if already at the last path.
     */
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

    /** Returns a {@link SequentialCommandGroup} that runs all paths in order. */
    public Command getAll(){
        Command[] commands = new Command[list.size()];
        for (int i = 0; i < list.size(); i++){
            commands[i] = getPathCommand(list.get(i));
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

    /**
     * Returns the raw {@link Path} at {@code index}, or {@link Optional#empty()} if out of bounds.
     * Use this when you need direct path access rather than a follow command.
     *
     * @param index zero-based path index
     */
    public Optional<Path> getPath(int index){
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

    /**
     * Appends additional paths to the end of this sequence.
     * Paths that fail to load are skipped with a DS error (no exception thrown).
     *
     * @param pathNames one or more BLine path names to append
     * @return {@code this}, for chaining
     */
    public BLinePathSequence withAdditional(String... pathNames){
        for (String pathName : pathNames){
            getPath(pathName).ifPresent(list::add);
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
        BLinePathSequence mirrored = new BLinePathSequence(builder, flipType, pathNames);
        for (Path path : mirrored.list) {
            path.mirror();
        }
        return mirrored;
    }
}