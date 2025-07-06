package org.firstinspires.ftc.teamcode.vision;

import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes.ColorResult;

import org.firstinspires.ftc.teamcode.utils.AngleUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector2;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Predicate;

@Config
public class LLBlockDetectionPostProcessor {
    public enum BlockColor {
        YELLOW(0),
        RED(1),
        BLUE(2);

        private final int pipelineIndex;

        BlockColor(int pipelineIndex) {
            this.pipelineIndex = pipelineIndex;
        }
    }

    public class Block {
        private Pose2d pose;
        private boolean justUpdated;
        private final BlockColor color;
        private double width;
        private double height;
        private double area;

        protected Block(double x, double y, double heading, double width, double height, BlockColor blockColor) {
            pose = new Pose2d(x, y, heading);
            this.width = width;
            this.height = height;
            area = width * height;
            color = blockColor;
            justUpdated = true;
        }

        protected Block(Pose2d p, double width, double height, BlockColor blockColor) {
            this(p.x, p.y, p.heading, width, height, blockColor);
        }

        public Pose2d getPose() {
            return pose;
        }

        public double getArea() {
            return area;
        }

        // We got new camera values
        protected void updateNewValues(double x, double y, double heading, double width, double height) {
            pose = new Pose2d(x, y, heading);
            this.width = width;
            this.height = height;
            area = width * height;
            justUpdated = true;
        }

        protected void updateNewValues(Pose2d p, double width, double height) {
            this.updateNewValues(p.x, p.y, p.heading, width, height);
        }

        public void update() {
            if (!justUpdated)
                pose = getExpectedPosition();
            else
                justUpdated = false;
        }

        public double getX() {
            return pose.x;
        }

        public double getY() {
            return pose.y;
        }

        public double getHeading() {
            return pose.heading;
        }

        public Pose2d getExpectedPosition () {
            // Get expected based on robot velocities
            Pose2d p = robot.sensors.getOdometryPosition();
            Pose2d pDelta = new Pose2d( // Pose delta
                p.x - lastRobotPosition.x,
                p.y - lastRobotPosition.y,
                p.heading - lastRobotPosition.heading
            );
            Pose2d pDeltaRelative = new Pose2d(
                pDelta.x * Math.cos(p.heading) + pDelta.y * Math.sin(p.heading),
                -pDelta.x * Math.sin(p.heading) + pDelta.y * Math.cos(p.heading),
                pDelta.heading
            );
            Pose2d bRelative = new Pose2d(
                pose.x * Math.cos(pDelta.heading) + pose.y * Math.sin(pDelta.heading),
                -pose.x * Math.sin(pDelta.heading) + pose.y * Math.cos(pDelta.heading),
                pose.heading
            );
            return new Pose2d(
                bRelative.x - pDeltaRelative.x + lastOffset.x - offset.x,
                bRelative.y - pDeltaRelative.y + lastOffset.y - offset.y,
                pose.heading + pDelta.heading
            );
        }

        public Pose2d getGlobalPose() {
            Pose2d p = robot.sensors.getOdometryPosition();
            return new Pose2d(
                (pose.x) * Math.cos(-p.heading) + (pose.y) * Math.sin(-p.heading) + p.x,
                -(pose.x) * Math.sin(-p.heading) + (pose.y) * Math.cos(-p.heading) + p.y,
                pose.heading - p.heading
            );
        }
    }

    private final LinkedList<Block> blocks;
    private final Limelight3A ll;
    private BlockColor blockColor;
    private Pose2d lastRobotPosition;
    private final Robot robot;
    private boolean firstOffset = true;
    private boolean detecting = false;
    private Vector2 offset = new Vector2(0, 0);
    private Vector2 lastOffset = new Vector2(0, 0);
    public static int pollRate = 100;
    private double orientation = 0;
    private double lastLoop = System.currentTimeMillis();

    public LLBlockDetectionPostProcessor(Robot robot) {
        ll = robot.hardwareMap.get(Limelight3A.class, "limelight");
        blockColor = BlockColor.YELLOW;
        ll.setPollRateHz(pollRate);
        blocks = new LinkedList<>();
        lastRobotPosition = null;
        this.robot = robot;
    }

    public void start() {
        ll.pipelineSwitch(blockColor.pipelineIndex);
        ll.start();
    }

    public void stop() { ll.stop(); }

    public void setBlockColor(BlockColor block) {
        this.blockColor = block;
        ll.pipelineSwitch(block.pipelineIndex);
    }

    private boolean connection = true;
    public boolean isConnected() { return connection; }

    /**
     * <b>IMPORTANT</b>: Update the localizer before<br>
     * If you didn't do this you're stupid btw
     */
    public void update() {
        TelemetryUtil.packet.put("LL : offsetX", offset.x);
        TelemetryUtil.packet.put("LL : offsetY", offset.y);
        TelemetryUtil.packet.put("LL : orientation", orientation);
        Canvas canvas = TelemetryUtil.packet.fieldOverlay();
        for (Block b : blocks) {
            Pose2d rel = b.getPose();
            Pose2d cur = robot.sensors.getOdometryPosition();
            Pose2d abs = new Pose2d(
                cur.x + rel.x * Math.cos(cur.heading) - rel.y * Math.sin(cur.heading),
                cur.y + rel.x * Math.sin(cur.heading) + rel.y * Math.cos(cur.heading),
                rel.heading - cur.heading
            );
/*
            canvas.setStroke("#808080");
            canvas.strokeCircle(b.getX(), b.getY(), 2);
            canvas.strokeLine(b.getX(), b.getY(), b.getX() + 5 * Math.sin(b.getHeading()), b.getY() + 3 * Math.cos(b.getHeading()));

            canvas.setStroke("#c04040");
            canvas.strokeCircle(abs.x, abs.y, 1.5);
            canvas.strokeLine(abs.x, abs.y, abs.x + 5 * Math.sin(abs.heading), abs.y + 3 * Math.cos(abs.heading));
*/
            canvas.setStroke("#4040c0");
            canvas.strokeCircle(b.getGlobalPose().x, b.getGlobalPose().y, 2);
            canvas.strokeLine(b.getGlobalPose().x, b.getGlobalPose().y, b.getGlobalPose().x + 3 * Math.sin(b.getGlobalPose().heading), b.getGlobalPose().y + 5 * Math.cos(b.getGlobalPose().heading));
        }

        TelemetryUtil.packet.put("LL : connected", ll.isConnected());
        TelemetryUtil.packet.put("LL : detecting", detecting);
        if (!ll.isConnected()) {
            Log.e("ERROR BIG", "Limelight Broke");
            connection = false;
        }

        long loopStart = System.currentTimeMillis();

        if (detecting) {
            LLResult result = ll.getLatestResult();
            // TelemetryUtil.packet.put("LL result", "> " + result); Stop. Don't do this.

            // Hn we are cooked
            if (result == null || result.getStaleness() >= 100)
                return;

            // We have valid results, update the list of blocks properly
            List<ColorResult> crs = result.getColorResults();

            for (ColorResult cr : crs) {
                if (cr.getTargetCorners().size() != 4) // Ok. If a block literally has a corner count of not 4 it is obvi cooked
                    continue;

                Pose2d newPose = new Pose2d(getInchesX(cr.getTargetYDegrees()) + offset.x, getInchesY(-cr.getTargetXDegrees()) + offset.y, 0);

                List<List<Double>> corners = cr.getTargetCorners();
                // This format sucks balls im changing it
                Vector2[] vcorners = new Vector2[corners.size()];
                canvas.setFill("#808080");
                for (int i = 0; i < corners.size(); i++) {
                    vcorners[i] = new Vector2(corners.get(i).get(0), corners.get(i).get(1));
                    canvas.fillCircle(vcorners[i].x / 10, vcorners[i].y / 10, 1);
                }

                int l0 = 0; // nyeh pointer !
                int l1 = 0;
                double largestDistance = Vector2.distance(vcorners[l0], vcorners[l1]);
                for (int i = 0; i < vcorners.length; i++) {
                    for (int j = i + 1; j < vcorners.length; j++) { // No gaf
                        double d = Vector2.distance(vcorners[i], vcorners[j]);
                        if (d >= largestDistance) {
                            l0 = i;
                            l1 = j;
                            largestDistance = d;
                        }
                    }
                }
                // Now find out which one is the closest for each
                int cl0 = -1;
                int cl1 = -1;
                double distcl0 = Double.MAX_VALUE;
                double distcl1 = Double.MAX_VALUE;
                for (int i = 0; i < vcorners.length; i++) {
                    double dl0 = Vector2.distance(vcorners[l0], vcorners[i]);
                    double dl1 = Vector2.distance(vcorners[l1], vcorners[i]);
                    if (i != l0 && (cl0 == -1 || dl0 < distcl0)) {
                        cl0 = i;
                        distcl0 = dl0;
                    }
                    if (i != l1 && (cl1 == -1 || dl1 < distcl1)) {
                        cl1 = i;
                        distcl1 = dl1;
                    }
                }

                double width = Math.sqrt(Math.pow(vcorners[cl0].x - vcorners[l0].x, 2) + Math.pow(vcorners[cl0].y - vcorners[l0].y, 2));
                double height = Math.sqrt(Math.pow(largestDistance, 2) - width * width);
                double area = width * height;

                double h1 = AngleUtil.mirroredClipAngleTolerence(Math.atan2(vcorners[l0].y - vcorners[cl0].y, vcorners[l0].x - vcorners[cl0].x), Math.toRadians(20));
                double h2 = AngleUtil.mirroredClipAngleTolerence(Math.atan2(vcorners[cl1].y - vcorners[l1].y, vcorners[cl1].x - vcorners[l1].x), Math.toRadians(20));
                newPose.heading = AngleUtil.mirroredClipAngleTolerence((h1 + h2) / 2 - orientation, Math.toRadians(20));

                // I'm going to discard velo based weighted avg for now

                // Now we need to find the block that is closest
                Block b = null;
                double dist = 1.5;
                for (Block block : blocks) {
                    double d = block.getExpectedPosition().getDistanceFromPoint(newPose);
                    if (d <= dist) {
                        b = block;
                        dist = d;
                    }
                }

                TelemetryUtil.packet.put("LL : BlockArea", area);
                TelemetryUtil.packet.put("LL : Robot Velocity", robot.sensors.getVelocity().toVec3().getMag());

                if (area >= 10500 && area <= 12500 && robot.sensors.getVelocity().toVec3().getMag() <= 1.5) {
                    if (b == null) {
                        b = new Block(newPose, width, height, blockColor);
                        blocks.add(b);
                    } else
                        b.updateNewValues(newPose, width, height);
                }
            }

            //canvas.setFill("#ffffff");
            //canvas.fillText("Vision is on", 24, 24, "8px Arial", 0);
        }

        for (Block b : blocks) b.update();
            /*if ((area <= 10000 || area >= 14000) && detections >= 1) { // Pre worlds jank - Eric
                blockPos = expectedNewBlockPose.clone();
                newDetections = detections;
            }*/
        TelemetryUtil.packet.put("loopDelta", (loopStart - lastLoop) * 1.0e-3);

        // This is fine because detecting turning on would update this value properly
        lastRobotPosition = robot.sensors.getOdometryPosition().clone();

        lastLoop = loopStart;
    }

    /**
     * Starts detection and uses robot position deltas
     */
    public void startDetection() {
        lastRobotPosition = robot.sensors.getOdometryPosition().clone();
        orientation = 0;
        detecting = true;
        firstOffset = true;
        lastLoop = System.currentTimeMillis();
    }

    /**
     * Stops detection
     */
    public void stopDetection() { detecting = false; }


    public void setOffset(Vector2 v) {
        if (firstOffset) {
            lastOffset = v;
            firstOffset = false;
        } else {
            lastOffset = offset;
        }
        offset = v;
    }

    private double getInchesX(double x) { return 0.0162 + 0.114 * x + 1.76e-4 * x * x; }

    private double getInchesY(double y) { return 0.203 + 0.123 * y + -6.55e-5 * y * y; }

    public void setNewOrientation(double ang) { orientation = ang; }

    public boolean isDetecting() { return detecting; }

    public BlockColor getBlockColor() { return blockColor; }

    public void removeBlock(Block b) { blocks.remove(b); }

    public static LinkedList<Block> filterBlocks(LinkedList<Block> blocks, Predicate<Block> filter) {
        LinkedList<Block> output = new LinkedList<>();

        for (Block b : blocks) {
            if (filter.test(b))
                output.add(b);
        }

        return output;
    }

    public Block getClosestValidBlock() { return getClosestValidBlock(offset, blocks); }

    public static Block getClosestValidBlock(Vector2 offset, LinkedList<Block> blocks) {
        if (blocks.size() <= 0) return null;

        Block closest = blocks.get(0);
        double closestDist = closest.getPose().getDistanceFromPoint(new Pose2d(offset.x, offset.y, 0));
        for (Block b : blocks) {
            double d = b.getPose().getDistanceFromPoint(new Pose2d(offset.x, offset.y, 0));

            if (d <= closestDist) {
                closest = b;
                closestDist = d;
            }
        }

        return closest;
    }

    public LinkedList<Block> getBlocks() { return blocks; }

    public Vector2 getOffset() { return offset; }

    public void hardwareStop() {
        ll.stop();
        ll.close();
        ll.shutdown();
    }
}