package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import java.util.*;

public class StanleyAutoP1 {


        public ArrayList<PathChain> paths;
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;

        public StanleyAutoP1(Follower follower) {

            paths = new ArrayList<>();






            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 8.000), new Pose(56.000, 36.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 36.000), new Pose(14.049, 35.733))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(180))


                    //.setTangentHeadingInterpolation()
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(14.049, 35.733), new Pose(56.042, 7.483))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(180))

                    //.setTangentHeadingInterpolation()
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.042, 7.483), new Pose(45.200, 60.013))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(180))

                    //.setTangentHeadingInterpolation()
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(45.200, 60.013), new Pose(12.522, 59.860))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(180))

                    //.setTangentHeadingInterpolation()
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(12.522, 59.860), new Pose(56.501, 7.635))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(180))

                    //.setTangentHeadingInterpolation()
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.501, 7.635), new Pose(40.467, 83.835))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(180))

                    //.setTangentHeadingInterpolation()
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(40.467, 83.835), new Pose(11.606, 83.529))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(180))

                    //.setTangentHeadingInterpolation()
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(11.606, 83.529), new Pose(56.959, 7.635))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(180))

                    //.setTangentHeadingInterpolation()
                    .build();


            paths.add(Path1);
            paths.add(Path2);
            paths.add(Path3);
            paths.add(Path4);
            paths.add(Path5);
            paths.add(Path6);
            paths.add(Path7);
            paths.add(Path8);
            paths.add(Path9);
        }

}
