package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree;

import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeAction;
import us.ihmc.javaSpriteWorld.examples.behaviorTree.BehaviorTreeNodeStatus;

import static us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree.RobotBehaviorTools.*;

public class AvoidWallsBehaviorNode implements BehaviorTreeAction
{
   private static double WALL_DISTANCE_ACTIVATION_THRESHOLD = 1.0;

   private final RobotBehaviorSensors sensors;
   private double accelerationGain = 1.0;
   private double turnRateGain = 5.0;
   private double turnRateDamping = -0.5;
   private RobotBehaviorActuators actuators;
   private RobotBehaviorEnvironment environment;
   private double dt = 0.01;
   private double lastVelocity = 0.0;
   private double boundaryStrength = 3.0;
   private double boundaryGraduation = 2.5;
   private double[] action = new double[2];
   private double utility;

   public AvoidWallsBehaviorNode(RobotBehaviorSensors sensors,
                                 RobotBehaviorActuators actuators,
                                 RobotBehaviorEnvironment environment)
   {
      this.sensors = sensors;
      this.actuators = actuators;
      this.environment = environment;
   }

   @Override
   public double evaluateUtility()
   {
      boolean enable = sensors.getClosestWallDistance() < WALL_DISTANCE_ACTIVATION_THRESHOLD;
      utility = enable ? 1.0 : 0.0;
      return utility;
   }

   @Override
   public BehaviorTreeNodeStatus tick()
   {
      Vector2D boundaryRepulsion = new Vector2D();
      double closestWallDistance = Double.POSITIVE_INFINITY;
      for (LineSegment2D wall : environment.getWalls())
      {
         Point2D closestPointOnWall = EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(sensors.getGlobalPosition(),
                                                                                              wall.getFirstEndpoint(),
                                                                                              wall.getSecondEndpoint());
         double wallDistance = sensors.getGlobalPosition().distance(closestPointOnWall);
         if (wallDistance < closestWallDistance)
            closestWallDistance = wallDistance;

         if (wallDistance < WALL_DISTANCE_ACTIVATION_THRESHOLD)
         {
            boundaryRepulsion.add(fieldVector(closestPointOnWall,
                                              sensors.getGlobalPosition(),
                                              distance -> boundaryStrength / Math.pow(distance, boundaryGraduation)));
         }
      }

      lastVelocity = doAttractionVectorControl(sensors, action, boundaryRepulsion, lastVelocity, dt, accelerationGain, turnRateGain, turnRateDamping);

      actuators.setAcceleration(action[0]);
      actuators.setTurnRate(action[1]);

      return BehaviorTreeNodeStatus.SUCCESS;
   }
}
