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
   private final BehaviorStatusHolder statusHolder;
   private double accelerationGain = 1.0;
   private double turnRateGain = 5.0;
   private double turnRateDamping = -0.5;
   private RobotBehaviorEnvironment environment;
   private double dt = 0.01;
   private double lastVelocity = 0.0;
   private double boundaryStrength = 3.0;
   private double boundaryGraduation = 2.5;

   public AvoidWallsBehaviorNode(RobotBehaviorSensors sensors, RobotBehaviorEnvironment environment, BehaviorStatusHolder statusHolder)
   {
      this.sensors = sensors;
      this.environment = environment;
      this.statusHolder = statusHolder;
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
            boundaryRepulsion.add(fieldVector(closestPointOnWall, sensors.getGlobalPosition(),
                                              distance -> boundaryStrength / Math.pow(distance, boundaryGraduation)));
         }
      }

      boolean enable = closestWallDistance < WALL_DISTANCE_ACTIVATION_THRESHOLD;

      statusHolder.setWallWeight(enable ? 1.0 : 0.0);
      lastVelocity = doAttractionVectorControl(sensors, statusHolder.getWallAction(), boundaryRepulsion, lastVelocity, dt, accelerationGain, turnRateGain, turnRateDamping);

      return BehaviorTreeNodeStatus.SUCCESS;
   }

   public void setAccelerationGain(double accelerationGain)
   {
      this.accelerationGain = accelerationGain;
   }

   public void setTurnRateGain(double turnRateGain)
   {
      this.turnRateGain = turnRateGain;
   }

   public void setTurnRateDamping(double turnRateDamping)
   {
      this.turnRateDamping = turnRateDamping;
   }

   public void setBoundaryStrength(double boundaryStrength)
   {
      this.boundaryStrength = boundaryStrength;
   }

   public void setBoundaryGraduation(double boundaryGraduation)
   {
      this.boundaryGraduation = boundaryGraduation;
   }

   public void setWallDistanceActivationThreshold(double wallDistanceActivationThreshold)
   {
      this.WALL_DISTANCE_ACTIVATION_THRESHOLD = wallDistanceActivationThreshold;
   }
}
