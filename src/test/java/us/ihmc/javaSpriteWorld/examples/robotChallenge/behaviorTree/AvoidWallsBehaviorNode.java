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
   private final RobotBehaviorSensors sensors;
   private final RobotBehaviorActuators actuators;
   private double accelerationGain = 1.0;
   private double turnRateGain = 5.0;
   private double turnRateDamping = -0.5;
   private RobotBehaviorEnvironment environment;
   private double dt = 0.01;
   private double lastVelocity = 0.0;
   private double boundaryStrength = 3.0;
   private double boundaryGraduation = 2.5;
   private double wallDistanceActivationThreshold = 1.0;

   public AvoidWallsBehaviorNode(RobotBehaviorSensors sensors, RobotBehaviorActuators actuators, RobotBehaviorEnvironment environment)
   {
      this.sensors = sensors;
      this.actuators = actuators;
      this.environment = environment;
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

         if (wallDistance < wallDistanceActivationThreshold)
         {
            boundaryRepulsion.add(fieldVector(closestPointOnWall, sensors.getGlobalPosition(),
                                              distance -> boundaryStrength / Math.pow(distance, boundaryGraduation)));
         }
      }

      if (closestWallDistance < wallDistanceActivationThreshold)
      {
         lastVelocity = doAttractionVectorControl(sensors, actuators, boundaryRepulsion, lastVelocity, dt, accelerationGain, turnRateGain, turnRateDamping);
         return BehaviorTreeNodeStatus.RUNNING;
      }

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
      this.wallDistanceActivationThreshold = wallDistanceActivationThreshold;
   }
}
