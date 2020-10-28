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
   private RobotBehaviorEnvironment environment;
   private double dt = 0.01;
   private double lastVelocity = 0.0;

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
      double boundaryStrength = 3.0;
      double boundaryGraduation = 2.5;
      double wallDistanceActivationThreshold = 0.2;
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
         lastVelocity = doAttractionVectorControl(sensors, actuators, boundaryRepulsion, lastVelocity, dt);
         return BehaviorTreeNodeStatus.RUNNING;
      }
      else
      {
         return BehaviorTreeNodeStatus.SUCCESS;
      }
   }
}
