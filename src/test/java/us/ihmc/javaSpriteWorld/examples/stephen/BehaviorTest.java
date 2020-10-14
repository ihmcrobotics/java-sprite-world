package us.ihmc.javaSpriteWorld.examples.stephen;

import org.junit.jupiter.api.Assertions;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

import java.util.Random;

import static us.ihmc.javaSpriteWorld.examples.stephen.BehaviorUtils.bodyFrameToWorldFrame;
import static us.ihmc.javaSpriteWorld.examples.stephen.BehaviorUtils.worldFrameToBodyFrame;

public class BehaviorTest
{
   static void testBodyWorldTransforms()
   {
      StephenRobotBehavior behavior = new StephenRobotBehavior();
      Random random = new Random(38932);

      for (int i = 0; i < 100; i++)
      {
         behavior.heading = random.nextDouble() * Math.PI;
         behavior.xyPosition.setX(random.nextDouble() * 5.0);
         behavior.xyPosition.setY(random.nextDouble() * 5.0);

         // test point conversion

         Point2D bodyFramePointInput = new Point2D(random.nextDouble() * 3.0, random.nextDouble() * 3.0);
         Point2D worldFramePointResult = new Point2D();
         Point2D bodyFramePointOutput = new Point2D();

         bodyFrameToWorldFrame(bodyFramePointInput, worldFramePointResult, behavior.heading, behavior.xyPosition);
         worldFrameToBodyFrame(worldFramePointResult, bodyFramePointOutput, behavior.heading, behavior.xyPosition);

         Assertions.assertTrue(bodyFramePointInput.epsilonEquals(bodyFramePointOutput, 1e-12));

         // test vector conversion

         Vector2D bodyFrameVectortInput = new Vector2D(random.nextDouble() * 3.0, random.nextDouble() * 3.0);
         Vector2D worldFrameVectorResult = new Vector2D();
         Vector2D bodyFrameVectorOutput = new Vector2D();

         bodyFrameToWorldFrame(bodyFrameVectortInput, worldFrameVectorResult, behavior.heading);
         worldFrameToBodyFrame(worldFrameVectorResult, bodyFrameVectorOutput, behavior.heading);

         Assertions.assertTrue(bodyFrameVectortInput.epsilonEquals(bodyFrameVectorOutput, 1e-12));

      }
   }

   public static void main(String[] args)
   {
      testBodyWorldTransforms();
   }
}
