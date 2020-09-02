package us.ihmc.javaSpriteWorld.examples.robotChallenge01;

import java.util.Random;

import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.javaSpriteWorld.Sprite;
import us.ihmc.javaSpriteWorld.SpriteCollisionGroup;
import us.ihmc.javaSpriteWorld.SpriteCollisionListener;
import us.ihmc.javaSpriteWorld.SpriteWorld;

public class CollisionProcessor01 implements SpriteCollisionListener
{
   private final Robot01 robot;
   private final FoodList01 foodList;
   private final Random random;
   private final double xMax, yMax;
   private final SpriteWorld spriteWorld;
   private final SpriteCollisionGroup collisionGroup;

   public CollisionProcessor01(Robot01 robot, FoodList01 foodList, Random random, double xMax, double yMax, SpriteWorld spriteWorld,
                               SpriteCollisionGroup collisionGroup)
   {
      this.robot = robot;
      this.foodList = foodList;
      this.random = random;
      this.xMax = xMax;
      this.yMax = yMax;
      this.spriteWorld = spriteWorld;
      this.collisionGroup = collisionGroup;
   }

   @Override
   public void spritesAreColliding(Sprite spriteOne, Sprite spriteTwo)
   {
      System.out.println(spriteOne.getName() + " colliding with " + spriteTwo.getName());

      if (spriteOne == robot.getSprite())
      {
         processRobotCollision(robot, spriteTwo);
      }

      else if (spriteTwo == robot.getSprite())
      {
         processRobotCollision(robot, spriteOne);
      }

      else
      {
         Food01 foodOne = foodList.findFood(spriteOne);
         Food01 foodTwo = foodList.findFood(spriteTwo);

         if ((foodOne != null) && (foodTwo != null))
         {
            processFoodFoodCollision(foodOne, foodTwo);
         }
      }

   }

   private void processFoodFoodCollision(Food01 foodOne, Food01 foodTwo)
   {
      double xOne = foodOne.getX();
      double yOne = foodOne.getY();
      
      double xTwo = foodTwo.getX();
      double yTwo = foodTwo.getY();
      
      Vector2D v;
      double deltaX = xTwo - xOne;
      double deltaY = yTwo - yOne;
      
      if ((deltaX == 0.0) && (deltaY == 0.0))
      {
         deltaX = random.nextDouble();
         deltaY = random.nextDouble();
      }
      
      double magnitude = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
      deltaX = deltaX / magnitude;
      deltaY = deltaY / magnitude;
      
      double heading = Math.atan2(deltaY, deltaX);
      foodOne.setHeading(heading);
      foodTwo.setHeading(Math.PI + heading);
      foodOne.setVelocity(1.0); //Math.abs(foodOne.getVelocity()));
      foodTwo.setVelocity(1.0); //Math.abs(foodTwo.getVelocity()));
      
   }

   private void processRobotCollision(Robot01 robot, Sprite sprite)
   {
      Food01 food = foodList.findFood(sprite);

      if (food != null)
      {
         processRobotAndFoodCollision(robot, food);
      }

   }

   private void processRobotAndFoodCollision(Robot01 robot, Food01 food)
   {
      robot.eatFood(food);

      foodList.removeFood(food);
      foodList.createSomeFood(random, xMax, yMax, spriteWorld, collisionGroup);
   }

}