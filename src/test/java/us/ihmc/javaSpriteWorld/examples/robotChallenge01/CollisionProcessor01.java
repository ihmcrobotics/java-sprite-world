package us.ihmc.javaSpriteWorld.examples.robotChallenge01;

import java.util.Random;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.javaSpriteWorld.Sprite;
import us.ihmc.javaSpriteWorld.SpriteCollisionGroup;
import us.ihmc.javaSpriteWorld.SpriteCollisionListener;
import us.ihmc.javaSpriteWorld.SpriteWorld;

public class CollisionProcessor01 implements SpriteCollisionListener
{
   private RobotChallengeRules robotChallengeRules;
   private final RobotChallengeRobot robot;
   private final FoodList01 foodList;
   private final WallList wallList;
   private final PredatorList01 predatorList;
   private final FlagList flagList;
   private final Random random;
   private final double xMax, yMax;
   private final SpriteWorld spriteWorld;
   private final SpriteCollisionGroup collisionGroup;

   public CollisionProcessor01(RobotChallengeRobot robot, FoodList01 foodList, PredatorList01 predatorList,
                               FlagList flagList, WallList wallList, Random random, double xMax, double yMax, SpriteWorld spriteWorld, SpriteCollisionGroup collisionGroup)
   {
      this.robot = robot;
      this.foodList = foodList;
      this.predatorList = predatorList;
      this.flagList = flagList;
      this.wallList = wallList;
      this.random = random;
      this.xMax = xMax;
      this.yMax = yMax;
      this.spriteWorld = spriteWorld;
      this.collisionGroup = collisionGroup;
   }

   @Override
   public void spritesAreColliding(Sprite spriteOne, Sprite spriteTwo)
   {
      //      System.out.println(spriteOne.getName() + " colliding with " + spriteTwo.getName());

      if (spriteOne == robot.getSprite())
      {
         processRobotCollision(robot, spriteTwo);
         return;
      }

      if (spriteTwo == robot.getSprite())
      {
         processRobotCollision(robot, spriteOne);
         return;
      }

      Predator01 predator = predatorList.findPredator(spriteOne);
      if (predator != null)
      {
         processPredatorCollision(predator, spriteTwo);
         return;
      }

      predator = predatorList.findPredator(spriteTwo);
      if (predator != null)
      {
         processPredatorCollision(predator, spriteOne);
         return;
      }

      Food01 food = foodList.findFood(spriteOne);
      if (food != null)
      {
         processFoodCollision(food, spriteTwo);
         return;
      }
      
      food = foodList.findFood(spriteTwo);
      if (food != null)
      {
         processFoodCollision(food, spriteOne);
         return;
      }

      Wall wall = wallList.findWall(spriteOne);
      if (wall != null)
      {
         processWallCollision(wall, spriteTwo);
         return;
      }
      
      wall = wallList.findWall(spriteTwo);
      if (wall != null)
      {
         processWallCollision(wall, spriteOne);
      }

   }

   private void processWallCollision(Wall wall, Sprite sprite)
   {
      Flag flag = flagList.findFlag(sprite);
      if (flag != null)
      {
//         System.out.println("Scootching flag " + flag.getId());
         double x = flag.getX();
         double y = flag.getY();

         if (x < 1.0) 
            x = x + 0.1;
         else
            x = x - 0.1;
         if (y < 1.0)
            y = y + 0.1;
         else
            y = y - 0.1;
         
         flag.setLocation(new Point2D(x, y));
      }
      
   }

   private void processFoodCollision(Food01 food, Sprite sprite)
   {
      Food01 foodTwo = foodList.findFood(sprite);

      if ((foodTwo != null))
      {
         processFoodAndFoodCollision(food, foodTwo);
      }
    
      Wall wall = wallList.findWall(sprite);
      if (wall != null)
      {
         processFoodAndWallCollision(food, wall);
      }
   }

   private void processFoodAndWallCollision(Food01 food, Wall wall)
   {
      // Go through for now.
   }

   private void processFoodAndFoodCollision(Food01 foodOne, Food01 foodTwo)
   {
      double xOne = foodOne.getX();
      double yOne = foodOne.getY();

      double xTwo = foodTwo.getX();
      double yTwo = foodTwo.getY();

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
      foodOne.setHeading(Math.PI + heading);
      foodTwo.setHeading(heading);
      foodOne.setSpeed(1.0); //Math.abs(foodOne.getVelocity()));
      foodTwo.setSpeed(1.0); //Math.abs(foodTwo.getVelocity()));
   }

   private void processPredatorCollision(Predator01 predator, Sprite sprite)
   {
      Food01 food = foodList.findFood(sprite);

      if (food != null)
      {
         processPredatorAndFoodCollision(predator, food);
      }
      
      Wall wall = wallList.findWall(sprite);
      if (wall != null)
      {
         processPredatorAndWallCollision(predator, wall);
      }
   }

   private void processPredatorAndWallCollision(Predator01 predator, Wall wall)
   {
      // Go right through for now.
   }

   private void processPredatorAndFoodCollision(Predator01 predator, Food01 food)
   {
   }

   private void processRobotCollision(RobotChallengeRobot robot, Sprite sprite)
   {
      Predator01 predator = predatorList.findPredator(sprite);
      if (predator != null)
      {
         processRobotAndPredatorCollision(robot, predator);
         return;
      }

      Food01 food = foodList.findFood(sprite);
      if (food != null)
      {
         processRobotAndFoodCollision(robot, food);
         return;
      }

      Flag flag = flagList.findFlag(sprite);
      if (flag != null)
      {
         processRobotAndFlagCollision(robot, flag);
         return;
      }

      Wall wall = wallList.findWall(sprite);
      if (wall != null)
      {
         processRobotAndWallCollision(robot, wall);
         return;
      }

   }

   private void processRobotAndWallCollision(RobotChallengeRobot robot, Wall wall)
   {
      robot.hitWall();
      robotChallengeRules.hitWall();
   }

   private void processRobotAndFlagCollision(RobotChallengeRobot robot, Flag flag)
   {
      System.out.println("Robot captured flag " + flag.getId());
      Flag droppedFlag = robot.dropFlag();
      if (droppedFlag != null)
      {
//         System.out.println("Robot dropping flag " + droppedFlag.getId());

         Vector2D headingVector = robot.getHeadingVector();
         headingVector.scale(-1.5);

         Point2D position = robot.getPosition();
         position.add(headingVector);

         droppedFlag.setLocation(position);
         flagList.droppedFlag(droppedFlag, spriteWorld, collisionGroup);
      }

      if (robotChallengeRules != null)
      {
         robot.capturedFlag(flag);
         flagList.capturedFlag(flag, spriteWorld, collisionGroup);
         robotChallengeRules.capturedFlag(flag.getId());
      }

      if (droppedFlag != null)
      {
         robotChallengeRules.droppedFlag(droppedFlag.getId());
      }

   }

   private void processRobotAndPredatorCollision(RobotChallengeRobot robot, Predator01 predator)
   {
      robot.getHitByPredator(predator);
      predator.teleportToRandomLocation(random);
   }

   private void processRobotAndFoodCollision(RobotChallengeRobot robot, Food01 food)
   {
      robot.eatFood(food);

      foodList.removeFood(food);
      foodList.createSomeFood(random, xMax, yMax, spriteWorld, collisionGroup);
   }

   public void setRobotChallengeRules(RobotChallengeRules robotChallengeRules)
   {
      this.robotChallengeRules = robotChallengeRules;
   }

}
