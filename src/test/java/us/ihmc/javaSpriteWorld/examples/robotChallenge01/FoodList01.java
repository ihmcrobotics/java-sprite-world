package us.ihmc.javaSpriteWorld.examples.robotChallenge01;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Random;

import org.apache.commons.lang3.tuple.ImmutableTriple;
import org.apache.commons.lang3.tuple.Triple;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.javaSpriteWorld.Sprite;
import us.ihmc.javaSpriteWorld.SpriteCollisionGroup;
import us.ihmc.javaSpriteWorld.SpriteWorld;

public class FoodList01
{
   private final double xMax, yMax;
   
   private final ArrayList<Food01> foodInPlay = new ArrayList<Food01>();
   private final ArrayList<Food01> recycledFood = new ArrayList<Food01>();
   private final HashMap<Sprite, Food01> map = new HashMap<Sprite, Food01>();

   private int nextId = 1;
   
   public FoodList01(double xMax, double yMax)
   {
      this.xMax = xMax;
      this.yMax = yMax;
   }

   public void createSomeFood(Random random, SpriteWorld spriteWorld, SpriteCollisionGroup collisionGroup)
   {
      createSomeFood(random, xMax, yMax, spriteWorld, collisionGroup);
   }
   
   public void reset(Random random)
   {
      for (Food01 food : foodInPlay)
      {
         food.teleportToRandomLocation(random);
      }
      
      for (Food01 food : recycledFood)
      {
         food.teleportToRandomLocation(random);
      }
   }
   
   public void createSomeFood(Random random, double xMax, double yMax, SpriteWorld spriteWorld, SpriteCollisionGroup collisionGroup)
   {
      Food01 food;

      if (!recycledFood.isEmpty())
      {
         food = recycledFood.remove(0);
         food.teleportToRandomLocation(random);
      }
      else
      {
         food = new Food01(nextId++, random, xMax, yMax);
         spriteWorld.addSprite(food.getSprite());
         collisionGroup.addSprite(food.getSprite());
      }

      foodInPlay.add(food);

      food.getSprite().show();
      map.put(food.getSprite(), food);
   }

   public void removeFood(Food01 food)
   {
      foodInPlay.remove(food);
      food.getSprite().hide();
      if (!recycledFood.contains(food))
      {
         recycledFood.add(food);
      }

      map.remove(food.getSprite(), food);
   }

   public Food01 findFood(Sprite sprite)
   {
      Food01 food = map.get(sprite);
      return food;
   }

   public void doDynamicsAndUpdateSprites(double dt)
   {
      for (Food01 food : foodInPlay)
      {
         food.doDynamicsAndUpdateSprite(dt);
      }
   }

   public ArrayList<Triple<Integer, Point2D, Vector2D>> getLocationAndVelocityOfAllFood()
   {
      ArrayList<Triple<Integer, Point2D, Vector2D>> locationsAndVelocities = new ArrayList<Triple<Integer, Point2D, Vector2D>>();

      for (Food01 food : foodInPlay)
      {
         Point2D location = new Point2D(food.getX(), food.getY());

         double heading = food.getHeading();
         double speed = food.getSpeed();

         Vector2D velocity = new Vector2D(speed * Math.cos(heading), speed * Math.sin(heading));
         locationsAndVelocities.add(new ImmutableTriple<Integer, Point2D, Vector2D>(food.getId(), location, velocity));
      }

      return locationsAndVelocities;
   }

}
