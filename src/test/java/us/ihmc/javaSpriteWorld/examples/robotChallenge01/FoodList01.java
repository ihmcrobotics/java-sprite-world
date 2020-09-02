package us.ihmc.javaSpriteWorld.examples.robotChallenge01;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Random;

import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.javaSpriteWorld.Sprite;
import us.ihmc.javaSpriteWorld.SpriteCollisionGroup;
import us.ihmc.javaSpriteWorld.SpriteWorld;

public class FoodList01
{
   private final ArrayList<Food01> foodInPlay = new ArrayList<Food01>();
   private final ArrayList<Food01> recycledFood = new ArrayList<Food01>();
   private final HashMap<Sprite, Food01> map = new HashMap<Sprite, Food01>();

   public FoodList01()
   {
      
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
         food = new Food01(random, xMax, yMax);
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

   public ArrayList<Vector2D> getLocationOfAllFood()
   {
      ArrayList<Vector2D> locations = new ArrayList<Vector2D>();
      
      for (Food01 food : foodInPlay)
      {
         Vector2D location = new Vector2D(food.getX(), food.getY());
         locations.add(location);
      }
      
      return locations;
      
   }

}