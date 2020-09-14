package us.ihmc.javaSpriteWorld.examples.robotChallenge01;

import java.util.ArrayList;
import java.util.HashMap;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;

import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.javaSpriteWorld.Sprite;
import us.ihmc.javaSpriteWorld.SpriteCollisionGroup;
import us.ihmc.javaSpriteWorld.SpriteWorld;

public class FlagList
{
   private final ArrayList<Flag> flagsInPlay = new ArrayList<Flag>();
   private final HashMap<Sprite, Flag> map = new HashMap<Sprite, Flag>();

   public void createAFlag(int id, double x, double y, SpriteWorld spriteWorld, SpriteCollisionGroup collisionGroup)
   {
      Flag flag = new Flag(id, x, y);
      addFlag(flag, spriteWorld, collisionGroup);
   }

   public void addFlag(Flag flag, SpriteWorld spriteWorld, SpriteCollisionGroup collisionGroup)
   {
      if (!spriteWorld.getSprites().contains(flag.getSprite()))
      {
         spriteWorld.addSprite(flag.getSprite());
         collisionGroup.addSprite(flag.getSprite());
      }

      if (!flagsInPlay.contains(flag))
      {
         flagsInPlay.add(flag);
      }

      flag.getSprite().show();

      if (map.get(flag.getSprite()) == null)
      {
         map.put(flag.getSprite(), flag);
      }
   }

   public void removeFlag(Flag flag, SpriteWorld spriteWorld, SpriteCollisionGroup collisionGroup)
   {
      collisionGroup.removeSprite(flag.getSprite());
      spriteWorld.removeSprite(flag.getSprite());
      flagsInPlay.remove(flag);
      flag.getSprite().hide();

      map.remove(flag.getSprite(), flag);
   }

   public Flag findFlag(Sprite sprite)
   {
      Flag flag = map.get(sprite);
      return flag;
   }

   public void doDynamicsAndUpdateSprites(double dt)
   {
      for (Flag flag : flagsInPlay)
      {
         flag.doDynamicsAndUpdateSprite(dt);
      }
   }

   public ArrayList<Pair<Vector2D, Integer>> getLocationAndIdsOfAllFlags()
   {
      ArrayList<Pair<Vector2D, Integer>> locationsAndIds = new ArrayList<Pair<Vector2D, Integer>>();

      for (Flag flag : flagsInPlay)
      {
         Vector2D location = new Vector2D(flag.getX(), flag.getY());

         int id = flag.getId();

         locationsAndIds.add(new ImmutablePair<Vector2D, Integer>(location, id));
      }

      return locationsAndIds;
   }

}
