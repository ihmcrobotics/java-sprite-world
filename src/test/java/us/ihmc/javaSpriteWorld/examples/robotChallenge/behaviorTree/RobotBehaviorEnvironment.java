package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree;

import us.ihmc.euclid.geometry.LineSegment2D;

import java.util.ArrayList;

public class RobotBehaviorEnvironment
{
   private ArrayList<LineSegment2D> walls = new ArrayList<>();
   private final double mapSizeX;
   private final double mapSizeY;

   public RobotBehaviorEnvironment(int challengeNumber)
   {
      double mapSize = challengeNumber >= 8 ? 20.0 : 10.0;
      mapSizeX = mapSize;
      mapSizeY = mapSize;

      walls.add(new LineSegment2D(0.0, 0.0, 0.0, getMapSizeY()));
      walls.add(new LineSegment2D(0.0, getMapSizeY(), 10.0, getMapSizeY()));
      walls.add(new LineSegment2D(getMapSizeX(), getMapSizeY(), getMapSizeX(), 0.0));
      walls.add(new LineSegment2D(getMapSizeX(), 0.0, 0.0, 0.0));

      if (challengeNumber == 7)
      {
         walls.add(new LineSegment2D(2.0 / 3.0 * mapSizeX,
                                     1.0 / 3.0 * mapSizeY,
                                     2.0 / 3.0 * mapSizeX,
                                     2.0 / 3.0 * mapSizeY));
      }

      if (challengeNumber == 8)
      {
         double x0 = 0.0;
         double x1 = 0.35 * mapSizeX;
         double x2 = 0.6 * mapSizeX;

         double y0 = 0.0;
         double y1 = 0.3 * mapSizeY;
         double y2 = 0.45 * mapSizeY;
         double y3 = 0.65 * mapSizeY;
         double y4 = 0.8 * mapSizeY;

         walls.add(new LineSegment2D(x0, y2, x1, y2));
         walls.add(new LineSegment2D(x1, y2, x1, y3));
         walls.add(new LineSegment2D(x1, y4, x1, mapSizeY));

         walls.add(new LineSegment2D(x2, y0, x2, y1));
         walls.add(new LineSegment2D(x2, y2, x2, y3));
         walls.add(new LineSegment2D(x2, y3, mapSizeX, y3));

         walls.add(new LineSegment2D(x2, y4, x2, mapSizeY));
      }
   }

   public ArrayList<LineSegment2D> getWalls()
   {
      return walls;
   }

   public double getMapSizeX()
   {
      return mapSizeX;
   }

   public double getMapSizeY()
   {
      return mapSizeY;
   }
}
