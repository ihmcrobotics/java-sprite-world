package us.ihmc.javaSpriteWorld.examples.robotChallenge01;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.util.ArrayList;

import javafx.embed.swing.SwingFXUtils;
import javafx.scene.image.Image;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.javaSpriteWorld.Sprite;
import us.ihmc.javaSpriteWorld.SpriteCostume;
import us.ihmc.javaSpriteWorld.geometry.ConvexPolygon;
import us.ihmc.javaSpriteWorld.geometry.Point;

public class Wall
{
   private static final double wallThickness = 0.1;

   private final Sprite sprite;

   private final Point2D pointOne, pointTwo;
   private final LineSegment2D lineSegment;

   public Wall(Point2D pointOne, Point2D pointTwo)
   {
      this.pointOne = new Point2D(pointOne);
      this.pointTwo = new Point2D(pointTwo);

      this.lineSegment = new LineSegment2D(pointOne, pointTwo);
      ArrayList<Point2D> wallPoints = getWallPoints();

      sprite = createSprite(wallPoints);

      Point2D[] minAndMaxPoints = getMinAndMaxPoints(wallPoints);
      Point2D minPoint = minAndMaxPoints[0];
      
      ArrayList<Point> wallPointsForCollisionPolygon = new ArrayList<Point>();
      for (Point2D wallPoint : wallPoints)
      {
         wallPointsForCollisionPolygon.add(new Point(wallPoint.getX() - minPoint.getX(), wallPoint.getY() - minPoint.getY()));
      }
      
      System.out.println("\nCreating wall. LineSegment = " + lineSegment + " with points : " + wallPoints + " with collision polygon points = " + wallPointsForCollisionPolygon);


      ConvexPolygon collisionPolygon = new ConvexPolygon(wallPointsForCollisionPolygon);
      sprite.addCollisionPolygon(collisionPolygon);
   }

   private ArrayList<Point2D> getWallPoints()
   {
      Vector2D vector = new Vector2D(pointTwo);
      vector.sub(pointOne);
      vector.normalize();

      Vector2D perpendicular = new Vector2D(-vector.getY(), vector.getX());

      ArrayList<Point2D> points = new ArrayList<Point2D>();
      perpendicular.scale(wallThickness / 2.0);

      Point2D pointA = new Point2D(pointOne);
      pointA.add(perpendicular);
      points.add(pointA);

      Point2D pointB = new Point2D(pointOne);
      pointB.sub(perpendicular);
      points.add(pointB);

      Point2D pointC = new Point2D(pointTwo);
      pointC.sub(perpendicular);
      points.add(pointC);

      Point2D pointD = new Point2D(pointTwo);
      pointD.add(perpendicular);
      points.add(pointD);

      return points;
   }

   private Point2D[] getMinAndMaxPoints(ArrayList<Point2D> wallPoints)
   {
      double xMin = Double.POSITIVE_INFINITY;
      double yMin = Double.POSITIVE_INFINITY;

      double xMax = Double.NEGATIVE_INFINITY;
      double yMax = Double.NEGATIVE_INFINITY;

      for (Point2D wallPoint : wallPoints)
      {
         if (wallPoint.getX() < xMin)
            xMin = wallPoint.getX();
         if (wallPoint.getY() < yMin)
            yMin = wallPoint.getY();

         if (wallPoint.getX() > xMax)
            xMax = wallPoint.getX();
         if (wallPoint.getY() > yMax)
            yMax = wallPoint.getY();
      }
      
      return new Point2D[] {new Point2D(xMin, yMin), new Point2D(xMax, yMax)};
   }
   
   
   private Sprite createSprite(ArrayList<Point2D> wallPoints)
   {
      Sprite sprite = new Sprite("Wall");

      Point2D[] minAndMaxPoints = getMinAndMaxPoints(wallPoints);
      Point2D minPoint = minAndMaxPoints[0];
      Point2D maxPoint = minAndMaxPoints[1];
      
      double deltaX = Math.abs(maxPoint.getX() - minPoint.getX());
      double deltaY = Math.abs(maxPoint.getY() - minPoint.getY());

      double scale = 100.0;
      int xPixels = (int) (deltaX * scale);
      int yPixels = (int) (deltaY * scale);

      BufferedImage bufferedImage = new BufferedImage(xPixels, yPixels, BufferedImage.TYPE_INT_ARGB);
      Graphics2D graphics2D = bufferedImage.createGraphics();

      graphics2D.setColor(Color.blue);

      int[] xPoints = new int[wallPoints.size()];
      int[] yPoints = new int[wallPoints.size()];
      int nPoints = wallPoints.size();

      for (int i = 0; i < wallPoints.size(); i++)
      {
         Point2D wallPoint = wallPoints.get(i);
         xPoints[i] = (int) ((wallPoint.getX() - minPoint.getX()) * scale);
         yPoints[i] = (int) ((wallPoint.getY() - minPoint.getY()) * scale);
      }
      graphics2D.fillPolygon(xPoints, yPoints, nPoints);

      graphics2D.dispose();

      Image fxImage = SwingFXUtils.toFXImage(bufferedImage, null);
      SpriteCostume costume = new SpriteCostume(fxImage);
      costume.setXReferencePercent(0.0);
      costume.setYReferencePercent(0.0);
      sprite.addCostume(costume);

      sprite.setX(minPoint.getX());
      sprite.setY(minPoint.getY());

      sprite.setWidth(deltaX);
      sprite.setHeight(deltaY);

      return sprite;
   }

   public Sprite getSprite()
   {
      return sprite;
   }

   public Point2DReadOnly getPointOne()
   {
      return pointOne;
   }

   public Point2DReadOnly getPointTwo()
   {
      return pointTwo;
   }

   public LineSegment2DReadOnly getLineSegment()
   {
      return lineSegment;
   }

}
