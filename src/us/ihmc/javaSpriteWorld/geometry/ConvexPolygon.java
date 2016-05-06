package us.ihmc.javaSpriteWorld.geometry;

import java.util.ArrayList;

public class ConvexPolygon
{
   private final ArrayList<Point> vertices = new ArrayList<>();
   private final AxisAlignedBoundingBox2D boundingBox = new AxisAlignedBoundingBox2D();

   private boolean edgeNormalsDirty = true;
   private boolean boundingBoxDirty = true;

   private final ArrayList<Vector> edgeNormals = new ArrayList<>();

   public ConvexPolygon(ArrayList<Point> pointsInCounterclockwiseOrder)
   {
      for (int i = 0; i < pointsInCounterclockwiseOrder.size(); i++)
      {
         this.vertices.add(new Point(pointsInCounterclockwiseOrder.get(i)));
      }
   }

   public ConvexPolygon(ConvexPolygon collisionPolygon)
   {
      this(collisionPolygon.vertices);
   }

   public static ConvexPolygon createRectangle(Point oneCorner, Vector vectorToOtherCorner)
   {
      ArrayList<Point> points = new ArrayList<Point>();

      points.add(new Point(oneCorner));

      Point point = new Point(oneCorner);
      point.setX(point.getX() + vectorToOtherCorner.getX());
      points.add(new Point(point));

      point.setY(point.getY() + vectorToOtherCorner.getY());
      points.add(new Point(point));

      point.setX(oneCorner.getX());
      points.add(new Point(point));

      ConvexPolygon polygon = new ConvexPolygon(points);
      return polygon;
   }
   
   public boolean isEntirelyInside(AxisAlignedBoundingBox2D boundingBox)
   {
      for (int i=0; i<vertices.size(); i++)
      {
         Point point = vertices.get(i);
         
         if (!boundingBox.isPointInside(point)) return false;
      }
      
      return true;
   }


   public boolean isPointInsideBoundingBox(double xToCheck, double yToCheck)
   {
      if (boundingBoxDirty) computeBoundingBox();

      return boundingBox.isPointInside(xToCheck, yToCheck);
   }

   public void transform(boolean flipX, boolean flipY, double xTranslation, double yTranslation, double rotation)
   {
      for (Point point : vertices)
      {
         point.transform(flipX, flipY, xTranslation, yTranslation, rotation);
      }

      edgeNormalsDirty = true;
      boundingBoxDirty = true;
   }

   public void getAxisAlignedBoundingBox(AxisAlignedBoundingBox2D boundingBoxToPack)
   {
      if (boundingBoxDirty)
         computeBoundingBox();

      boundingBoxToPack.set(this.boundingBox);
   }

   private void computeBoundingBox()
   {
      this.boundingBox.setToNaN();

      for (int i = 0; i < vertices.size(); i++)
      {
         Point point = vertices.get(i);
         boundingBox.expandToInclude(point);
      }
   }

   public void getEdgeNormals(ArrayList<Vector> edgeNormalsToPack)
   {
      if (edgeNormalsDirty)
         computeEdgeNormals();

      edgeNormalsToPack.addAll(edgeNormals);
   }

   private void computeEdgeNormals()
   {
      if (edgeNormals.isEmpty())
      {
         for (int i = 0; i < vertices.size(); i++)
         {
            edgeNormals.add(new Vector());
         }
      }

      for (int i = 0; i < vertices.size(); i++)
      {
         Vector edgeNormal = edgeNormals.get(i);

         Point firstPoint = vertices.get(i);
         Point secondPoint;
         if (i == vertices.size() - 1)
            secondPoint = vertices.get(0);
         else
            secondPoint = vertices.get(i + 1);

         edgeNormal.set(secondPoint.getY() - firstPoint.getY(), -(secondPoint.getX() - firstPoint.getX()));
         edgeNormal.normalize();
      }

      edgeNormalsDirty = false;
   }

   public ArrayList<Point> getVertices()
   {
      return vertices;

   }

   public void set(ConvexPolygon convexPolygon)
   {
      for (int i = 0; i < this.vertices.size(); i++)
      {
         this.vertices.get(i).set(convexPolygon.vertices.get(i));
      }

      this.edgeNormalsDirty = true;
      this.boundingBoxDirty = true;
   }

   public void growBoundingBoxToIncludeThisPolygon(AxisAlignedBoundingBox2D axisAlignedBoundingBoxToPack)
   {
      if (boundingBoxDirty) computeBoundingBox();
      
      axisAlignedBoundingBoxToPack.expandToInclude(boundingBox);
   }

   public String toString()
   {
      String stringToReturn = "";

      for (Point point : vertices)
      {
         stringToReturn += point.toString() + " ";
      }

      return stringToReturn;
   }

  
}
