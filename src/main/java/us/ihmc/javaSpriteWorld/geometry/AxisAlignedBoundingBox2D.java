package us.ihmc.javaSpriteWorld.geometry;

public class AxisAlignedBoundingBox2D
{   
   private Point minPoint = new Point(Double.NaN, Double.NaN);
   private Point maxPoint = new Point(Double.NaN, Double.NaN);
   
   public void setToNaN()
   {
      minPoint.setToNaN();
      maxPoint.setToNaN();
   }
   
   public boolean isNaN()
   {
      if (minPoint.isNaN()) return true;
      if (maxPoint.isNaN()) return true;
      
      return false;
   }
   
   public void setMinPoint(Point minPoint)
   {
      this.minPoint.set(minPoint);
   }
   
   public void setMaxPoint(Point maxPoint)
   {
      this.maxPoint.set(maxPoint);
   }
   
   public void getMinPoint(Point minPointToPack)
   {
      minPointToPack.set(this.minPoint);
   }
   
   public void getMaxPoint(Point maxPointToPack)
   {
      maxPointToPack.set(this.maxPoint);
   }
   
   public void expandToInclude(Point point)
   {
      if (isNaN())
      {
         minPoint.set(point);
         maxPoint.set(point);
      }
      
      minPoint.setX(Math.min(minPoint.getX(), point.getX()));
      minPoint.setY(Math.min(minPoint.getY(), point.getY()));
      
      maxPoint.setX(Math.max(maxPoint.getX(), point.getX()));
      maxPoint.setY(Math.max(maxPoint.getY(), point.getY()));
   }
   
   public void expandToInclude(AxisAlignedBoundingBox2D boundingBox)
   {
      if (boundingBox.isNaN()) return;
      
      if (isNaN()) 
      {
         this.set(boundingBox);
      }
      else
      {
         minPoint.setX(Math.min(minPoint.getX(), boundingBox.minPoint.getX()));
         minPoint.setY(Math.min(minPoint.getY(), boundingBox.minPoint.getY()));
         
         maxPoint.setX(Math.max(maxPoint.getX(), boundingBox.maxPoint.getX()));
         maxPoint.setY(Math.max(maxPoint.getY(), boundingBox.maxPoint.getY()));
      }
   }

   public void set(AxisAlignedBoundingBox2D boundingBox)
   {
      this.minPoint.set(boundingBox.minPoint);      
      this.maxPoint.set(boundingBox.maxPoint);      
   }

   public double getMinX()
   {
      return minPoint.getX();
   }
   
   public double getMinY()
   {
      return minPoint.getY();
   }
   
   public double getMaxX()
   {
      return maxPoint.getX();
   }
   
   public double getMaxY()
   {
      return maxPoint.getY();
   }

   public boolean isPointInside(Point point)
   {
      return isPointInside(point.getX(), point.getY());
   }

   public boolean isPointInside(double xToCheck, double yToCheck)
   {
      if (xToCheck < minPoint.getX()) return false;
      if (yToCheck < minPoint.getY()) return false;
      if (xToCheck > maxPoint.getX()) return false;
      if (yToCheck > maxPoint.getY()) return false;
      
      return true;
   }
   
   public boolean isEntirelyInside(AxisAlignedBoundingBox2D boundingBox)
   {
      if (this.isNaN()) return false;
      if (boundingBox.isNaN()) return false;
      
      if (minPoint.getX() < boundingBox.minPoint.getX()) return false;
      if (minPoint.getY() < boundingBox.minPoint.getY()) return false;      
      if (maxPoint.getX() > boundingBox.maxPoint.getX()) return false;
      if (maxPoint.getY() > boundingBox.maxPoint.getY()) return false;
      
      return true;
   }


   public String toString()
   {
      return minPoint + " -- " + maxPoint;
   }


}
