import java.util.Map;

public class LaserPropertiesResponse implements Response
{
   private Map<String, Object> data;

   public void setData(Map<String, Object> data)
   {
      this.data = data;
   }

   public double[] getOrientation()
   {
      Map<String, Object> pose = (Map<String, Object>)data.get("Pose");
      Map<String, Object> orientation = (Map<String, Object>)pose.get("Orientation");
      
      double w =  (Double)orientation.get("W");      
      double x =  (Double)orientation.get("X");
      double y =  (Double)orientation.get("Y");
      double z =  (Double)orientation.get("Z");
      
      return new double[] {w, x, y, z};
   }

   public double[] getPosition()
   {
      Map<String, Object> pose = (Map<String, Object>)data.get("Pose");
      Map<String, Object> position = (Map<String, Object>)pose.get("Position");

      double x =  (Double)position.get("X");
      double y =  (Double)position.get("Y");
      double z =  (Double)position.get("Z");
      
      return new double[] {x, y, z};
   }

   public String getPath()
   {
      return "/lokarria/laser/properties";
   }

   public long getTimestamp()
   {
      return 0;
   }

}
