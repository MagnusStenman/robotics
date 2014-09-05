import java.util.Map;

public class InclinometerResponse implements Response
{
   private Map<String, Object> data;

   public void setData(Map<String, Object> data)
   {
      this.data = data;
   }

   public double getPitchAngle()
   {
      return Double.parseDouble( (String)data.get("PitchAngle") );
   }

   public double getRollAngle()
   {
      return Double.parseDouble( (String)data.get("RollAngle") );
   }

   public String getPath()
   {
      return "/lokarria/inclinometer";
   }
   
   public long getTimestamp()
   {
      return (Long)data.get("TimeStamp");
   }

}


