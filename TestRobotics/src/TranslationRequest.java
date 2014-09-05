import java.util.HashMap;
import java.util.Map;

public class TranslationRequest implements Request
{
   private Map<String, Object> data;

   public TranslationRequest()
   {
      data = new HashMap<String, Object>();

      data.put("Distance", 0);
      data.put("MaxSpeed", 0);
   }

   public void setDistance(double distance)
   {
      data.put("Distance", distance);
   }

   public void setMaxSpeed(double maxSpeed)
   {
      data.put("MaxSpeed", maxSpeed);
   }

   public Map<String, Object> getData()
   {
      return data;
   }

   public String getPath()
   {
      return "/lokarria/translate";
   }


}
