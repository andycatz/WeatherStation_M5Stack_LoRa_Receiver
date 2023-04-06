#define HPA_TO_INHG 0.02953
#define MM_TO_IN 0.0393701
#define LUXTOWM2 0.0079

/**
 * Converts Celsius to Farenheit.
 */
float CelsiusToFarenheit(float tempC){
  return (tempC * 9/5 + 32);
}

/**
 * Converts hectoPascals to inches of Mercury.
 */
float HectoPascalsToInMercury(float phPa){
  return (phPa * HPA_TO_INHG);
}

/**
 * Converts millimetres to inches.
 */
float millimetresToInches(float mm){
  return (mm * MM_TO_IN);
}

float luxToWattsPerSquareMetre(float lux){
  return (lux * LUXTOWM2);
}
