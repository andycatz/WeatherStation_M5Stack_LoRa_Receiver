/**
 * Code to upload data to weather underground server.
 */
 #include <HTTPClient.h>
 #include <WiFiClient.h>
 char SERVER [] = "rtupdate.wunderground.com";
 char ID []= "ISHORE1";
 char PASSWORD [] = "CCgji8rm";

 char WEBPAGE [] = "GET /weatherstation/updateweatherstation.php?";

 void wunderground(float tempF, float rh, float pressureInHG, float windDir, float wsMPH, float wgMPH, float dpF, float solarIr, float uvIndex, float rain1hIn, float dayRainIn, float indoorTempF, float indoorRH){
    if(WiFi.status() != WL_CONNECTED){
      log("WiFi not connected while doing wu update\r\n");
    }
    WiFiClient client;
    if(client.connect(SERVER,80)){
      Serial.print(F("...Connected to Wunderground server: "));
      Serial.print(SERVER);
      char c = client.read();
      Serial.print(F(", Server response: "));
      Serial.write(c);
      Serial.println(F(""));
      Serial.println(F("... Sending DATA "));
      Serial.println(F(""));
      client.print(WEBPAGE);
      client.print("ID=");
      client.print(ID);
      client.print("&PASSWORD=");
      client.print(PASSWORD);
      client.print("&dateutc=now");
      if(trhRx>0){
        client.print("&tempf=");
        client.print(tempF);
        client.print("&dewptf=");
        client.print(dpF);
        client.print("&humidity=");
        client.print(rh);
      }
      client.print("&baromin=");
      client.print(pressureInHG);
      if(uvlRx>0){
        client.print("&solarradiation=");
        client.print(solarIr);
        client.print("&UV=");
        client.print(uvIndex);
      }
      if(rainRx>0){
        client.print("&dailyrainin=");
        client.print(dayRainIn);
      }
      client.print("&indoortempf=");
      client.print(indoorTempF);
      client.print("&indoorhumidity=");
      client.print(indoorRH);
      client.print("&softwaretype=Arduino%20M5%20version1&action=updateraw&realtime=1&rtfreq=60");   //Using Rapid Fire, sending data 1time every 60sec  
      client.println("/ HTTP/1.1\r\nHost: host:port\r\nConnection: close\r\n\r\n"); // Working since first days of March 2017
      Serial.println(F("...Server Response:"));
      while(client.connected()){
        while(client.available()){
          char c= client.read();
          Serial.write(c);
        }
        log("WU OK\r\n");
      }
      client.flush();
      client.stop();
    }
    else{
      Serial.println(F("Connection failed"));
      log("WU Not OK\r\n");
      //char c = client.read();
      //Serial.write(c);
      //client.flush();
      client.stop();  
    }
 }
