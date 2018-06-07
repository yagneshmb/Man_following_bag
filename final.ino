#include <SoftwareSerial.h> //library for serial communication
#include <TinyGPS.h> //library for GPS sensor
#include <Wire.h> //library for compass

#define addr 0x0D
#define RADTODEG 57.295779513082320876f
#define DEGTORAD 0.0174532925199432957f

SoftwareSerial bluetoothSerial(2, 3); // RX, TX for bluetooth module
SoftwareSerial gpsSerial(4,5); //RX, TX for GPS module
String mobileLocation = ""; //string recieved via location sensor of mobile of users'
String mobileLocationLat, mobileLocationLon; //string for distinguishing latitude and logitude
TinyGPS gps;

const int col11 = 6;  //L293D pin 2
const int col21 = 7;  //l293D pin 6
const int col12 = 8;  //L293D pin 10
const int col22 = 9;  //L293D pin 14
 const int trigPin = 10; 
  const int echoPin = 11;
float bearing;
String bearingDirection;
float heading;
String headingDirection;
float threshold = 0.001;// define threshold.

String bluetoothData;
String GPSData; //string recieved via gps sensor of trolley
void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
   while (!Serial) {
    ; // wait for serial port to connect.
  }
 
  Wire.beginTransmission(addr); //open communication with HMC5883
  Wire.write(0x09); //select mode register
  Wire.write(0x0D); //continuous measurement mode
  Wire.endTransmission();

  Wire.beginTransmission(addr); //open communication with HMC5883
  Wire.write(0x0B); //select Set/Reset period register
  Wire.write(0x01); //Define Set/Reset period
  Wire.endTransmission();
  
  
  pinMode(col11 , OUTPUT); //declare output
  pinMode(col12 , OUTPUT);
  pinMode(col21 , OUTPUT);
  pinMode(col22 , OUTPUT);
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);
  


  // set the data rate for the SoftwareSerial port
  bluetoothSerial.begin(9600);
  gpsSerial.begin(9600);

}



float getBearing(float lat1, float lon1, float lat2, float lon2){
   lat1 = lat1*DEGTORAD;
   lat2 = lat2*DEGTORAD;
   lon1 = lon1*DEGTORAD;
   lon2 = lon2*DEGTORAD;
   float y = sin(lon2-lon1) * cos(lat2);
  float x = (cos(lat1)*sin(lat2)) - (sin(lat1)*cos(lat2)*cos(lon2 - lon1));
  
  float z =  atan2(y, x) * RADTODEG;
  z =  (((int )z + 360)%360);
  
  return z;
   
}
float getDistance(float lat1, float lon1, float lat2, float lon2){
  const float R = 6371000; // radius of earth km
  float p1 = lat1 * DEGTORAD;
  float p2 = lat2 * DEGTORAD;
  float dp = (lat2-lat1) * DEGTORAD;
  float dl = (lon2-lon1) * DEGTORAD;
  float x = sin(dp/2) * sin(dp/2) + cos(p1) * cos(p2) * sin(dl/2) * sin(dl/2);
  float y = 2 * atan2(sqrt(x), sqrt(1-x));

  return R * y; 
}

String getBearingDirection(float bearing){
    if((bearing > 337.5) || (bearing < 22.5))    return "North";
  if((bearing > 22.5)  && (bearing < 67.5 ))   return "North-East";
  if((bearing > 67.5)  && (bearing < 112.5 ))  return "East";
  if((bearing > 112.5) && (bearing < 157.5 ))  return "South-East";
  if((bearing > 157.5) && (bearing < 202.5 ))  return "South";
  if((bearing > 202.5) && (bearing < 247.5 ))  return "South-West";
  if((bearing > 247.5) && (bearing < 292.5 ))  return "West";
  if((bearing > 292.5) && (bearing < 337.5 ))  return "North-West";
  
  }




    
String getHeading(float trolley_lat, float trolley_lon){
  float angle;
int compassX, compassY, compassZ;
  Wire.beginTransmission(addr);
  Wire.write(0x00); //select register 0, X MSB register
  Wire.endTransmission();
  
  //Read data from each axis, 2 registers per axis
  Wire.requestFrom(addr, 6);
  if(Wire.available()>=6){
    compassX = Wire.read(); //x lsb
    compassX |= Wire.read()<<8; //x msb
    compassY = Wire.read(); //y lsb
    compassY |= Wire.read()<<8;; //y msb
    compassZ = Wire.read(); //z lsb
    compassZ |= Wire.read()<<8; //z msb
  
}
angle = (double)atan2(compassY, compassX); // angle in radians
 
  float declinationAngle = 0.00843517627f; //declination angle at SEAS
  angle += declinationAngle;
  
  // Correct for when signs are reversed.
  if (angle < 0)    angle += 2*PI;
    
  // Check for wrap due to addition of declination.
  if (angle > 2*PI) angle -= 2*PI;
   
  // Convert radians to degrees for readability.
  float heading = angle * 180/PI; 

  
  if((heading > 337.5) || (heading < 22.5)) return "North";
  if((heading > 22.5)  && (heading < 67.5 ))   return "North-East";
  if((heading > 67.5)  && (heading < 112.5 ))  return "East";
  if((heading > 112.5) && (heading < 157.5 ))  return "South-East";
  if((heading > 157.5) && (heading < 202.5 ))  return "South";
  if((heading > 202.5) && (heading < 247.5 ))  return "South-West";
  if((heading > 247.5) && (heading < 292.5 ))  return "West";
  if((heading > 292.5) && (heading < 337.5 ))  return "North-West";
  
  
  return getBearingDirection(heading);
  }

String getBluetoothData(){
  
    bluetoothSerial.listen();  //for we have two serial port  
  if(bluetoothSerial.available()){
    while(bluetoothSerial.available()){  
        mobileLocation = bluetoothSerial.readString();
        return mobileLocation;
  }
       
  }
  else{
    Serial.println("no blueooth signal");
    return "0";
       
  }
}


String getGPSData(){
  gpsSerial.listen();
  
  float bagLocationlat, bagLocationLon;
  String GPSString;
  //gives trolley gps coordinates.
if(gpsSerial.available()){  
  while(gpsSerial.available()){
   
    //delay(5000);
      if(gps.encode(gpsSerial.read())){
        Serial.println("gps encoding started");
          gps.f_get_position(&bagLocationLat, &bagLocationLon);
      GPSString = String(bagLocationLat, 8) + String(bagLocationLon, 8);
      return GPSString;
     }
  }
 }
 else{
  Serial.println("no gps signal");
  return  "0";
 }
 
}

int getObstacleData(int threshold){
 
  // defines variables
  long duration;
  int distance;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
// Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
// Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
// Calculating the distance
  distance= duration*0.034/2;
  if(distance < threshold){
    return 1; 
  }
  else{
    return 0;
  }
  
}


void loop() {

  String trolleyLocationLat;  //trolley location latitude 
  String trolleyLocationLon ; //trolley location langitude
  float trolley_location_lat ; //trolley location in float
  float trolley_location_lon; //trolley location in float 
  
 mobileLocation = getBluetoothData(); //get data via bluetooth from phone
 GPSData = getGPSData(); //get locaiton of trolley
 
 trolleyLocationLat = GPSData.substring(0,8);
 trolleyLocationLon = GPSData.substring(8); 
  trolley_location_lat = trolleyLocationLat.toFloat(); 
  trolley_location_lon = trolleyLocationLon.toFloat();
 
  mobileLocationLat = mobileLocation.substring(0,8);
  mobileLocationLon = mobileLocation.substring(8); 
  float mobile_location_lat = mobileLocationLat.toFloat(); 
  float mobile_location_lon = mobileLocationLon.toFloat();
 
  bearing = getBearing(trolley_location_lat, trolley_location_lon,mobile_location_lat,mobile_location_lon); //direction in which trolley should to be able to reach destination
  bearingDirection = getBearingDirection(bearing);  //get direction i.e., north, south, north-east etc.
  headingDirection = getHeading(trolley_location_lat, trolley_location_lon);//direction in which trolley currently is in
    if(bearingDirection!= headingDirection){  //make trolley rotate until it heads to desirable direction
    /*rotate wheels so that motor rotate*/
    digitalWrite(col11,HIGH);
    digitalWrite(col12,LOW);    
    digitalWrite(col22,HIGH);
    digitalWrite(col21,LOW);   
  }
  if(bearingDirection==headingDirection){
    /*when trolley heading to desirable direction move forward*/
    digitalWrite(col11,HIGH);
    digitalWrite(col21,HIGH);
    digitalWrite(col22,LOW);
    digitalWrite(col12,LOW);            
  }
  if(getObstacledata() == 1){
    /*rotate until get a clear road*/
    digitalWrite(col11,HIGH);
    digitalWrite(col12,LOW);    
    digitalWrite(col22,HIGH);
    digitalWrite(col21,LOW);  
  }
  
  
  float distance = getDistance(trolley_location_lat, trolley_location_lon,mobile_location_lat,mobile_location_lon);
  threshold = 0.001 /*(km)*/;
 if(distance==threshold){ //we assume that trolley will maintain a constant distnace 
  
    digitalWrite(leftForward,LOW);
    digitalWrite(rightForward,LOW);
    digitalWrite(rightBackward,LOW);
    digitalWrite(leftBackward,LOW);
    
  }
}

