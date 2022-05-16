#include <HCSR04.h>                 // for the USS
#include <NewPing.h>
#include <math.h>

#define M_E 2.7182818284590452354 // euler's constant

// load data from april 5 field test
float values[102] = {166.33
                     ,
                     166.33
                     ,
                     166.33
                     ,
                     166.5
                     ,
                     166.58
                     ,
                     166.75
                     ,
                     167.08
                     ,
                     167.42
                     ,
                     168.08
                     ,
                     168.58
                     ,
                     169.25
                     ,
                     169.75
                     ,
                     170.33
                     ,
                     170.58
                     ,
                     171.0
                     ,
                     171.0
                     ,
                     171.0
                     ,
                     171.0
                     ,
                     171.08
                     ,
                     168.25
                     ,
                     168.5
                     ,
                     168.67
                     ,
                     168.58
                     ,
                     168.75
                     ,
                     168.67
                     ,
                     168.75
                     ,
                     168.17
                     ,
                     168.17
                     ,
                     168.0
                     ,
                     167.83
                     ,
                     167.5
                     ,
                     167.67
                     ,
                     168.75
                     ,
                     168.58
                     ,
                     168.58
                     ,
                     169.75
                     ,
                     170.25
                     ,
                     171.0
                     ,
                     170.92
                     ,
                     170.67
                     ,
                     170.42
                     ,
                     169.67
                     ,
                     168.92
                     ,
                     168.5
                     ,
                     168.0
                     ,
                     167.58
                     ,
                     167.17
                     ,
                     167.25
                     ,
                     166.92
                     ,
                     166.83
                     ,
                     166.75
                     ,
                     166.83
                     ,
                     166.83
                     ,
                     166.75
                     ,
                     166.25
                     ,
                     166.75
                     ,
                     166.58
                     ,
                     166.67
                     ,
                     166.67
                     ,
                     166.58
                     ,
                     166.58
                     ,
                     166.58
                     ,
                     166.75
                     ,
                     167.0
                     ,
                     167.17
                     ,
                     167.42
                     ,
                     167.42
                     ,
                     167.42
                     ,
                     165.33
                     ,
                     168.25
                     ,
                     157.17
                     ,
                     169.08
                     ,
                     169.17
                     ,
                     169.42
                     ,
                     169.42
                     ,
                     157.83
                     ,
                     168.58
                     ,
                     168.5
                     ,
                     167.92
                     ,
                     167.67
                     ,
                     167.5
                     ,
                     167.25
                     ,
                     167.25
                     ,
                     167.17
                     ,
                     167.0
                     ,
                     167.0
                     ,
                     166.75
                     ,
                     155.92
                     ,
                     166.83
                     ,
                     166.75
                     ,
                     166.75
                     ,
                     166.67
                     ,
                     166.75
                     ,
                     166.58
                     ,
                     166.67
                     ,
                     166.67
                     ,
                     166.67
                     ,
                     166.58
                     ,
                     166.58
                     ,
                     166.58
                     ,
                     166.5
                     ,
                     166.5
                    };

float deltas[5];                    // array of second derivatives
float prevDist = -1;                // initialize values to impossible readings
float prevFirstDeriv = -1;
int i = 0;                          // index # of array                   
int skipIndicies = 1; 
float prevTimeInterval = 4;         // initialize previous time interval for calculation
float bottomLimit = 4;              // fastest allowed sampling rate
float topLimit = 16;                // slowest allowed sampling rate

const int window = 5;               // if you change this, be sure to change the arguments of ultrasonicMedian(dist[0-window-1]) in the loop()
float dist[window];                 // array of distance measurements taken by USS
float tolerance = 3;                // how many feet +/- we can expect a valid reading to be within of the last reading (we dont expect it to jump 4 feet in 4 minutes)
float filtered;

// construct the ultrasonic sensor
const int trigPin = 47;         
const int echoPin = 46;         
NewPing sonar(trigPin, echoPin);

void setup() {
  // configure pins for ultrasonic sensor
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.println("Executing asa_apr5.ino");

  for (int i = 0; i < window; i++) {
    dist[i] = values[i];
  }
  Serial.println("array initialized");

  plot();
  
}

void plot() {
  float distance;
  for(int i=window; i<102;) {
    distance = values[i];
    Serial.print(i); Serial.print(", ");
    //Serial.println(distance);
    //delay(200);
    if (distance > dist[window - 1] + tolerance || distance < dist[window - 1] - tolerance) {    // means the value is likely an error! apply median filter
        //filtered = bubble_sort(dist[window-5], dist[window-4], dist[window-3], dist[window-2], dist[window-1]);                     
       // filtered = ultrasonicMedian(dist[0], dist[1], dist[2], dist[3], dist[4]);
       filtered = dist[window-1];
    }
    else {
      filtered = distance;
    }
    //delay(200);
    //Serial.print(", "); 
    Serial.print(filtered);
    delay(100);
    for (int i = 0 ; i < window - 1; i++) {
      dist[i] = dist[i + 1];
      delay(10);
    }
    dist[window - 1] = filtered;
    delay(250);

   // Serial.println("metrics: ---------------------------------------");
    //Serial.print("Distance: ");Serial.println(filtered);
    
    // find first derivative
    if(prevDist != -1) {
      float firstDeriv = abs(filtered - prevDist);
      //Serial.print("first derivative: ");Serial.println(firstDeriv);

      // find second derivative
      if(prevFirstDeriv >= 0) {
        delay(500);
        float secondDeriv = abs(firstDeriv - prevFirstDeriv);
        //Serial.print("second derivative: ");Serial.println(secondDeriv);

        // if the array is not fully populated (happens in the beginning)
        if(i<window){
          deltas[i] = secondDeriv;
          //Serial.println("running 1");
          i++;
        }

        // the array is fully populated, shift everything over one index
        else {
          delay(500);
          for(int j=1;j<window+1;j++) {
              float temp = deltas[j];
              deltas[j-1] = temp;
              delay(100);
          }
          deltas[window-1] = secondDeriv;
          //delay(500);
          float avgSecondDeriv = averageDerivs(deltas);
          //Serial.print("Average second derivative: ");Serial.println(avgSecondDeriv);
          //delay(500);
          changeTime(avgSecondDeriv);
        }
      }
      prevFirstDeriv = firstDeriv;
     }
     prevDist = filtered;
     i = i+skipIndicies;
     //Serial.print("next index: "); Serial.println(i);

    //printArray(deltas);
    delay(2000);
  }
}  

float bubble_sort(float a, float b, float c, float d, float e) {
  float sort[window] = {a, b, c, d, e};
  int i, j;
  float temp;

  for (i = 0 ; i < window - 1; i++)
    {
      for (j = 0 ; j < window - i - 1; j++)
        {
          if (sort[j] > sort[j+1])
            {
            // Swap values
            temp            = sort[j];
            sort[j]   = sort[j+1];
            sort[j+1] = temp;
            }
          }
      }
     // printArray(sort);
  return sort[2];
}

float averageDerivs(float* a) {
  float total = 0;
  for(int k=0; k<window; k++){
    delay(100);
    total = total + a[k];
  }
  float avg = total/window;
  return avg;
}

void changeTime(float avgSecondDeriv) {
  // this function will change the RTC alarm value based on the average second derivatives

  // original adaptive equation:
//  avgSecondDeriv = avgSecondDeriv * 30.48;
//  float timeInterval = ((1-avgSecondDeriv)*5) + prevTimeInterval;

  // sigmoid equation
  Serial.print(", "); Serial.println(avgSecondDeriv);
  avgSecondDeriv = avgSecondDeriv*26.41;    // used to scale feet readings to the x values of the sigmoid function
  float timeInterval = ((topLimit-bottomLimit)/(1+ pow(M_E, avgSecondDeriv-8))) +bottomLimit;
  
  if(timeInterval < bottomLimit) {
    timeInterval = bottomLimit;
  }
  else if(timeInterval > topLimit) {
    timeInterval = topLimit;
  }
  //Serial.print("time interval: ");Serial.println(timeInterval);
  skipIndicies = round(timeInterval/4);
  prevTimeInterval = timeInterval;
}

void printArray(float* a) {
  for(int k=0; k<window; k++) {
    Serial.print(a[k]);Serial.print(", ");
  }
  Serial.println();
}

void loop() {
}
