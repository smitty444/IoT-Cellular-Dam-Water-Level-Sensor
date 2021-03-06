#include <NewPing.h>

#include <SimpleKalmanFilter.h>

#define trigPin 47
#define echoPin 46

NewPing sonar(trigPin, echoPin);

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
                     ,                                                                                                                                                                                                                        
                     166.5
                    };


void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.println("*** Executing filtering.ino ***");

  //plotMedian();
  //plotMovingAverage();
  plotKalman();
  // printArray(dist);
}

void plotMedian() {
  Serial.println("Let's use the median filter!");

  const int window = 5;           // if you change this, be sure to change the arguments of ultrasonicMedian(dist[0-window-1]) in the loop()
  float dist[102];
  float tolerance = 3;      // how many feet +/- we can expect a valid reading to be within of the last reading (we dont expect it to jump 4 feet in 4 minutes)
  float filtered;

  for (int i = 0; i < window;) {
    dist[i] = values[i];
    i++;
    delay(100);
  }
  Serial.println("array initialized");

  for (int i = window; i < 102; i++) {
    Serial.print(values[i]);
    dist[i] = values[i];
    if (dist[i] > dist[i - 1] + tolerance || dist[i] < dist[i - 1] - tolerance) {                       // means the value is likely an error! apply median filter
      filtered = bubble_sort(dist[i - 5], dist[i - 4], dist[i - 3], dist[i - 2], dist[i - 1], window);          // NOTE: change the value of these if you change the window size
      // filtered = ultrasonicMedian(dist[0], dist[1], dist[2], dist[3], dist[4]);
    }
    else {
      filtered = dist[i];
    }
    //delay(200);
    Serial.print(", "); Serial.println(filtered);
    delay(100);
    dist[i] = filtered;

    delay(250);
  }
}

void plotMovingAverage() {
  Serial.println("Let's use the moving average filter!");

  const int window = 5;
  int index = 0;
  float sum = 0;
  float dist[window];
  float averaged = 0;

  for (int i = 0; i < window;) {
    dist[i] = values[i];
    sum = sum + dist[i];
    i++;
    delay(100);
  }
  Serial.println("array initialized");

  for (int i = 5; i < 102; i++) {
    sum = sum - dist[index];
    float distance = values[i];
    Serial.print(distance);
    dist[index] = distance;
    sum = sum + distance;
    index = (index + 1) % window;
    averaged = sum / window;
    Serial.print(","); Serial.println(averaged);
    delay(250);
  }
}

void plotKalman() {
  Serial.println("Let's use the Kalman filter!");

  for(int i=0; i<102; i++) {
    float distance = values[i];
    float estimate = kalman(distance);
    Serial.print(distance); Serial.print(","); Serial.println(estimate);
    delay(200);
  }
}

float kalman(float U){

  static const double R = 40;
  static const double H = 1.00;
  static double Q = 10;
  static double P = 0;
  static double U_hat = 0;
  static double K = 0;
  
  K = P*H/(H*P*H+R);
  U_hat += + K*(U-H*U_hat);
  P = (1-K*H)*P+Q;

  return U_hat;
}


float bubble_sort(float a, float b, float c, float d, float e, int window) {
  float sort[window] = {a, b, c, d, e};
  int i, j;
  float temp;

  for (i = 0 ; i < window - 1; i++)
  {
    for (j = 0 ; j < window - i - 1; j++)
    {
      if (sort[j] > sort[j + 1])
      {
        // Swap values
        temp            = sort[j];
        sort[j]   = sort[j + 1];
        sort[j + 1] = temp;
      }
    }
  }
  // printArray(sort);
  return sort[2];
}

//void printArray(float* a) {
//  Serial.println();
//  for (int k = 0; k < window; k++) {
//    Serial.print(a[k]); Serial.print(", ");
//  }
//  Serial.println();
//}

void loop() {

}
