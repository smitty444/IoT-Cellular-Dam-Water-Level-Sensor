#include <NewPing.h>

#define trigPin 47
#define echoPin 46

NewPing sonar(trigPin, echoPin);

const int window = 5;           // if you change this, be sure to change the arguments of ultrasonicMedian(dist[0-window-1]) in the loop()
float dist[window];
float tolerance = 1.5;      // how many feet +/- we can expect a valid reading to be within of the last reading (we dont expect it to jump 4 feet in 4 minutes)
float filtered;

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.println("*** Executing median_filter_tester.ino ***");
  for (int i = 0; i < window;) {
    float distance = sonar.ping_in();
    if (distance > 9) {
      distance = distance / 12;
      dist[i] = distance;
      i++;
      delay(100);
    }
    else {
      delay(50);
    }
  }
  Serial.println("array initialized");
 // printArray(dist);
}

void loop() {

  float distance = sonar.ping_in();
  if (distance > 9) {
    distance = distance / 12;
    Serial.print(distance);
    //delay(200);
    if (distance > dist[window - 1] + tolerance || distance < dist[window - 1] - tolerance) {                                     // means the value is likely an error! apply median filter
        filtered = bubble_sort(dist[0], dist[1], dist[2], dist[3], dist[4]);             // NOTE: change the value of these if you change the window size
       // filtered = ultrasonicMedian(dist[0], dist[1], dist[2], dist[3], dist[4]);
    }
    else {
      filtered = distance;
    }
    //delay(200);
    Serial.print(", "); Serial.println(filtered);
    delay(100);
    for (int i = 0 ; i < window - 1; i++) {
      dist[i] = dist[i + 1];
      //      Serial.print(dist[i]); Serial.print(", ");
      delay(10);
    }
    dist[window - 1] = filtered;
    //Serial.println(dist[window - 1]);

    delay(250);
  }
  else {
    delay(50);
  }
}

//// Swap two variables
//float temp;
//#define swap(w, z) temp = w; w = z; z = temp;
//#define sort(x, y) if(x > y) { swap(x, y); }
//
//// Median calculation
//float ultrasonicMedian(float a, float b, float c, float d, float e) {
//  sort(a, b);
//  sort(d, e);
//  sort(a, c);
//  sort(b, c);
//  sort(a, d);
//  sort(c, d);
//  sort(b, e);
//  sort(b, c);
//
//  return c;
//}

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

void printArray(float* a) {
  Serial.println();
  for (int k = 0; k < window; k++) {
    Serial.print(a[k]); Serial.print(", ");
  }
  Serial.println();
}
