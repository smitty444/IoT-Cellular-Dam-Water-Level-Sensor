/*
 * Resources:
 * 
 * Median filter
 *  - https://renegaderobotics.org/filtering-sensor-data/
 *  - https://dlacko.org/blog/2016/01/24/remove-impulse-noise-from-ultrasonic/#:~:text=If%20you%20want%20to%20smooth%20the%20sensor%20data,related%20to%20the%20topic.%20Conclusion%20and%20future%20work
 *  
 * Moving average filter
 *  - https://maker.pro/arduino/tutorial/how-to-clean-up-noisy-sensor-data-with-a-moving-average-filter
 *  
 * Kalman filter
 *  - https://github.com/rizkymille/ultrasonic-hc-sr04-kalman-filter/blob/master/hc-sr04_kalman_filter/hc-sr04_kalman_filter.ino
 */
 
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


void setup() {
  Serial.begin(9600);
  Serial.println("*** Executing filtering.ino ***");

// uncomment which filter to run ------------------------------

  //plotDerivs();         // used to find a good threshold for the median filter
  //plotMedian();
  //plotMovingAverage();
  //plotKalman();
  plotLastReplacement();
  
}

void plotDerivs() {
  for(int i=1; i<102; i++) {
    Serial.println(values[i] - values[i-1]);
  }
}

void plotLastReplacement() {
  Serial.println("Let's use the last replacement filter!");
  
  float dist[102];          // you can nix the sliding window array since this is not trying to save memory
  float tolerance = 3;      // how many feet +/- we can expect a valid reading to be within of the last reading (we dont expect it to jump 4 feet in 4 minutes)
  float filtered; 
  int window = 5;

  // initialize the distance array with five readings
  for (int i = 0; i < window;) {
    dist[i] = values[i];
    i++;
    delay(100);
  }
  Serial.println("array initialized");

  // cycle through the data points and apply median filter when necessary
  for (int i = window; i < 102; i++) {
    Serial.print(values[i]);
    dist[i] = values[i];
    // check if the value is within the tolerance of the previous value
    if (dist[i] > dist[i - 1] + tolerance || dist[i] < dist[i - 1] - tolerance) {    // means the value is likely an error! apply median filter
      filtered = dist[i - 1];     // replace the reading with the last reading
    }
    else {
      filtered = dist[i];
    }
    Serial.print(", "); Serial.println(filtered);
    delay(100);
    // replace the newest index of the distance array with the filtered value
    dist[i] = filtered;

    delay(250);
  }
}

void plotMedian() {
  Serial.println("Let's use the median filter!");

  float dist[102];          // you can nix the sliding window array since this is not trying to save memory
  float tolerance = 3;      // how many feet +/- we can expect a valid reading to be within of the last reading (we dont expect it to jump 4 feet in 4 minutes)
  float filtered; 
  int window = 5;

  // initialize the distance array with five readings
  for (int i = 0; i < window;) {
    dist[i] = values[i];
    i++;
    delay(100);
  }
  Serial.println("array initialized");

  // cycle through the data points and apply median filter when necessary
  for (int i = window; i < 102; i++) {
    Serial.print(values[i]);
    dist[i] = values[i];
    // check if the value is within the tolerance of the previous value
    if (dist[i] > dist[i - 1] + tolerance || dist[i] < dist[i - 1] - tolerance) {                          // means the value is likely an error! apply median filter
      filtered = bubble_sort(dist[i - 5], dist[i - 4], dist[i - 3], dist[i - 2], dist[i - 1], window);     // take the last five distance values to find the median of (rather than the whole 102 element array)
    }
    else {
      filtered = dist[i];
    }
    Serial.print(", "); Serial.println(filtered);
    delay(100);
    // replace the newest index of the distance array with the filtered value
    dist[i] = filtered;

    delay(250);
  }
}

void plotMovingAverage() {
  Serial.println("Let's use the moving average filter!");

  // initialize variables
  const int window = 5;
  int index = 0;
  float sum = 0;
  float dist[window];       // sliding window array
  float averaged = 0;

  // populate the array and determine the sum with the first five values 
  for (int i = 0; i < window;) {
    dist[i] = values[i];
    sum = sum + dist[i];
    i++;
    delay(100);
  }
  Serial.println("array initialized");

  // iterate through the rest of the data and apply the moving average formula
  for (int i = 5; i < 102; i++) {
    sum = sum - dist[index];            // remove the oldest value from the sum 
    float distance = values[i];         // determine the new value
    Serial.print(distance);             
    dist[index] = distance;             // integrate the new value into the sliding window array
    sum = sum + distance;               // update new sum
    index = (index + 1) % window;       // iterate to the next index of the sliding window array
    averaged = sum / window;            // calculate the average
    
    Serial.print(","); Serial.println(averaged);
    delay(250);
  }
}

void plotKalman() {
  Serial.println("Let's use the Kalman filter!");

  // iterate through the values beginning at the first index
  for(int i=0; i<102; i++) {
    float distance = values[i];
    float estimate = kalman(distance);    // use the Kalman filter to find the filtered distance
    Serial.print(distance); Serial.print(","); Serial.println(estimate);
    delay(200);
  }
}

float kalman(float U) {
  
  // initialize Kalman constants
  static const double R = 40;
  static const double H = 1.00;
  static double Q = 10;
  static double P = 0;
  static double U_hat = 0;
  static double K = 0;

  // apply formula
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
        temp = sort[j];
        sort[j] = sort[j + 1];
        sort[j + 1] = temp;
      }
    }
  }
  return sort[2];
}


// runs nothing
void loop() {

}
