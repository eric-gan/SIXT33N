/*
   classify.ino

   Eric Gan, Shloak Jain, Arjun Mishra

*/

#define MIC_INPUT                   P6_3 //6_0

#define SIZE                        2752
#define SIZE_AFTER_FILTER           172
#define ADC_TIMER_MS                0.35

/*---------------------------*/
/*      CODE BLOCK PCA1      */
/*---------------------------*/

// Enveloping and K-means constants
#define SNIPPET_SIZE                60
#define PRELENGTH                   5
#define THRESHOLD                   0.5

#define KMEANS_THRESHOLD            2
#define LOUDNESS_THRESHOLD          300   //700

/*---------------------------*/
/*      CODE BLOCK PCA2      */
/*---------------------------*/

float pca_vec1[SNIPPET_SIZE] = {-0.06716767, -0.07004317, -0.11084192, -0.14128375, -0.11422977,
       -0.1929778 , -0.24404791, -0.24548683, -0.2334107 , -0.25234989,
       -0.26370735, -0.32426893, -0.25881816, -0.26530061, -0.17449282,
       -0.15485602, -0.09910471, -0.0649278 , -0.02519573,  0.00200453,
        0.02441459,  0.05013696,  0.05493784,  0.06828052,  0.09566325,
        0.11747435,  0.11135773,  0.11452107,  0.1134278 ,  0.10687163,
        0.1131143 ,  0.11113591,  0.0974676 ,  0.09692765,  0.07849804,
        0.08114003,  0.07056205,  0.08436875,  0.08248171,  0.08568099,
        0.07784777,  0.07810262,  0.07557588,  0.06589137,  0.06186107,
        0.06406873,  0.05943707,  0.06462292,  0.07337583,  0.08455939,
        0.09133936,  0.09305491,  0.09903175,  0.09371459,  0.0931485 ,
        0.09033694,  0.08286068,  0.07403801,  0.06483635,  0.05434048};
float pca_vec2[SNIPPET_SIZE] = {0.08985958,  0.07049129,  0.09695266,  0.08985893,  0.08233977,
        0.15381225,  0.17153381,  0.18302232,  0.12093834,  0.07480875,
        0.00324929, -0.00981696, -0.08477207, -0.13002695, -0.13724364,
       -0.1788262 , -0.20226325, -0.20791719, -0.2378026 , -0.21179701,
       -0.24461989, -0.27075911, -0.21203581, -0.18246147, -0.08957707,
       -0.03698406, -0.01790242, -0.01406918, -0.01467182, -0.01311922,
        0.02117632,  0.01264703, -0.0034214 ,  0.01190202, -0.02042973,
       -0.01123385, -0.04559165, -0.05060501, -0.06546871, -0.06931807,
       -0.06784139, -0.07305958, -0.03894559, -0.06387997, -0.05676415,
       -0.04066775, -0.01503536,  0.00812812,  0.03581192,  0.09194573,
        0.14799894,  0.16823889,  0.1888229 ,  0.20300434,  0.20376323,
        0.19351041,  0.19058159,  0.18687124,  0.17343283,  0.14422564};
float mean_vec[SNIPPET_SIZE] = {0.00846215,  0.00942943,  0.01126267,  0.01457914,  0.01894316,
        0.02919552,  0.03381401,  0.03506188,  0.03407082,  0.0317931 ,
        0.03056272,  0.02999529,  0.02655939,  0.02630262,  0.02305912,
        0.02187768,  0.02056809,  0.01904702,  0.01738555,  0.01575812,
        0.01514577,  0.01588877,  0.01481642,  0.01560956,  0.01657069,
        0.01773527,  0.01678088,  0.0170932 ,  0.01713098,  0.01667172,
        0.01698472,  0.01667717,  0.01513216,  0.01553195,  0.01382387,
        0.01381617,  0.01324974,  0.01367322,  0.01489576,  0.01503096,
        0.01474543,  0.01462039,  0.01406049,  0.01389135,  0.01323334,
        0.01237083,  0.01241218,  0.01185292,  0.01160649,  0.01138331,
        0.01167739,  0.0115918 ,  0.01112556,  0.0108163 ,  0.01041615,
        0.01009481,  0.00977564,  0.00887367,  0.00817444,  0.00729103};
float smallestDist = 1e7;
float centroid1[2] = {-0.11321782,  0.01763347} ; // tac
float centroid2[2] = { 0.01477732, -0.01038442}; // flamingo
float centroid3[2] = { 0.01257968, -0.04428253};  // inspire
float centroid4[2] =  { 0.06420241,  0.03986789};  // crescendo
float* centroids[4] = {
  (float *) &centroid1, (float *) &centroid2,
  (float *) &centroid3, (float *) &centroid4
};

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

float result[SNIPPET_SIZE] = {0};
float proj1 = 0;
float proj2 = 0;

// Data array and index pointer
int re[SIZE] = {0};
volatile int re_pointer = 0;

/*---------------------------*/
/*       Norm functions      */
/*---------------------------*/

// Compute the L2 norm of (dim1, dim2) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        centroid: size-2 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm(float dim1, float dim2, float* centroid) {
  return sqrt(pow(dim1 - centroid[0], 2) + pow(dim2 - centroid[1], 2));
}



void setup(void) {
  Serial.begin(38400);

  pinMode(MIC_INPUT, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  re_pointer = 0;
  reset_blinker();
  setTimer();
}

void loop(void) {
  if (re_pointer == SIZE) {
    digitalWrite(RED_LED, LOW);
    Serial.println("entered loop");
    // Apply enveloping function and get snippet with speech.
    // Do classification only if loud enough.
    if (envelope(re, result)) {
      Serial.println("loud enough");

      // Reset projection result variables declared above
      proj1 = 0;
      proj2 = 0;

      /*---------------------------*/
      /*      CODE BLOCK PCA3      */
      /*---------------------------*/

      // Perform principal component projection
      for (int i = 0; i < SNIPPET_SIZE; i++) {
        proj1 += (result[i] - mean_vec[i]) * pca_vec1[i];
        proj2 += (result[i] - mean_vec[i]) * pca_vec2[i];
      }
      Serial.print(proj1);
      Serial.print(" , ");
      Serial.println(proj2);
      float dists[4] = {0};
      for (int i = 0; i < 4; i++) {
        dists[i] = l2_norm(proj1, proj2, centroids[i]);
      }
      int smallest = 0;
      smallestDist = dists[0];
      //Serial.println(dists[0]);
      for (int i = 1; i < 4; i++) {
        if ( dists[i] < smallestDist) {
          smallest = i;
          smallestDist = dists[i];
        }
      }
      Serial.println(smallestDist);

      // Classification - Hint: use the function l2_norm defined above
      // YOUR CODE HERE
      
      // Check against KMEANS_THRESHOLD and print result over serial

      if (smallestDist < KMEANS_THRESHOLD) {
        Serial.println(smallest);
      }


      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }

    delay(2000);
    re_pointer = 0;
  }
}

// Enveloping function with thresholding and normalizing,
// returns snippet of interest (containing speech)
bool envelope(int* data, float* data_out) {
  int32_t avg = 0;
  float maximum = 0;
  int32_t total = 0;
  int block;

  // Apply enveloping filter while finding maximum value
  for (block = 0; block < SIZE_AFTER_FILTER; block++) {
    avg = 0;
    for (int i = 0; i < 16; i++) {
      avg += data[i + block * 16];
    }
    avg = avg >> 4;
    data[block] = abs(data[block * 16] - avg);
    for (int i = 1; i < 16; i++) {
      data[block] += abs(data[i + block * 16] - avg);
    }
    if (data[block] > maximum) {
      maximum = data[block];
    }
  }

  // If not loud enough, return false
  if (maximum < LOUDNESS_THRESHOLD) {
    return false;
  }

  // Determine threshold
  float thres = THRESHOLD * maximum;

  // Figure out when interesting snippet starts and write to data_out
  block = PRELENGTH;
  while (data[block++] < thres);
  if (block > SIZE_AFTER_FILTER - SNIPPET_SIZE) {
    block = SIZE_AFTER_FILTER - SNIPPET_SIZE;
  }
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data[block - PRELENGTH + i];
    total += data_out[i];
  }

  // Normalize data_out
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data_out[i] / total;
  }

  return true;
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void reset_blinker(void) {
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(RED_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, LOW);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(GREEN_LED, LOW);
}

/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/

// ISR for timestep
#pragma vector=TIMER2_A0_VECTOR    // Timer A ISR
__interrupt void Timer2_A0_ISR(void) {
  if (re_pointer < SIZE) {
    digitalWrite(RED_LED, HIGH);
    re[re_pointer] = (analogRead(MIC_INPUT) >> 4) - 128;
    re_pointer += 1;
  }
}

// Set timer for timestep; use A2 since A0 & A1 are used by PWM
void setTimer(void) {
  // Set the timer based on 25MHz clock
  TA2CCR0 = (unsigned int) (25000 * ADC_TIMER_MS);
  TA2CCTL0 = CCIE;
  __bis_SR_register(GIE);
  TA2CTL = TASSEL_2 + MC_1 + TACLR + ID_0;
}
