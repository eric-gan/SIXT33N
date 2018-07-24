/*
   classify.ino

   Eric Gan, Shloak Jain, Arjun Mishra

*/

#define LEFT_MOTOR                  P2_0
#define LEFT_ENCODER                P6_1
#define RIGHT_MOTOR                 P1_5
#define RIGHT_ENCODER               P6_2

#define RUN_TIME                    (20*1000)
#define SAMPLING_INTERVAL           100
#define SAMPLE_LEN                  (RUN_TIME/SAMPLING_INTERVAL)

#define JOLT_STEPS                  2

int step_num = 0;
volatile boolean do_loop = 0; // timer signal to increment timestep

int16_t deltaArr[SAMPLE_LEN] = {0};
uint16_t lpos[SAMPLE_LEN] = {0};
uint16_t rpos[SAMPLE_LEN] = {0};
uint8_t lpwm[SAMPLE_LEN] = {0};
uint8_t rpwm[SAMPLE_LEN] = {0};

typedef struct encoder {
  int pin;
  int pos;
  bool level;
  int avg;
} encoder_t;

encoder_t left_encoder = {LEFT_ENCODER, 0, LOW, 0};
encoder_t right_encoder = {RIGHT_ENCODER, 0, LOW, 0};

/*---------------------------*/
/*      CODE BLOCK CON1      */
/*     From open_loop.ino    */
/*       with changes        */
/*---------------------------*/

float theta_left = 0.5704;
float theta_right = 0.5996;
float beta_left = 32.04;
float beta_right = 25.31;
float v_star = 29.1;

// PWM inputs to jolt the car straight
int left_jolt = 240;
int right_jolt = 200;

// Control gains
float k_left = 0.9;
float k_right = 1.05;

/*---------------------------*/
/*      CODE BLOCK CON2      */
/*---------------------------*/
float driveStraight_left_OL(float v_star) {
  return (v_star + beta_left) / theta_left;
}

float driveStraight_right_OL(float v_star) {
  return (v_star + beta_right) / theta_right;
}


float driveStraight_left(float v_star, float delta) {
  return driveStraight_left_OL(v_star) - (k_left / theta_left) * delta;
}

float driveStraight_right(float v_star, float delta) {
      return driveStraight_right_OL(v_star) + (k_right / theta_right) * delta;
}

/*---------------------------*/
/*      CODE BLOCK CON3      */
/*---------------------------*/

float delta_ss = -4;

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

void setup(void) {
  Serial.begin(38400);

  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_ENCODER, INPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_ENCODER, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);


  write_pwm(0, 0); // Turn off motors
  delay(2000); // Wait 2 seconds to put down car
  reset_blinker(); // Blink lights to indicate car is running
  setTimer(); // Set timer for timestep
}

void loop(void) {
  check_encoders();
  if (do_loop) {
    // Apply maximum input for a short time to start motors
    if (step_num < JOLT_STEPS) {
      write_pwm(left_jolt, right_jolt);
      step_num++;
    }
    // If not done running
    else if (step_num < SAMPLE_LEN) {

      // Save positions because _left_position and _right_position
      // can change in the middle of one loop.
      int left_position = left_encoder.pos;
      int right_position = right_encoder.pos;

      /*---------------------------*/
      /*      CODE BLOCK CON0      */
      /*---------------------------*/

      /*--------------------------------------*/
      /*     Add the steady-state value of    */
      /*    delta from this calculation to    */
      /*    compensate for initial turning    */
      /*--------------------------------------*/
      float delta = left_position - right_position + delta_ss;

      // Drive straight using feedback
      // Compute the needed pwm values for each wheel using delta and v_star
      int left_cur_pwm = driveStraight_left(v_star, delta);
      int right_cur_pwm = driveStraight_right(v_star, delta);
      write_pwm(left_cur_pwm, right_cur_pwm);

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/

      lpos[step_num] = left_position;
      rpos[step_num] = right_position;
      deltaArr[step_num] = delta;
      lpwm[step_num] = left_cur_pwm;
      rpwm[step_num] = right_cur_pwm;

      step_num++;
    }

    else { // When step_num has reached SAMPLE_LEN
      // Turn off motors
      write_pwm(0, 0);

      // Print out result
      Serial.println("Start");
      Serial.println("delta - left pos - right pos - left pwm - right pwm");
      for (int i = 0; i < SAMPLE_LEN; i++) {
        Serial.print(deltaArr[i]);
        Serial.print(',');
        Serial.print(lpos[i]);
        Serial.print(',');
        Serial.print(rpos[i]);
        Serial.print(',');
        Serial.print(lpwm[i]);
        Serial.print(',');
        Serial.print(rpwm[i]);
        Serial.print('\n');
      }
    }
    do_loop = 0;
  }
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void write_pwm(int pwm_left, int pwm_right) {
  analogWrite(LEFT_MOTOR, (int) min(max(0, pwm_left), 255));
  analogWrite(RIGHT_MOTOR, (int) min(max(0, pwm_right), 255));
}

void reset_blinker(void) {
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

#define AVG_DECAY_RATE              0.3
#define LOW_THRESH                  ((int) (0.2*4096))
#define HIGH_THRESH                 ((int) (0.8*4096))

void check_encoder(encoder_t* enc) {
  int new_val = analogRead(enc->pin);
  enc->avg = (int) (AVG_DECAY_RATE*enc->avg + (1 - AVG_DECAY_RATE)*new_val);
  if ((enc->level == LOW && HIGH_THRESH < enc->avg) ||
      (enc->level == HIGH && enc->avg < LOW_THRESH)) {
    enc->pos++;
    enc->level = !enc->level;
  }
}

void check_encoders(void) {
  check_encoder(&left_encoder);
  check_encoder(&right_encoder);
}

// Set timer for timestep; use A2 since A0 & A1 are used by PWM
void setTimer(void) {
  TA2CCR0 = (unsigned int) (32.768*SAMPLING_INTERVAL); // set the timer based on 32kHz clock
  TA2CCTL0 = CCIE; // enable interrupts for Timer A
  __bis_SR_register(GIE);
  TA2CTL = TASSEL_1 + MC_1 + TACLR + ID_0;
}

// ISR for timestep
#pragma vector=TIMER2_A0_VECTOR    // Timer A ISR
__interrupt void Timer2_A0_ISR(void) {
  do_loop = 1;
}
