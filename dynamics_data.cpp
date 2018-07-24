/*
 * dynamics_data.ino
 *
 * Collects dynamics data (position) with
 * some varying (bounded) input PWM
 *
 * Eric Gan, Shloak Jain, Arjun Mishra
 */

#define LEFT_MOTOR                  P2_0
#define LEFT_ENCODER                P6_1
#define RIGHT_MOTOR                 P1_5
#define RIGHT_ENCODER               P6_2

/*---------------------------*/
/*      CODE BLOCK SID1      */
/*---------------------------*/

// Parameters for sweep of whole PWM range
//#define SAMPLING_INTERVAL           500 // in ms
//#define SAMPLES_PER_PWM             1
//#define LOW_PWM                     0
//#define HIGH_PWM                    250
//#define PWM_STEP                    10

// Parameters for second sweep
 #define SAMPLING_INTERVAL           500 // in ms
 #define SAMPLES_PER_PWM             4
 #define LOW_PWM                     120// your value here!
 #define HIGH_PWM                    170// your value here!
 #define PWM_STEP                    10

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

// First half of samples are from (HIGH-STEP)->LOW_PWM
// Second half are from (LOW+STEP)->HIGH
#define NUM_PWM                     (2*(HIGH_PWM-LOW_PWM)/PWM_STEP)
#define SAMPLE_LEN                  (SAMPLES_PER_PWM*NUM_PWM)

// Arrays for position, velocity, and pwm data storage
uint16_t lpos[SAMPLE_LEN] = {0};
uint16_t rpos[SAMPLE_LEN] = {0};
uint8_t pwm[SAMPLE_LEN] = {0};

int step_num = 0;
volatile boolean do_loop = 0; // timer signal to increment timestep

int cur_pwm = HIGH_PWM;
int dir = -1;

typedef struct encoder {
  int pin;
  int pos;
  bool level;
  int avg;
} encoder_t;

encoder_t left_encoder = {LEFT_ENCODER, 0, LOW, 0};
encoder_t right_encoder = {RIGHT_ENCODER, 0, LOW, 0};

void setup(void) {
  Serial.begin(38400);

  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_ENCODER, INPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_ENCODER, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  write_pwm(0, 0); // Turn off motors
  delay(3000); // Wait 3 seconds to put down car

  // Start motors
  reset_blinker();
  write_pwm(255, 255);
  delay(500);
  write_pwm(cur_pwm, cur_pwm);
  delay(1000); // Wait before starting to record

  // Set timer for timestep
  setTimer();
}

void loop(void) {
  check_encoders();
  if (do_loop) {
    // If not done collecting data
    if (step_num < SAMPLE_LEN) {
      lpos[step_num] = left_encoder.pos;
      rpos[step_num] = right_encoder.pos;
      pwm[step_num] = cur_pwm;

      if (step_num % SAMPLES_PER_PWM == 0) {
        // Provide new input to system
        cur_pwm += dir*PWM_STEP;
        write_pwm(cur_pwm, cur_pwm);
        if (cur_pwm <= LOW_PWM || HIGH_PWM <= cur_pwm) {
          dir *= -1;
        }
      }
      step_num++;
    }
    else { // If done collecting data
      // Turn off motors
      write_pwm(0, 0);

      // Print out result
      Serial.println("Start");
      Serial.println("PWM - Left Pos - Right Pos");
      for(int i = 0; i < SAMPLE_LEN; i++){
        Serial.print(pwm[i]);
        Serial.print(',');
        Serial.print(lpos[i]);
        Serial.print(',');
        Serial.print(rpos[i]);
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
