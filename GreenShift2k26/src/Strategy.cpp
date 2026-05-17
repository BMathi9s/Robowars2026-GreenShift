#include "Strategy.h"
#include "Motors.h"
#include <Arduino.h>
#include <PID_v1.h> // Brett's PID Library

// --- Motor Test State Machine Enums ---
enum MotorTestState {
    TEST_WAIT_START,
    TEST_RAMP_FORWARD,
    TEST_FORWARD,
    TEST_EDGE_PAUSE,
    TEST_RAMP_REVERSE,
    TEST_REVERSE
};
enum StandardState  { STD_WAIT_START, STD_CAPTURE_HEADING, STD_FORWARD_PID, STD_EDGE_PAUSE, STD_RAMP_REVERSE, STD_REVERSE };

// --- PD Punch State Machine Enums & Variables ---
enum PDPunchState {
    PUNCH_WAIT_START,
    PUNCH_CAPTURE_HEADING,
    PUNCH_HUNT,
    PUNCH_ATTACK,
    PUNCH_COOLDOWN,
    PUNCH_EDGE_REVERSE,
    PUNCH_EDGE_SPIN
};

static PDPunchState currentPunchState = PUNCH_WAIT_START;
static unsigned long punchTimer = 0;
static float punchBaseHeading = 0.0;
static float spinTargetHeading = 0.0;
static unsigned long lastTrackTime = 0;
// punch



// --- Static State Variables ---
static MotorTestState currentTestState = TEST_WAIT_START;
static StandardState currentStdState = STD_WAIT_START;
static unsigned long stateTimer = 0;

// --- PID Setup ---
static double pidSetpoint = 0; // We always want 0 error
static double pidInput = 0;    // This will be our shortest-path error
static double pidOutput = 0;   // The steering correction value
// Minimal Tuning PD Profile (Drop Ki entirely for sumo collisions)
// --- TUNING VARIABLES ---
static double Kp = 28.0;  // Increase if it feels lazy, stop when it wiggles 12
static double Ki = 0.0;  // Keep at 0 for sumo
static double Kd = 1.0;  // Increase to dampen the wiggle  0.2



// ====================================================================
// --- SLOW & RELIABLE MODE ENUMS & VARIABLES ---
// ====================================================================
enum SlowRelState {
    SLOW_WAIT_START,
    SLOW_CAPTURE_HEADING,
    SLOW_HUNT,
    SLOW_ATTACK,
    SLOW_COOLDOWN,
    SLOW_EDGE_REVERSE,
    SLOW_EDGE_SPIN
};

static SlowRelState currentSlowState = SLOW_WAIT_START;
static unsigned long slowTimer = 0;
static float slowBaseHeading = 0.0;
static unsigned long lastSlowTrackTime = 0;

// --- DEDICATED SLOW PID SETUP ---
static double slowPidSetpoint = 0; 
static double slowPidInput = 0;    
static double slowPidOutput = 0;   

// TUNING VARIABLES FOR 20% SPEED:
// At lower speeds, you often need a slightly HIGHER Kp to generate enough 
// torque to actually turn the robot, but keep Kd low to prevent jitters.
static double sKp = 20.0;  // Proportional: Adjust up if it doesn't track fast enough
static double sKi = 0.0;   // Keep 0
static double sKd = 0.2;   // Derivative: Dampens oscillation

static PID slowHeadingPID(&slowPidInput, &slowPidOutput, &slowPidSetpoint, sKp, sKi, sKd, DIRECT);
static bool slowPidInitialized = false;

// --- EASY TUNE SPEED TARGETS (0.0 to 1.0) ---
const float SLOW_CRUISE_POWER = 0.20;  // 20% Speed for hunting
const float SLOW_ATTACK_POWER = 0.35;  // 35% Speed for pushing (safe, no flip)
const float SLOW_REVERSE_POWER = 0.25; // 25% Speed for backing away from edges
const int   MIN_MOTOR_PWM = 60;        // Minimum PWM required to physically move the robot
// ====================================================================
// --- SLOW & RELIABLE MODE ENUMS & VARIABLES ---
// ====================================================================
enum SlowAttackState {
    SA_WAIT_START,
    SA_CAPTURE_HEADING,
    SA_HUNT,
    SA_ATTACK,
    SA_EDGE_REVERSE,
    SA_EDGE_SPIN
};

static SlowAttackState currentSaState = SA_WAIT_START;
static unsigned long saTimer = 0;
static float saBaseHeading = 0.0;
static unsigned long lastSaTrackTime = 0;
static unsigned long lastPunchTime = 0; // Tracks the 6-second cooldown

// Dedicated PID for Slow Attack cruising (Exact copy of slow_reliable)
static double saPidSetpoint = 0; 
static double saPidInput = 0;    
static double saPidOutput = 0;   

static double saKp = 20.0;  
static double saKi = 0.0;   
static double saKd = 0.2;   

static PID saHeadingPID(&saPidInput, &saPidOutput, &saPidSetpoint, saKp, saKi, saKd, DIRECT);
static bool saPidInitialized = false;

// --- STRICT SPEED LIMITS ---
const float SA_CRUISE_POWER = 0.40;  // STRICT 20% Speed for all movement
const float SA_PUNCH_POWER = 0.85;   // Impulse punch power
const float SA_REVERSE_POWER = 0.35; // Safe edge escape
// ====================================================================
// --- SLOW ATTACK MODE ENUMS & VARIABLES ---
// ====================================================================



static PID headingPID(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, DIRECT);
static bool pidInitialized = false;
static float targetHeading = 0.0;

void Strategy::execute(SharedSensorData &snap, bool isPDControl, bool isDebugMotor,bool ispd_punch, bool isslow_reliable, bool isslow_attack) {
    if (isDebugMotor) {
        runMotorDebugTest(snap); 
    } else if (isPDControl) {
        runStandardProtocol(snap);
    } else if (ispd_punch) {
        runPDPUNCH(snap);
    } else if (isslow_reliable) {
        slow_reliable(snap);
    } else if (isslow_attack) {
        slow_attack(snap);
    }

}


void Strategy::slow_attack(SharedSensorData &snap) {
    unsigned long currentMillis = millis();

    // 1. One-Time PID Initialization
    if (!saPidInitialized) {
        saHeadingPID.SetMode(AUTOMATIC);
        saHeadingPID.SetOutputLimits(-150, 150); // Same limits as slow_reliable
        saHeadingPID.SetSampleTime(2);
        saPidInitialized = true;
    }

    // 2. Edge Detection Override (Highest Priority)
    bool edgeDetected = snap.edgeFL1 || snap.edgeFL2 || snap.edgeFR1 || snap.edgeFR2;
    if (edgeDetected && currentSaState != SA_EDGE_REVERSE && currentSaState != SA_EDGE_SPIN) {
        currentSaState = SA_EDGE_REVERSE;
        saTimer = currentMillis;
        saBaseHeading = snap.yaw; // Lock heading for straight reverse
    }

    // 3. The Slow Attack State Machine
    switch(currentSaState) {
        case SA_WAIT_START:
            Motors::setSpeeds(0, 0);
            if (snap.matchActiveFlag) {
                currentSaState = SA_CAPTURE_HEADING;
            }
            break;

        case SA_CAPTURE_HEADING:
            saBaseHeading = snap.yaw; 
            currentSaState = SA_HUNT;
            break;

        case SA_HUNT: {
            if (!snap.matchActiveFlag) { currentSaState = SA_WAIT_START; break; }

            saHeadingPID.SetTunings(saKp, saKi, saKd);

            bool leftDet = snap.oppFL;   
            bool centerDet = snap.oppFC;
            bool rightDet = snap.oppFR;

            // Count how many sensors currently see the opponent
            int sensorsActive = leftDet + centerDet + rightDet;

            // --- 1. THE 200ms PUNCH TRIGGER ---
            // If 2 or 3 sensors see the enemy AND 6 seconds have passed since the last punch
            if (sensorsActive >= 2 && (currentMillis - lastPunchTime >= 6000)) {
                currentSaState = SA_ATTACK;
                saTimer = currentMillis;
                break;
            }

            // --- 2. EXACT SLOW_RELIABLE TRACKING ---
            if (currentMillis - lastSaTrackTime >= 25) {
                if (leftDet && !rightDet) {
                    saBaseHeading += 15.0; 
                    lastSaTrackTime = currentMillis;
                } else if (rightDet && !leftDet) {
                    saBaseHeading -= 15.0; 
                    lastSaTrackTime = currentMillis;
                }
            }

            if (saBaseHeading >= 360.0) saBaseHeading -= 360.0;
            if (saBaseHeading < 0.0) saBaseHeading += 360.0;

            double error = saBaseHeading - snap.yaw;
            if (error > 180.0) error -= 360.0;
            else if (error < -180.0) error += 360.0;

            saPidInput = -error; 
            saHeadingPID.Compute();

            int basePwm = max((int)(255 * SA_CRUISE_POWER), MIN_MOTOR_PWM);
            
            // Allow negative output to reverse inner wheel slightly for agile turns
            int leftPwr = constrain(basePwm + saPidOutput, -150, 255);
            int rightPwr = constrain(basePwm - saPidOutput, -150, 255);

            Motors::setSpeeds(leftPwr, rightPwr);
            break;
        }

        case SA_ATTACK: {
            // Unrestricted impulse payload 
            int punchPwm = 255 * SA_PUNCH_POWER;
            Motors::setSpeeds(punchPwm, punchPwm);
            
            // Hold exactly 200ms
            if (currentMillis - saTimer >= 200) {
                lastPunchTime = currentMillis; // Start the 6-second cooldown clock
                currentSaState = SA_CAPTURE_HEADING; // Immediately return to 20% tracking
            }
            
            if (!snap.matchActiveFlag) currentSaState = SA_WAIT_START;
            break;
        }

        case SA_EDGE_REVERSE: {
            saHeadingPID.SetTunings(saKp * 0.8, saKi, saKd); 

            double error = saBaseHeading - snap.yaw;
            if (error > 180.0) error -= 360.0;
            else if (error < -180.0) error += 360.0;

            saPidInput = -error; 
            saHeadingPID.Compute();

            int revPwm = -(max((int)(255 * SA_REVERSE_POWER), MIN_MOTOR_PWM)); 

            int leftPwr = constrain(revPwm + saPidOutput, -255, 0);
            int rightPwr = constrain(revPwm - saPidOutput, -255, 0);

            Motors::setSpeeds(leftPwr, rightPwr);

            if (currentMillis - saTimer >= 450) { 
                currentSaState = SA_EDGE_SPIN;
                saTimer = currentMillis;
                saBaseHeading = snap.yaw + 180.0;
                if (saBaseHeading >= 360.0) saBaseHeading -= 360.0;
            }
            
            if (!snap.matchActiveFlag) currentSaState = SA_WAIT_START;
            break;
        }

        case SA_EDGE_SPIN: {
            saHeadingPID.SetTunings(saKp, saKi, saKd);

            double error = saBaseHeading - snap.yaw;
            if (error > 180.0) error -= 360.0;
            else if (error < -180.0) error += 360.0;

            saPidInput = -error; 
            saHeadingPID.Compute();

            int spinEffort = saPidOutput;
            
            const int pivotMinPwm = MIN_MOTOR_PWM + 15; 
            if (spinEffort > 0 && spinEffort < pivotMinPwm) spinEffort = pivotMinPwm;
            if (spinEffort < 0 && spinEffort > -pivotMinPwm) spinEffort = -pivotMinPwm;

            int maxSpinSpeed = max((int)(255 * SA_CRUISE_POWER * 1.5), pivotMinPwm); 
            spinEffort = constrain(spinEffort, -maxSpinSpeed, maxSpinSpeed);
            
            Motors::setSpeeds(spinEffort, -spinEffort); 

            if (abs(error) < 5.0 || (currentMillis - saTimer > 1000)) { 
                currentSaState = SA_CAPTURE_HEADING;
            }

            if (!snap.matchActiveFlag) currentSaState = SA_WAIT_START;
            break;
        }
    }
}




void Strategy::slow_reliable(SharedSensorData &snap) {
    unsigned long currentMillis = millis();

    // 1. One-Time PID Initialization
    if (!slowPidInitialized) {
        slowHeadingPID.SetMode(AUTOMATIC);
        // Limit PID output so it doesn't jerk the robot wildly at low speeds
        slowHeadingPID.SetOutputLimits(-150, 150); 
        slowHeadingPID.SetSampleTime(2);
        slowPidInitialized = true;
    }

    // 2. Edge Detection Override (Highest Priority)
    bool edgeDetected = snap.edgeFL1 || snap.edgeFL2 || snap.edgeFR1 || snap.edgeFR2;
    if (edgeDetected && currentSlowState != SLOW_EDGE_REVERSE && currentSlowState != SLOW_EDGE_SPIN) {
        currentSlowState = SLOW_EDGE_REVERSE;
        slowTimer = currentMillis;
        slowBaseHeading = snap.yaw; // Lock heading for a perfectly straight reverse
    }

    // 3. The Slow & Reliable State Machine
    switch(currentSlowState) {
        case SLOW_WAIT_START:
            Motors::setSpeeds(0, 0);
            if (snap.matchActiveFlag) {
                currentSlowState = SLOW_CAPTURE_HEADING;
            }
            break;

        case SLOW_CAPTURE_HEADING:
            slowBaseHeading = snap.yaw; 
            currentSlowState = SLOW_HUNT;
            break;

        case SLOW_HUNT: {
            if (!snap.matchActiveFlag) { currentSlowState = SLOW_WAIT_START; break; }

            slowHeadingPID.SetTunings(sKp, sKi, sKd); 

            bool leftDet = snap.oppFL;   
            bool centerDet = snap.oppFC;
            bool rightDet = snap.oppFR;

            // --- 1. AGGRESSIVE ATTACK TRIGGER ---
            // Don't wait for all 3 sensors. If they are in the center, or triggering both sides, strike immediately.
            if (centerDet || (leftDet && rightDet)) {
                currentSlowState = SLOW_ATTACK;
                slowTimer = currentMillis;
                break;
            }

            // --- 2. FASTER TARGET TRACKING ---
            // Decreased the delay to 25ms and increased the jump to 15 degrees.
            // This whips the target heading around much faster, forcing the PID to fight harder.
            if (currentMillis - lastSlowTrackTime >= 25) {
                if (leftDet && !rightDet) {
                    slowBaseHeading += 15.0; 
                    lastSlowTrackTime = currentMillis;
                } else if (rightDet && !leftDet) {
                    slowBaseHeading -= 15.0; 
                    lastSlowTrackTime = currentMillis;
                }
            }

            if (slowBaseHeading >= 360.0) slowBaseHeading -= 360.0;
            if (slowBaseHeading < 0.0) slowBaseHeading += 360.0;

            // --- PD MOVEMENT ---
            double error = slowBaseHeading - snap.yaw;
            if (error > 180.0) error -= 360.0;
            else if (error < -180.0) error += 360.0;

            slowPidInput = -error; 
            slowHeadingPID.Compute();

            int basePwm = max((int)(255 * SLOW_CRUISE_POWER), MIN_MOTOR_PWM);
            
            // --- 3. AGILE STEERING (NEGATIVE PWM ALLOWED) ---
            // By constraining to -150 instead of 0, the PID is allowed to reverse the inner wheel 
            // slightly to actively pull the robot into a sharp, aggressive turn while hunting.
            int leftPwr = constrain(basePwm + slowPidOutput, -150, 255);
            int rightPwr = constrain(basePwm - slowPidOutput, -150, 255);

            Motors::setSpeeds(leftPwr, rightPwr);
            break;
        }

        case SLOW_ATTACK: {
            // Steady, reliable push. No massive ramp-up, just a solid torque push.
            int attackPwm = max((int)(255 * SLOW_ATTACK_POWER), MIN_MOTOR_PWM);
            Motors::setSpeeds(attackPwm, attackPwm);
            
            // Push for 1 second max, then evaluate
            if (currentMillis - slowTimer >= 1000) {
                currentSlowState = SLOW_COOLDOWN; 
                slowTimer = currentMillis; 
            }
            
            if (!snap.matchActiveFlag) currentSlowState = SLOW_WAIT_START;
            break;
        }

        case SLOW_COOLDOWN: {
            // Briefly pause or drive forward at cruise speed to reassess sensors
            int basePwm = max((int)(255 * SLOW_CRUISE_POWER), MIN_MOTOR_PWM);
            Motors::setSpeeds(basePwm, basePwm);

            if (currentMillis - slowTimer >= 300) {
                currentSlowState = SLOW_CAPTURE_HEADING; 
            }

            if (!snap.matchActiveFlag) currentSlowState = SLOW_WAIT_START;
            break;
        }

        case SLOW_EDGE_REVERSE: {
            // Soft reverse to prevent wheel slip and maintain grip
            slowHeadingPID.SetTunings(sKp * 0.8, sKi, sKd); 

            double error = slowBaseHeading - snap.yaw;
            if (error > 180.0) error -= 360.0;
            else if (error < -180.0) error += 360.0;

            slowPidInput = -error; 
            slowHeadingPID.Compute();

            int revPwm = -(max((int)(255 * SLOW_REVERSE_POWER), MIN_MOTOR_PWM)); 

            int leftPwr = constrain(revPwm + slowPidOutput, -255, 0);
            int rightPwr = constrain(revPwm - slowPidOutput, -255, 0);

            Motors::setSpeeds(leftPwr, rightPwr);

            // Reverse for 450ms
            if (currentMillis - slowTimer >= 450) { 
                currentSlowState = SLOW_EDGE_SPIN;
                slowTimer = currentMillis;
                slowBaseHeading = snap.yaw + 180.0;
                if (slowBaseHeading >= 360.0) slowBaseHeading -= 360.0;
            }
            
            if (!snap.matchActiveFlag) currentSlowState = SLOW_WAIT_START;
            break;
        }

        case SLOW_EDGE_SPIN: {
            // Gentle but firm pivot
            slowHeadingPID.SetTunings(sKp, sKi, sKd);

            double error = slowBaseHeading - snap.yaw;
            if (error > 180.0) error -= 360.0;
            else if (error < -180.0) error += 360.0;

            slowPidInput = -error; 
            slowHeadingPID.Compute();

            int spinEffort = slowPidOutput;
            
            // Anti-Stall Minimum Power for pivoting
            const int pivotMinPwm = MIN_MOTOR_PWM + 15; // Spinning usually takes slightly more effort
            if (spinEffort > 0 && spinEffort < pivotMinPwm) spinEffort = pivotMinPwm;
            if (spinEffort < 0 && spinEffort > -pivotMinPwm) spinEffort = -pivotMinPwm;

            int maxSpinSpeed = max((int)(255 * SLOW_CRUISE_POWER * 1.5), pivotMinPwm); 
            spinEffort = constrain(spinEffort, -maxSpinSpeed, maxSpinSpeed);
            
            Motors::setSpeeds(spinEffort, -spinEffort); 

            if (abs(error) < 5.0 || (currentMillis - slowTimer > 1000)) { // 1 sec timeout fail-safe
                currentSlowState = SLOW_CAPTURE_HEADING;
            }

            if (!snap.matchActiveFlag) currentSlowState = SLOW_WAIT_START;
            break;
        }
    }
}




void Strategy::runPDPUNCH(SharedSensorData &snap) {

    
    unsigned long currentMillis = millis();

    if (!pidInitialized) {
        headingPID.SetMode(AUTOMATIC);
        headingPID.SetOutputLimits(-255, 255); // Let the individual states constrain the power
        headingPID.SetSampleTime(2);
        pidInitialized = true;
    }

    // 1. Edge Detection Override (Highest Priority)
    bool edgeDetected = snap.edgeFL1 || snap.edgeFL2 || snap.edgeFR1 || snap.edgeFR2;
    if (edgeDetected && currentPunchState != PUNCH_EDGE_REVERSE && currentPunchState != PUNCH_EDGE_SPIN) {
        currentPunchState = PUNCH_EDGE_REVERSE;
        punchTimer = currentMillis;
        punchBaseHeading = snap.yaw; // Lock heading for a perfectly straight reverse
    }

    // 2. The Refined State Machine
    switch(currentPunchState) {
        case PUNCH_WAIT_START:
            Motors::setSpeeds(0, 0); //[cite: 3]
            if (snap.matchActiveFlag) {
                currentPunchState = PUNCH_CAPTURE_HEADING;
            }
            break;

        case PUNCH_CAPTURE_HEADING:
            punchBaseHeading = snap.yaw; 
            currentPunchState = PUNCH_HUNT;
            break;

        case PUNCH_HUNT: {
            if (!snap.matchActiveFlag) { currentPunchState = PUNCH_WAIT_START; break; }

            // 1. Crank the Kp up from 12.0 to 18.0 for a snappier turn response
            headingPID.SetTunings(18.0, 0.0, 0.2); 

            bool leftDet = snap.oppFL;   
            bool centerDet = snap.oppFC;
            bool rightDet = snap.oppFR;

            // --- THE PRECISION PUNCH ---
            if (leftDet && centerDet && rightDet) {
                currentPunchState = PUNCH_ATTACK;
                punchTimer = currentMillis;
                break;
            }

            // --- AGGRESSIVE TRACKING ---
            // 2. Reduce the delay to 30ms and triple the angle jump to 15 degrees
            if (currentMillis - lastTrackTime >= 30) {
                if (leftDet && !rightDet) {
                    punchBaseHeading += 15.0; // Whip the target heading Left
                    lastTrackTime = currentMillis;
                } else if (rightDet && !leftDet) {
                    punchBaseHeading -= 15.0; // Whip the target heading Right
                    lastTrackTime = currentMillis;
                }
            }

            // Normalize the base heading to keep it within 0-360 range
            if (punchBaseHeading >= 360.0) punchBaseHeading -= 360.0;
            if (punchBaseHeading < 0.0) punchBaseHeading += 360.0;

            // --- PD MOVEMENT ---
            double error = punchBaseHeading - snap.yaw;
            if (error > 180.0) error -= 360.0;
            else if (error < -180.0) error += 360.0;

            pidInput = -error; 
            headingPID.Compute();

            int basePower = 255 * 0.50; // Cruise at 50%
            int leftPwr = constrain(basePower + pidOutput, 0, 255);
            int rightPwr = constrain(basePower - pidOutput, 0, 255);

            Motors::setSpeeds(leftPwr, rightPwr);
            break;
        }

        case PUNCH_ATTACK: {
            long elapsed = currentMillis - punchTimer;
            const int rampTime = 150;       
            const int totalPunchTime = 500; 
            
            // --- 1. CLEAN POWER REDUCTION ---
            // Adjust this value (0-255) to tune your maximum punch output
            const int maxPunchPower = 200; 
            
            if (elapsed <= rampTime) {
                int punchPwr = map(elapsed, 0, rampTime, 128, maxPunchPower);
                Motors::setSpeeds(punchPwr, punchPwr);
            } else if (elapsed <= totalPunchTime) {
                Motors::setSpeeds(maxPunchPower, maxPunchPower);
            } else {
                currentPunchState = PUNCH_COOLDOWN; 
                punchTimer = currentMillis; 
            }
            
            if (!snap.matchActiveFlag) currentPunchState = PUNCH_WAIT_START;
            break;
        }

        case PUNCH_COOLDOWN: {
            // Drop to a stable 50% cruise to maintain pressure without draining the capacitor
            int basePower = 255 * 0.50;
            
            // Keep using the PID to stay straight during the cooldown so we don't veer off
            double error = punchBaseHeading - snap.yaw;
            if (error > 180.0) error -= 360.0;
            else if (error < -180.0) error += 360.0;

            pidInput = -error; 
            headingPID.Compute();

            int leftPwr = constrain(basePower + pidOutput, 0, 255);
            int rightPwr = constrain(basePower - pidOutput, 0, 255);

            Motors::setSpeeds(leftPwr, rightPwr);

            // Mandatory 400ms cooldown before we are allowed to punch again
            if (currentMillis - punchTimer >= 400) {
                currentPunchState = PUNCH_CAPTURE_HEADING; 
            }

            if (!snap.matchActiveFlag) currentPunchState = PUNCH_WAIT_START;
            break;
        }

        case PUNCH_EDGE_REVERSE: {
            headingPID.SetTunings(12.0, 0.0, 0.2); 

            double error = punchBaseHeading - snap.yaw;
            if (error > 180.0) error -= 360.0;
            else if (error < -180.0) error += 360.0;

            pidInput = -error; 
            headingPID.Compute();

            int revPower = -(255 * 0.50); 

            // --- 2. STRAIGHT REVERSING FIX ---
            // Signs are swapped: Right wheel must pull backward faster to swing the front right.
            int leftPwr = constrain(revPower + pidOutput, -255, 0);
            int rightPwr = constrain(revPower - pidOutput, -255, 0);

            Motors::setSpeeds(leftPwr, rightPwr);

            // --- 3. EDGE FALLING FIX ---
            // Shorten the reverse time to 350ms to keep it safely inside the arena.
            if (currentMillis - punchTimer >= 600) { 
                currentPunchState = PUNCH_EDGE_SPIN;

                punchTimer = currentMillis;
                punchBaseHeading = snap.yaw + 180.0;
                if (punchBaseHeading >= 360.0) punchBaseHeading -= 360.0;
            }
            
            if (!snap.matchActiveFlag) currentPunchState = PUNCH_WAIT_START;
            break;
        }

    case PUNCH_EDGE_SPIN: {
            // Apply 70% Power PID Tunings for a snappy, stable pivot
            headingPID.SetTunings(15.0, 0.0, 0.5);

            double error = punchBaseHeading - snap.yaw;
            if (error > 180.0) error -= 360.0;
            else if (error < -180.0) error += 360.0;

            pidInput = -error; 
            headingPID.Compute();

            // Feed the PID output directly into opposing wheels to spin in place
            int spinEffort = pidOutput;
            
            // --- CRITICAL FIX: Anti-Stall Minimum Power ---
            // If the PID wants to turn, ensure it applies at least enough power to break friction
            const int minSpinPower = 90; // ~35% power to overcome magnets
            if (spinEffort > 0 && spinEffort < minSpinPower) spinEffort = minSpinPower;
            if (spinEffort < 0 && spinEffort > -minSpinPower) spinEffort = -minSpinPower;


            long elapsedSpin = currentMillis - punchTimer;
            const int spinRampTime = 200; // Ramp up torque over 200ms
            const int maxSpinSpeed = 135; // Lower max top speed to ~53% (down from 178)
            
            // Dynamically calculate the maximum allowed power based on how long we've been spinning
            int currentMaxSpin = maxSpinSpeed;
            if (elapsedSpin < spinRampTime) {
                currentMaxSpin = map(elapsedSpin, 0, spinRampTime, minSpinPower, maxSpinSpeed);
            }
            // Cap the maximum spin speed to 70% (~178)
            spinEffort = constrain(spinEffort, -currentMaxSpin, currentMaxSpin);
            
            Motors::setSpeeds(spinEffort, -spinEffort); 

            // If we are within 5 degrees of the 180 flip, lock it in and hunt
            if (abs(error) < 5.0) { 
                currentPunchState = PUNCH_CAPTURE_HEADING;
            }

            if (!snap.matchActiveFlag) currentPunchState = PUNCH_WAIT_START;
            break;
        }
        
    }
}

void Strategy::runStandardProtocol(SharedSensorData &snap) {
    unsigned long currentMillis = millis();

    // --- CONFIGURABLE DRIVE VARIABLES ---
    const int basePower = 255 * 0.70;    // 50% Forward Power
    const int reversePower = 255 * 0.30; // 60% Reverse Power (No more 100% yeet)
    const int maxCorrection = 128;       // Limit how much PWM it can add/subtract to steer
    const int rampTime = 150;          
    const int pauseTime = 500;         
    const int reverseTime = 1500;      

    // One-time PID initialization
    if (!pidInitialized) {
        headingPID.SetMode(AUTOMATIC);
        headingPID.SetOutputLimits(-maxCorrection, maxCorrection); 
        headingPID.SetSampleTime(2); // 500Hz compute capability
        pidInitialized = true;
    }

    // 1. Edge Detection Override (Highest Priority)
    bool edgeDetected = snap.edgeFL1 || snap.edgeFL2 || snap.edgeFR1 || snap.edgeFR2;
    if (edgeDetected && currentStdState == STD_FORWARD_PID) {
        currentStdState = STD_EDGE_PAUSE;
        stateTimer = currentMillis;
        Motors::setSpeeds(0, 0); 
    }

    // 2. Standard Combat State Machine
    switch (currentStdState) {
        case STD_WAIT_START:
            Motors::setSpeeds(0, 0);
            if (snap.matchActiveFlag) {
                currentStdState = STD_CAPTURE_HEADING;
            }
            break;

        case STD_CAPTURE_HEADING:
            // Lock in the current IMU yaw as our absolute forward trajectory
            targetHeading = snap.yaw;
            currentStdState = STD_FORWARD_PID;
            break;

        case STD_FORWARD_PID: {
            // --- IMU Wrap-Around Math ---
            double error = targetHeading - snap.yaw;
            
            // Normalize error to shortest path (-180 to +180)
            if (error > 180.0) error -= 360.0;
            else if (error < -180.0) error += 360.0;

            // --- THE SUMO DEADBAND ---
            // if (abs(error) < 2.0) {
            //     pidInput = 0;          
            //     pidOutput = 0;         
            // } else {
                pidInput = -error; 
                headingPID.Compute();
            // }
// hybride
            float rightMotorTrim = 0.80;
            int leftBase = basePower;
            int rightBase = basePower * rightMotorTrim;

            // --- ADDITIVE DIFFERENTIAL DRIVE ("Quick as F***" Mode) ---
            // Set the absolute floor to basePower. 
            int leftPwr = leftBase;
            int rightPwr = rightBase;

            // --- HYBRID DIFFERENTIAL DRIVE ---
            const int minMovingPower = 255 * 0.30;

            // Only apply PID correction to speed up the outside wheel.
            if (pidOutput > 0) {
                // Steer Right: Left wheel needs to speed up, right wheel slightly brakes
                leftPwr = constrain(leftBase + pidOutput, leftBase, 255);
                rightPwr = constrain(rightBase - (pidOutput * 0.3), 0, 255); // 30% subtractive braking
            } else if (pidOutput < 0) {
                // Steer Left: Right wheel needs to speed up, left wheel slightly brakes
                rightPwr = constrain(rightBase - pidOutput, rightBase, 255); 
                leftPwr = constrain(leftBase + (pidOutput * 0.3), 0, 255);
            }

            Motors::setSpeeds(leftPwr, rightPwr);

            if (!snap.matchActiveFlag) currentStdState = STD_WAIT_START;
            break;
        }

        case STD_EDGE_PAUSE:
            Motors::setSpeeds(0, 0);
            if (currentMillis - stateTimer >= pauseTime) {
                currentStdState = STD_RAMP_REVERSE;
                stateTimer = currentMillis;
            }
            if (!snap.matchActiveFlag) currentStdState = STD_WAIT_START;
            break;

        case STD_RAMP_REVERSE: {
            long elapsed = currentMillis - stateTimer;
            if (elapsed <= rampTime) {
                // Ramp backward aggressively but safely up to reversePower
                int pwr = map(elapsed, 0, rampTime, 0, -reversePower); 
                Motors::setSpeeds(pwr, pwr);
            } else {
                currentStdState = STD_REVERSE;
                stateTimer = currentMillis; 
            }
            if (!snap.matchActiveFlag) currentStdState = STD_WAIT_START;
            break;
        }

        case STD_REVERSE:
            // Use the controlled reverse power instead of -255
            Motors::setSpeeds(-reversePower, -reversePower);
            
            if (currentMillis - stateTimer >= reverseTime) {
                // Capture a NEW heading so we drive straight from our new position
                currentStdState = STD_CAPTURE_HEADING;
            }
            if (!snap.matchActiveFlag) currentStdState = STD_WAIT_START;
            break;
    }
}



void Strategy::runMotorDebugTest(SharedSensorData &snap) {
    unsigned long currentMillis = millis();

    // --- Configurable Variables ---
    const int maxPower = 255 * 0.75; // 75% Power limit (~191)
    const int rampTime = 150;        // 150ms quick ramp (fast but safe for gears)
    const int pauseTime = 500;       // 0.5s pause on edge detection
    const int reverseTime = 1500;    // 1.5s reverse driving duration 

    // 1. Edge Detection Override (Highest Priority)
    bool edgeDetected = snap.edgeFL1 || snap.edgeFL2 || snap.edgeFR1 || snap.edgeFR2;
    
    // If an edge is detected while moving forward, interrupt and trigger the pause sequence
    if (edgeDetected && (currentTestState == TEST_FORWARD || currentTestState == TEST_RAMP_FORWARD)) {
        currentTestState = TEST_EDGE_PAUSE;
        stateTimer = currentMillis;
        Motors::setSpeeds(0, 0); 
    }

    // 2. Motor Test State Machine
    switch (currentTestState) {
        case TEST_WAIT_START:
            Motors::setSpeeds(0, 0);
            if (snap.matchActiveFlag) {
                currentTestState = TEST_RAMP_FORWARD;
                stateTimer = currentMillis;
            }
            break;

        case TEST_RAMP_FORWARD: {
            long elapsed = currentMillis - stateTimer;
            if (elapsed <= rampTime) {
                // Dynamically scale from 0 to maxPower
                int pwr = map(elapsed, 0, rampTime, 0, maxPower);
                Motors::setSpeeds(pwr, pwr);
            } else {
                currentTestState = TEST_FORWARD;
            }
            if (!snap.matchActiveFlag) currentTestState = TEST_WAIT_START;
            break;
        }

        case TEST_FORWARD:
            Motors::setSpeeds(maxPower, maxPower);
            if (!snap.matchActiveFlag) currentTestState = TEST_WAIT_START;
            break;

        case TEST_EDGE_PAUSE:
            Motors::setSpeeds(0, 0);
            // Wait 0.5 seconds to bleed off forward momentum
            if (currentMillis - stateTimer >= pauseTime) {
                currentTestState = TEST_RAMP_REVERSE;
                stateTimer = currentMillis;
            }
            if (!snap.matchActiveFlag) currentTestState = TEST_WAIT_START;
            break;

        case TEST_RAMP_REVERSE: {
            long elapsed = currentMillis - stateTimer;
            if (elapsed <= rampTime) {
                // Dynamically scale from 0 to -maxPower
                int pwr = map(elapsed, 0, rampTime, 0, -maxPower);
                Motors::setSpeeds(pwr, pwr);
            } else {
                currentTestState = TEST_REVERSE;
                stateTimer = currentMillis; // Reset timer for the reverse drive phase
            }
            if (!snap.matchActiveFlag) currentTestState = TEST_WAIT_START;
            break;
        }

        case TEST_REVERSE:
            Motors::setSpeeds(-maxPower, -maxPower);
            // Drive backward for the designated reverse duration
            if (currentMillis - stateTimer >= reverseTime) {
                // Return to forward driving
                currentTestState = TEST_RAMP_FORWARD;
                stateTimer = currentMillis;
            }
            if (!snap.matchActiveFlag) currentTestState = TEST_WAIT_START;
            break;
    }
}


