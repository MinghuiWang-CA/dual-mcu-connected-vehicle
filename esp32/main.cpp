#include <Arduino.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <time.h>

#include "main.h"
#include "NotoSansMonoSCB20.h"
#include "pins_config.h"
#include "xbox_controller_ble.h"

// ================== I2C ==================
#define SDA_PIN 43
#define SCL_PIN 44
#define STM32_ADDR 0x57 // Adresse I2C 7-bit du STM32

#ifndef I2C_TASK_FREQ
#define I2C_TASK_FREQ 25.0f
#endif

#ifndef DISPLAY_TASK_FREQ
#define DISPLAY_TASK_FREQ 25.0f
#endif

// ================== États partagés ==================
volatile uint8_t M1S = 0;
volatile uint8_t M2S = 0;
volatile bool M1D = true; // true = avant (forward), false = arrière (reverse)
volatile bool M2D = true;
volatile bool g_stm32Ok = false;

// Manette Xbox
extern XboxSeriesXControllerESP32_asukiaaa::Core xboxController;
XboxState gXboxState;

// Ligne de statut (texte affiché en haut de l'écran)
String statusLine = "";

// Distance cumulée et vitesse (en cm et cm/s)
volatile long distance_left = 0;
volatile long distance_right = 0;

// Vitesses filtrées (cm/s) en virgule flottante
volatile float g_speedM1_f = 0.0f;
volatile float g_speedM2_f = 0.0f;

// ================== Modes de conduite ==================
volatile int g_mode = 1;       // 1 = mode manuel M1, 2 = mode manuel M2
volatile int g_speedIndex = 0; // niveau de vitesse arrière/rotation (33%,66%,100%)
const float g_speedLevels[3] = {0.33f, 0.66f, 1.0f};

// Mode automatique (A1, A2, A3)
volatile bool g_autoModeActive = false; // True = mode Auto
volatile int g_autoModeIndex = 1;       // 1=A1, 2=A2, 3=A3
volatile bool g_autoRunning = false;    // Auto en cours d'exécution

// Modes manuels sauvegardés
static int lastManualMode = 1; // Pour revenir au mode manuel précédent

// ================== Variables pour l'automatique ==================
volatile int stepNumber = 0;
volatile long startTicksL = 0;
volatile long startTicksR = 0;
volatile long targetTicks = 0;

// ================== NOUVEAUX PARAMETRES ==================
const float PULSES_PER_M = 1815.0f; // Calibré pour 1m
const float CIRCLE_DIAMETER = 1.0f;
const uint8_t CIRCLE_SPEED_OUTER = 240;
float circleSlipFactor = 1.4f;  // Facteur de glissement augmenté
float circleSpeedRatio = 0.40f; // Ratio de vitesse ajusté

// Vitesses automatiques (constantes choisies)
const uint8_t SPEED_AUTO_STD = 220;
const uint8_t SPEED_AUTO_TURN = 220;

// Constantes pour le mode Carré (Accélération)
const int MIN_SPEED_AUTO = 140;
const int MAX_SPEED_AUTO = 250;

// Impulsions nécessaires pour les rotations (approx.)
const long TICKS_ROTATION_90 = 272;
const long TICKS_ROTATION_180 = 790;

// ================== Display ==================
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite sprite = TFT_eSprite(&tft);

// Dimensions écran
int g_screenWidth = 0;
int g_screenHeight = 0;

// ================== Feedback STM32 ==================
const int8_t feedbackLength = 4;

// FreeRTOS handles
TaskHandle_t displayTaskHandle = NULL;
TaskHandle_t i2cTaskHandle = NULL;

// =======================================================
// ExcuteAutoTrajectory : LOGIQUE CORRIGÉE (Cercle Avant + Carré Accel)
// =======================================================
void ExcuteAutoTrajectory()
{
    // Si pause, arrêter les moteurs
    if (!g_autoRunning)
    {
        M1S = 0;
        M2S = 0;
        return;
    }
    // Calcul des distances parcourues en impulsions
    long distL = abs(distance_left - startTicksL);
    long distR = abs(distance_right - startTicksR);
    long distAvg = (distL + distR) / 2;

    // Variable statique pour le carré
    static int squareSideCount = 0;

    switch (g_autoModeIndex)
    {
    case 1: // Cercle (diamètre 0.5 m)
        if (stepNumber == 0)
        {
            long perimPulses = (long)(PULSES_PER_M * 3.14159f * CIRCLE_DIAMETER);
            targetTicks = (long)(perimPulses * circleSlipFactor);
            startTicksL = distance_left;
            startTicksR = distance_right;
            stepNumber = 1;
        }
        if (stepNumber == 1)
        {
            // M1 (droite) = extérieur, M2 (gauche) = intérieur
            M1S = CIRCLE_SPEED_OUTER;
            M1D = true; // CORRECTION: AVANT (true)
            M2S = (uint8_t)(CIRCLE_SPEED_OUTER * circleSpeedRatio);
            M2D = true; // CORRECTION: AVANT (true)

            // Arrêter quand la piste extérieure a parcouru la circonférence
            if (distR >= targetTicks)
            {
                M1S = 0;
                M2S = 0;
                g_autoRunning = false;
                stepNumber = 0;
            }
        }
        break;

    case 2: // Ligne (1 m, demi-tour, 1 m retour)
        if (stepNumber == 0)
        {
            targetTicks = (long)PULSES_PER_M;
            startTicksL = distance_left;
            startTicksR = distance_right;
            stepNumber = 1;
        }
        if (stepNumber == 1)
        {
            // Avancer 1 m
            M1S = SPEED_AUTO_STD;
            M1D = true;
            M2S = SPEED_AUTO_STD;
            M2D = true;
            if (distAvg >= targetTicks)
            {
                // Arrêt, puis demi-tour
                M1S = 0;
                M2S = 0;
                delay(500);
                startTicksL = distance_left;
                startTicksR = distance_right;
                targetTicks = TICKS_ROTATION_180;
                stepNumber = 2;
            }
        }
        else if (stepNumber == 2)
        {
            // Rotation 180° (M1 arrière, M2 avant) -> Correction direction selon ton robot
            M1S = SPEED_AUTO_TURN;
            M1D = true;
            M2S = SPEED_AUTO_TURN;
            M2D = false;
            if (distAvg >= targetTicks)
            {
                M1S = 0;
                M2S = 0;
                delay(500);
                startTicksL = distance_left;
                startTicksR = distance_right;
                targetTicks = (long)PULSES_PER_M;
                stepNumber = 3;
            }
        }
        else if (stepNumber == 3)
        {
            // Retour 1 m
            M1S = SPEED_AUTO_STD;
            M1D = true;
            M2S = SPEED_AUTO_STD;
            M2D = true;
            if (distAvg >= targetTicks)
            {
                M1S = 0;
                M2S = 0;
                g_autoRunning = false;
                stepNumber = 0;
            }
        }
        break;

    case 3:                    // Carré (Avec accélération)
        if (stepNumber == 0) // INIT GLOBAL
        {
            squareSideCount = 0;
            stepNumber = 1;
        }

        // --- Init d'un côté ---
        if (stepNumber == 1)
        {
            if (squareSideCount >= 4)
            {
                g_autoRunning = false;
                stepNumber = 0;
                return;
            }
            targetTicks = (long)PULSES_PER_M;
            startTicksL = distance_left;
            startTicksR = distance_right;
            stepNumber = 2;
        }

        // --- Mouvement avec accélération ---
        else if (stepNumber == 2)
        {
            long halfWay = targetTicks / 2;
            int currentPwm = 0;

            if (distAvg < halfWay)
            {
                // Accélération 0 -> 50%
                currentPwm = map(distAvg, 0, halfWay, MIN_SPEED_AUTO, MAX_SPEED_AUTO);
            }
            else
            {
                // Décélération 50% -> 100%
                currentPwm = map(distAvg, halfWay, targetTicks, MAX_SPEED_AUTO, MIN_SPEED_AUTO);
            }
            currentPwm = constrain(currentPwm, MIN_SPEED_AUTO, MAX_SPEED_AUTO);

            M1S = currentPwm;
            M1D = true;
            M2S = currentPwm;
            M2D = true;

            if (distAvg >= targetTicks)
            {
                M1S = 0;
                M2S = 0;
                delay(200);
                startTicksL = distance_left;
                startTicksR = distance_right;
                targetTicks = TICKS_ROTATION_90;
                stepNumber = 3;
            }
        }

        // --- Rotation 90 ---
        else if (stepNumber == 3)
        {
            M1S = SPEED_AUTO_TURN;
            M1D = true;
            M2S = SPEED_AUTO_TURN;
            M2D = false;

            if (distAvg >= targetTicks)
            {
                M1S = 0;
                M2S = 0;
                delay(500);
                squareSideCount++;
                stepNumber = 1;
            }
        }
        break;
    }
}

// =======================================================
// handleXboxInput : gère les entrées de la manette Xbox
// =======================================================
void handleXboxInput(const XboxState &Xbox)
{
    // États précédents des boutons (pour détecter front montant)
    static bool prevY = false;
    static bool prevB = false;
    static bool prevX = false;
    static bool prevA = false;
    static bool prevRB = false;

    // Lecture des boutons actuels
    bool Y = Xbox.btnY;
    bool B = Xbox.btnB;
    bool X_btn = Xbox.btnX;
    bool A_btn = Xbox.btnA;
    bool RB = Xbox.btnRB;

    // Détection fronts montants (appui)
    bool Y_rising = (Y && !prevY);
    bool B_rising = (B && !prevB);
    bool X_rising = (X_btn && !prevX);
    bool A_rising = (A_btn && !prevA);
    bool RB_rising = (RB && !prevRB);

    // --- Bouton X : bascule entre mode manuel et mode automatique ---
    if (X_rising)
    {
        if (!g_autoModeActive)
        {
            // Passage en mode automatique
            g_autoModeActive = true;
            lastManualMode = g_mode; // sauvegarder le mode manuel courant
            g_autoModeIndex = 1;     // A1 par défaut
            g_autoRunning = false;
            // Coupure moteurs par sécurité
            M1S = 0;
            M2S = 0;
            M1D = true;
            M2D = true;
            statusLine = "Mode AUTO"; // affichage simplifié
            Serial.println("Mode AUTO");
        }
        else
        {
            // Retour en mode manuel
            g_autoModeActive = false;
            g_autoRunning = false;
            // Restaurer le mode manuel précédent (M1 ou M2)
            g_mode = lastManualMode;
            // Coupure moteurs
            M1S = 0;
            M2S = 0;
            M1D = true;
            M2D = true;
            statusLine = "Mode MANUELLE";
            Serial.println("Mode MANUELLE");
        }
        prevY = Y;
        prevB = B;
        prevX = X_btn;
        prevA = A_btn;
        prevRB = RB;
        return;
    }

    // --- Si mode automatique actif ---
    if (g_autoModeActive)
    {
        // Bouton A : changer de sous-mode auto (A1, A2, A3) si arrêt
        if (A_rising && !g_autoRunning)
        {
            g_autoModeIndex = (g_autoModeIndex % 3) + 1;
            stepNumber = 0;
            startTicksL = distance_left;
            startTicksR = distance_right;
            // Mettre à jour ligne de statut
            if (g_autoModeIndex == 1)
                statusLine = "Mode AUTO A1 (CERCLE)";
            else if (g_autoModeIndex == 2)
                statusLine = "Mode AUTO A2 (LIGNE)";
            else
                statusLine = "Mode AUTO A3 (CARRE)";
            Serial.println(statusLine);
        }
        // Bouton RB : démarrer / mettre en pause
        if (RB_rising)
        {
            if (!g_autoRunning)
            {
                g_autoRunning = true;
                // On garde les compteurs si reprise, sinon reset si stepNumber == 0
                if (stepNumber == 0)
                {
                    startTicksL = distance_left;
                    startTicksR = distance_right;
                }
                Serial.println("Auto mode démarré");
            }
            else
            {
                g_autoRunning = false;
                Serial.println("Auto mode mis en pause");
            }
            M1S = 0;
            M2S = 0;
        }
        prevY = Y;
        prevB = B;
        prevX = X_btn;
        prevA = A_btn;
        prevRB = RB;
        return;
    }

    // ==================================================
    // MODE MANUEL ACTIF
    // ==================================================
    if (Y_rising)
    {
        g_mode = (g_mode == 1) ? 2 : 1;
        M1S = 0;
        M2S = 0;
        M1D = true;
        M2D = true;
        if (g_mode == 1)
            statusLine = "Mode 1 (RT/LT)";
        else
            statusLine = "Mode 2 (Joyst)";
        Serial.println(statusLine);
    }
    if (B_rising)
    {
        g_speedIndex = (g_speedIndex + 1) % 3;
        int percent = (int)(g_speedLevels[g_speedIndex] * 100.0f);
        Serial.printf("Niveau vitesse arrière/rot = %d%%\n", percent);
    }

    uint8_t pwm1 = 0;
    uint8_t pwm2 = 0;
    bool dir1 = true;
    bool dir2 = true;

    if (g_mode == 1)
    {
        if (Xbox.trigRT > 30)
        {
            dir1 = true; // forward
            pwm1 = (uint8_t)constrain(map((int)Xbox.trigRT, 30, 1023, 0, 255), 0, 255);
        }
        if (Xbox.trigLT > 30)
        {
            dir2 = true; // forward
            pwm2 = (uint8_t)constrain(map((int)Xbox.trigLT, 30, 1023, 0, 255), 0, 255);
        }
        uint8_t backPWM = (uint8_t)(g_speedLevels[g_speedIndex] * 255.0f);
        if (pwm1 == 0 && Xbox.btnRB)
        {
            dir1 = false;
            pwm1 = backPWM;
        }
        if (pwm2 == 0 && Xbox.btnLB)
        {
            dir2 = false;
            pwm2 = backPWM;
        }
    }
    else // Mode 2
    {
        int32_t rawX = (int32_t)Xbox.joyLHori;
        int32_t rawY = (int32_t)Xbox.joyLVert;
        static int32_t joyCenterX = 0;
        static int32_t joyCenterY = 0;
        static bool joyCenterValid = false;
        if (!joyCenterValid)
        {
            joyCenterX = rawX;
            joyCenterY = rawY;
            joyCenterValid = true;
        }

        int32_t x = rawX - joyCenterX;
        int32_t y = rawY - joyCenterY;
        const int32_t DEADZONE_RAW = 4000;
        if (labs(x) < DEADZONE_RAW)
            x = 0;
        if (labs(y) < DEADZONE_RAW)
            y = 0;

        const float JOY_RANGE = 30000.0f;
        float nx = (float)x / JOY_RANGE;
        float ny = (float)y / JOY_RANGE;
        nx = constrain(nx, -1.0f, 1.0f);
        ny = constrain(ny, -1.0f, 1.0f);

        const float DEADZONE_F = 0.10f;
        if (fabs(nx) < DEADZONE_F)
            nx = 0.0f;
        if (fabs(ny) < DEADZONE_F)
            ny = 0.0f;

        float vy = -ny;
        float vx = nx;
        float left = vy + vx;
        float right = vy - vx;
        left = constrain(left, -1.0f, 1.0f);
        right = constrain(right, -1.0f, 1.0f);

        float rot = g_speedLevels[g_speedIndex];
        if (Xbox.btnLB && !Xbox.btnRB)
        {
            left = rot;
            right = -rot;
        }
        else if (Xbox.btnRB && !Xbox.btnLB)
        {
            left = -rot;
            right = rot;
        }

        if (right >= 0.0f)
        {
            dir1 = true;
        }
        else
        {
            dir1 = false;
            right = -right;
        }
        right = constrain(right, 0.0f, 1.0f);
        pwm1 = (uint8_t)(right * 255.0f);

        if (left >= 0.0f)
        {
            dir2 = true;
        }
        else
        {
            dir2 = false;
            left = -left;
        }
        left = constrain(left, 0.0f, 1.0f);
        pwm2 = (uint8_t)(left * 255.0f);
    }

    M1S = pwm1;
    M2S = pwm2;
    M1D = dir1;
    M2D = dir2;

    prevY = Y;
    prevB = B;
    prevX = X_btn;
    prevA = A_btn;
    prevRB = RB;
}

// =======================================================
// I2CTask : Envoi moteurs + Lecture encodeurs -> CORE 0
// =======================================================
void I2CTask(void *pvParameters)
{
    (void)pvParameters;
    Serial.println("[I2C] Task started");
    pinMode(SDA_PIN, INPUT_PULLUP);
    pinMode(SCL_PIN, INPUT_PULLUP);
    Wire.begin(SDA_PIN, SCL_PIN, 400000); // 400 kHz pour fluidité
    delay(500);

    while (true)
    {
        // ** Mise à jour auto **
        if (g_autoModeActive && g_autoRunning)
        {
            ExcuteAutoTrajectory();
        }

        // Envoyer commandes
        uint8_t sm1, sm2, sd1, sd2;
        noInterrupts();
        sm1 = M1S;
        sm2 = M2S;
        sd1 = M1D ? 1 : 0;
        sd2 = M2D ? 1 : 0;
        interrupts();

        Wire.beginTransmission(STM32_ADDR);
        Wire.write(sm1);
        Wire.write(sd1);
        Wire.write(sm2);
        Wire.write(sd2);
        delayMicroseconds(500);
        uint8_t txError = Wire.endTransmission(true);

        if (txError == 0)
        {
            if (!g_stm32Ok)
            {
                g_stm32Ok = true;
                Serial.println("[I2C] STM32 OK");
            }

            // Lecture feedback
            delayMicroseconds(500);
            if (Wire.requestFrom(STM32_ADDR, feedbackLength) == feedbackLength)
            {
                uint8_t c1L = Wire.read();
                uint8_t c1H = Wire.read();
                uint8_t c2L = Wire.read();
                uint8_t c2H = Wire.read();
                uint16_t newCntM1 = (uint16_t)(c1L | (c1H << 8));
                uint16_t newCntM2 = (uint16_t)(c2L | (c2H << 8));

                // Calcul vitesse simple
                static uint16_t lastCntM1 = 0, lastCntM2 = 0;
                int16_t d1 = newCntM1 - lastCntM1;
                int16_t d2 = newCntM2 - lastCntM2;
                lastCntM1 = newCntM1;
                lastCntM2 = newCntM2;

                // Mise à jour odométrie
                noInterrupts();
                distance_right += abs(d1); // M1
                distance_left += abs(d2);  // M2
                interrupts();

                // Pour affichage
                g_speedM1_f = d1 * 5.0f; // Approx Hz to speed
                g_speedM2_f = d2 * 5.0f;
            }
        }
        else
        {
            g_stm32Ok = false;
        }

        vTaskDelay(pdMS_TO_TICKS(1000.0f / I2C_TASK_FREQ));
    }
}

// =======================================================
// DisplayTask : AFFICHE STRICTEMENT L'ORIGINE -> CORE 0 (MODIFIE)
// =======================================================
void DisplayTask(void *pvParameters)
{
    (void)pvParameters;
    const int barWidth = 40;
    const int barMaxHeight = 80;
    const int baseY = 155;
    const int m1X = 60;
    const int m2X = 180;
    while (true)
    {
        sprite.fillSprite(TFT_BLACK);

        // --------------------------------------------------
        // Ligne de statut en haut à gauche
        // --------------------------------------------------
        sprite.setTextDatum(TL_DATUM);
        sprite.setTextColor(TFT_YELLOW, TFT_BLACK);
        if (g_autoModeActive)
        {
            sprite.drawString("Mode: AUTOMATIQUE", 80, 5, 2);
        }
        else
        {
            sprite.drawString("Mode: MANUELLE", 80, 5, 2);
        }
        // Sous-mode (M1/M2 ou A1/A2/A3)
        sprite.setTextColor(TFT_WHITE, TFT_BLACK);
        if (!g_autoModeActive)
        {
            String mdesc = "M" + String(g_mode);
            sprite.drawString(mdesc, 80, 25, 2);
        }
        else
        {
            String adescr;
            if (g_autoModeIndex == 1)
                adescr = "A1 (CERCLE)";
            else if (g_autoModeIndex == 2)
                adescr = "A2 (LIGNE)";
            else
                adescr = "A3 (CARRE)";
            sprite.drawString(adescr, 80, 25, 2);
        }

        // --------------------------------------------------
        // Heure/Date en haut à droite
        // --------------------------------------------------
        sprite.setTextDatum(TR_DATUM);
        sprite.setTextColor(TFT_WHITE, TFT_BLACK);
        struct tm timeinfo;
         if (getLocalTime(&timeinfo))
         {
             char heureStr[16], dateStr[16];
             strftime(heureStr, sizeof(heureStr), "%H:%M:%S", &timeinfo);
             strftime(dateStr, sizeof(dateStr), "%d-%m-%Y", &timeinfo);
             sprite.drawString(heureStr, g_screenWidth - 5, 5, 2);
             sprite.drawString(dateStr, g_screenWidth - 5, 25, 1);
         }
         else
         {
             sprite.drawString("Heure N/C", g_screenWidth - 5, 5, 2);
         }

        sprite.pushImage(240, 100, LOGO_WIDTH, LOGO_HEIGHT, logo);

        // --------------------------------------------------
        // Connexions : Xbox et STM32
        // --------------------------------------------------
        sprite.setTextDatum(TL_DATUM);
        sprite.setTextColor(TFT_WHITE, TFT_BLACK);
        sprite.drawString("Xbox:", 5, 60, 2);
        if (gXboxState.connected)
        {
            sprite.setTextColor(TFT_GREEN, TFT_BLACK);
            String s = "OK  Bat=" + String(gXboxState.battery) + "%";
            sprite.drawString(s, 80, 60, 2);
        }
        else
        {
            sprite.setTextColor(TFT_RED, TFT_BLACK);
            sprite.drawString("DECONNECTEE", 80, 60, 2);
        }
        sprite.setTextColor(TFT_WHITE, TFT_BLACK);
        sprite.drawString("STM32:", 5, 85, 2);
        if (g_stm32Ok)
        {
            sprite.setTextColor(TFT_GREEN, TFT_BLACK);
            sprite.drawString("I2C OK", 80, 85, 2);
        }
        else
        {
            sprite.setTextColor(TFT_RED, TFT_BLACK);
            sprite.drawString("I2C ERR", 80, 85, 2);
        }

        // --------------------------------------------------
        // Affichage des vitesses courantes (tr/s)
        // --------------------------------------------------
        sprite.setTextColor(TFT_CYAN, TFT_BLACK);
        float rot1 = g_speedM1_f / 66.0f; // 1 rot/s = 66 cm/s approx
        float rot2 = g_speedM2_f / 66.0f;
        String info1 = "M1=" + String(rot1, 1) + " tr/s  " + (M1D ? "FWD" : "REV");
        String info2 = "M2=" + String(rot2, 1) + " tr/s  " + (M2D ? "FWD" : "REV");
        sprite.drawString(info1, 5, 110, 2);
        sprite.drawString(info2, 5, 130, 2);

        sprite.pushSprite(0, 0);
        vTaskDelay(pdMS_TO_TICKS(1000.0f / 100));
    }
}

// ======================= SETUP =======================
const char *WIFI_SSID = "SM-G990W23391";
const char *WIFI_PASSWORD = "rqqe9995";

void setup()
{
    Serial.begin(250000);
    delay(500);
    Serial.println("=== Setup start ===");
    // Alimentation et initialisation TFT
    pinMode(PIN_POWER_ON, OUTPUT);
    digitalWrite(PIN_POWER_ON, HIGH);
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    sprite.createSprite(TFT_HEIGHT, TFT_WIDTH);
    sprite.setTextDatum(MC_DATUM);
    sprite.setTextColor(TFT_RED, TFT_BLACK);
    sprite.setSwapBytes(1);
    sprite.fillScreen(TFT_BLACK);
    sprite.loadFont(NotoSansMonoSCB20);

    g_screenWidth = tft.width();
    g_screenHeight = tft.height();

    // Connexion WiFi NON BLOQUANTE (pour eviter lag demarrage)
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    // On ne met pas de while() ici pour ne pas bloquer le robot

    // Création des tâches FreeRTOS sur cœurs séparés
    // Core 0 : Moteurs et I2C (Keep at Core 0)
    xTaskCreatePinnedToCore(I2CTask, "I2CTask", 4096, NULL, 10, &i2cTaskHandle, 0);

    // Core 0 : Affichage (DEPLACE DU CORE 1 AU CORE 0 POUR EVITER LAG MANETTE)
    // Cela libère le Core 1 pour la boucle loop() qui gère la Xbox
    xTaskCreatePinnedToCore(DisplayTask, "DisplayTask", 4096, NULL, 1, &displayTaskHandle, 1);

    // Démarrage tâches manette Xbox
    startXboxTasks();
}

void loop()
{
}