# API PCA9685 Robot Arm Controller (STM32)

Version ULTIME d'une bibliothèque embarquée haute performance pour le contrôle non-bloquant de bras robotiques à 6 axes. Ce pilote utilise un contrôleur PWM **PCA9685** (I2C) associé à un **Timer matériel** STM32 (interruptions) pour gérer des trajectoires fluides de type **S-Curve** et le calcul de **Cinématique Inverse (IK)**.

---

## 🚀 Fonctionnalités Clés

* **Gestion Non-Bloquante (FIFO) :** Planification d'enchaînements de mouvements via une file d'attente circulaire (pas de `HAL_Delay`).
* **Profils de Trajectoire S-Curve :** Accélération et décélération progressives pour éliminer les secousses mécaniques et protéger les pignons.
* **Cinématique Inverse (IK) :** Traduction instantanée de coordonnées cartésiennes $(X, Y, Z)$ en angles moteurs pour les 3 axes principaux.
* **Sécurité Matérielle :** Butées logicielles (PWM min/max) personnalisées par axe, gestion des cas hors de portée (`NaN`) et arrêt d'urgence instantané.
* **Thread-Safe :** Protection des sections critiques contre les conditions de concurrence (*Race Conditions*) via le masquage des interruptions.

---

## 🛠️ Architecture du Matériel (Hardware)

### Brochage Type (STM32 ↔ PCA9685)
* **SCL :** Connecté à la broche I2C configurée sur le STM32 (ex: PB6, PB8).
* **SDA :** Connecté à la broche I2C configurée sur le STM32 (ex: PB7, PB9).
* **OE (Output Enable) :** (Optionnel) Maintenir à la masse `GND` pour activer les sorties PWM.

### Calibration Par Défaut des Axes (6 axes)
Le tableau interne `Arm_Axes` configure les moteurs de la manière suivante :


| Index Axe | Rôle | Type de Servo | Amplitude Angle | Plage PWM (Digital) | Spécificité |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **0** | Base | MG996R | 0° à 180° | 130 - 610 | Large amplitude de rotation |
| **1** | Épaule | MG996R | 0° à 180° | 150 - 580 | Sécurité de charge lourde |
| **2** | Coude | MG996R | 0° à 180° | 150 - 580 | Sécurité de charge lourde |
| **3** | Poignet R | SG90 | 0° à 180° | 160 - 550 | Rotation du poignet |
| **4** | Poignet I | SG90 | 0° à 180° | 160 - 550 | Inclinaison du poignet |
| **5** | Pince | SG90 | 0° à 180° | 170 - 500 | Protection mécanique serrage |

---

## 📝 Fichier d'En-tête Requis (`PCA9685_servo_stm32_RJ-ELEKTRONIK.h`)

Pour compiler le code source, vous devez posséder le fichier `.h` contenant les structures et définitions suivantes :

```c
#ifndef PCA9685_SERVO_STM32_RJ_ELEKTRONIK_H_
#define PCA9685_SERVO_STM32_RJ_ELEKTRONIK_H_

#include "stm32f4xx_hal.h" // À adapter selon votre série STM32 (ex: stm32f1xx_hal.h, stm32g4xx_hal.h)
#include <stdbool.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* --- Configurations Système --- */
#define PCA9685_ADDR          (0x40 << 1) // Adresse I2C standard (décalée de 1 bit pour HAL)
#define OSC_FREQ              25000000.0f // Fréquence de l'oscillateur interne du PCA9685
#define TICK_RATE_MS          20          // Intervalle du Timer (20ms pour du 50Hz)
#define ARM_AXES              6           // Nombre total de servomoteurs
#define MAX_QUEUE             32          // Capacité maximale de la file d'attente FIFO

/* --- Registres PCA9685 --- */
#define PCA9685_MODE1         0x00
#define PCA9685_MODE2         0x01
#define PCA9685_PRESCALE      0xFE
#define PCA9685_LED0_ON_L     0x06

/* --- Structures de Données --- */
typedef struct {
    float x;
    float y;
    float z;
} Point3D_t;

typedef struct {
    float current_angle;
    float target_angle;
    float start_angle;
    uint32_t current_tick;
    uint32_t total_ticks;
    uint16_t min_pwm;
    uint16_t max_pwm;
    float max_angle;
    bool use_scurve;
} Servo_Axis_t;

typedef struct {
    float angles[ARM_AXES];
    uint16_t duration;
    bool smooth;
} Arm_Command_t;

typedef struct {
    float angles[ARM_AXES];
    uint16_t duration;
    uint16_t pause_after;
} Robot_Step_t;

/* --- API Publique --- */
void Arm_Pro_Init(I2C_HandleTypeDef *hi2c, TIM_HandleTypeDef *htim);
bool Arm_Enqueue_Move(float *angles, uint16_t duration, bool smooth);
void Arm_Update_Tick(void);
void Arm_Compute_IK(Point3D_t target, float *out_angles);
void Arm_Move_Sync(float *target_angles, uint16_t duration_ms, bool smooth);
void Arm_Emergency_Stop(void);
void Arm_Play_Sequence(Robot_Step_t *sequence, uint8_t step_count, bool smooth);
void Arm_Clear_Queue(void);
void Arm_Diagnostic_Test(void);
bool Arm_Is_Busy(void);

#endif /* PCA9685_SERVO_STM32_RJ_ELEKTRONIK_H_ */
```

---

## 📐 Algorithmes Intégrés

### 1. Interpolation S-Curve (Lissage)
Lorsqu'un mouvement est configuré avec lissage (`smooth = true`), le système calcule le ratio de temps $t \in [0, 1]$ à chaque tick (20 ms). L'angle intermédiaire est modifié par la fonction polynomiale d'Hermite :
$$f(t) = 3t^2 - 2t^3$$
Cette équation force une accélération nulle au démarrage ($t=0$) et au freinage ($t=1$), supprimant l'inertie brusque liée au poids du bras.

### 2. Cinématique Inverse (Modèle Géométrique)
La fonction `Arm_Compute_IK` utilise les dimensions fixes suivantes du bras :
* `L_BASE = 5.0f` cm (Hauteur du pivot de la base)
* `L1 = 15.0f` cm (Longueur du segment Épaule ↔ Coude)
* `L2 = 12.0f` cm (Longueur du segment Coude ↔ Poignet)

L'angle du coude ($\theta_2$) est résolu via le théorème d'Al-Kashi (Loi des cosinus) :
$$\cos(\theta_2) = \frac{r^2 + h^2 - L1^2 - L2^2}{2 \cdot L1 \cdot L2}$$
Où $r = \sqrt{x^2 + y^2}$ et $h = z - L_{base}$.

---

## 📖 Référence de l'API (Fonctions)

### `void Arm_Pro_Init(I2C_HandleTypeDef *hi2c, TIM_HandleTypeDef *htim)`
Initialise le registre interne du PCA9685 via I2C, fixe la fréquence globale à 50 Hz, configure le mode de sortie électrique en Push-Pull et démarre le timer STM32 sous interruptions.

### `bool Arm_Enqueue_Move(float *angles, uint16_t duration, bool smooth)`
Ajoute un objectif de coordonnées angulaires pour les 6 axes à la file d'attente FIFO.
* **Retour :** `true` si le mouvement a été ajouté, `false` si la file d'attente est saturée.

### `void Arm_Compute_IK(Point3D_t target, float *out_angles)`
Prend un point $(X,Y,Z)$ dans l'espace en cm et écrit les 3 angles résultants (Base, Épaule, Coude) dans le tableau de sortie `out_angles`.

### `void Arm_Play_Sequence(Robot_Step_t *sequence, uint8_t step_count, bool smooth)`
Prend un tableau complet d'étapes de mouvements pré-enregistrées et les injecte automatiquement à la suite dans la file d'attente en y intégrant les pauses spécifiées.

### `void Arm_Emergency_Stop(void)`
Commande de sécurité absolue. Coupe instantanément la file d'attente et force la cible des moteurs sur leur position angulaire immédiate pour figer la structure mécanique.

---

## 💻 Exemple d'Intégration Complet (`main.c`)

Voici comment initialiser et utiliser l'API dans votre boucle principale STM32 HAL :

```c
#include "main.h"
#include "PCA9685_servo_stm32_RJ-ELEKTRONIK.h"

// Pointers de configurations HAL générés par CubeMX
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2; // Doit être configuré pour générer un évènement toutes les 20 ms

int main(void)
{
    // --- Initialisations Système générées par CubeMX ---
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_TIM2_Init();

    // 1. Initialisation du bras robotique (PCA9685 + Interruption Timer)
    Arm_Pro_Init(&hi2c1, &htim2);

    // 2. Définition d'une suite de mouvements (Exemple : Séquence de Pick & Place)
    Robot_Step_t pick_and_place[] = {
        // { {Angles Axe 0 à 5}, Durée(ms), PauseAprès(ms) }
        {{90.0f, 45.0f, 120.0f, 90.0f, 90.0f, 0.0f},   1500, 500},  // Étape 1 : Descente vers l'objet, Pince ouverte
        {{90.0f, 45.0f, 120.0f, 90.0f, 90.0f, 80.0f},  800,  400},  // Étape 2 : Fermeture de la pince sur l'objet
        {{90.0f, 90.0f, 70.0f,  90.0f, 90.0f, 80.0f},  1200, 200},  // Étape 3 : Levée du bras (Sécurité mécanique)
        {{180.0f, 90.0f, 70.0f, 90.0f, 90.0f, 80.0f},  2000, 500},  // Étape 4 : Rotation de la base à 180°
        {{180.0f, 45.0f, 120.0f, 90.0f, 90.0f, 0.0f},  1000, 100}   // Étape 5 : Descente et dépose (Pince ouverte)
    };

    // 3. Lancement de la séquence de mouvements lissés (S-Curve)
    Arm_Play_Sequence(pick_and_place, 5, true);

    /* Boucle principale */
    while (1)
    {
        // Exemple d'utilisation de la Cinématique Inverse de manière dynamique
        if (!Arm_Is_Busy()) 
        {
            Point3D_t point_cible = {10.0f, 8.0f, 12.0f}; // Coordonnées en cm (X, Y, Z)
            float angles_calcules[ARM_AXES] = {90.0f, 90.0f, 90.0f, 90.0f, 90.0f, 45.0f}; // Valeurs par défaut

            // Calcul géométrique
            Arm_Compute_IK(point_cible, angles_calcules);

            // Envoi non-bloquant de l'ordre cinématique
            Arm_Enqueue_Move(angles_calcules, 1500, true);
        }

        // Le CPU reste libre ici pour exécuter d'autres calculs ou tâches applicatives !
        HAL_Delay(100); 
    }
}
```
