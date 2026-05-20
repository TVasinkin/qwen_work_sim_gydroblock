/**
 * @file spool_simulator.c
 * @brief Реализация симулятора гидравлического золотника с двумя пропорциональными клапанами
 */

#include "spool_simulator.h"
#include <math.h>
#include <string.h>

/* ============================================================================
 * ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
 * ============================================================================ */

/**
 * @brief Простой генератор псевдослучайных чисел (LCG)
 */
static uint32_t rng_next(uint32_t* state) {
    *state = (*state * 1664525UL + 1013904223UL);
    return *state;
}

/**
 * @brief Получить случайное число в диапазоне [0.0, 1.0]
 */
static float rng_float(uint32_t* state) {
    return (float)rng_next(state) / (float)0xFFFFFFFFUL;
}

/**
 * @brief Ограничение значения диапазоном
 */
static float clamp(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

/**
 * @brief Вычисление давления с учетом нестабильности
 */
static float calc_pressure_with_deviation(float base_pressure, float deviation_percent, uint32_t* rng_state) {
    float deviation = (rng_float(rng_state) * 2.0f - 1.0f) * deviation_percent / 100.0f;
    return base_pressure * (1.0f + deviation);
}

/* ============================================================================
 * ИНИЦИАЛИЗАЦИЯ
 * ============================================================================ */

void SpoolSimulator_Init(SpoolSimulator_t* sim, const SpoolConfig_t* config) {
    if (sim == NULL) return;
    
    memset(sim, 0, sizeof(SpoolSimulator_t));
    
    // Значения по умолчанию
    sim->sim_time = 0.0f;
    sim->rng_state = config ? config->seed : 12345U;
    sim->position_mm = 0.0f;
    sim->position_counts = 0;
    sim->velocity = 0.0f;
    sim->valve1_opening = 0.0f;
    sim->valve2_opening = 0.0f;
    sim->pressure_A = 0.0f;
    sim->pressure_B = 0.0f;
    sim->at_limit_positive = false;
    sim->at_limit_negative = false;
}

void SpoolSimulator_Reset(SpoolSimulator_t* sim) {
    if (sim == NULL) return;
    
    sim->sim_time = 0.0f;
    sim->position_mm = 0.0f;
    sim->position_counts = 0;
    sim->velocity = 0.0f;
    sim->valve1_opening = 0.0f;
    sim->valve2_opening = 0.0f;
    sim->pressure_A = 0.0f;
    sim->pressure_B = 0.0f;
    sim->at_limit_positive = false;
    sim->at_limit_negative = false;
}

/* ============================================================================
 * ОСНОВНОЙ ЦИКЛ СИМУЛЯЦИИ
 * ============================================================================ */

void SpoolSimulator_Step(SpoolSimulator_t* sim, uint16_t pwm_v1, uint16_t pwm_v2, float dt) {
    if (sim == NULL || dt <= 0.0f) return;
    
    // Ограничение входных значений
    pwm_v1 = clamp((float)pwm_v1, PWM_MIN, PWM_MAX);
    pwm_v2 = clamp((float)pwm_v2, PWM_MIN, PWM_MAX);
    
    sim->pwm_valve1 = pwm_v1;
    sim->pwm_valve2 = pwm_v2;
    sim->sim_time += dt;
    
    // ========== 1. Обработка мертвой зоны ==========
    bool in_deadzone = (pwm_v1 < PWM_NEUTRAL_THRESHOLD) && (pwm_v2 < PWM_NEUTRAL_THRESHOLD);
    
    // ========== 2. Динамика открытия клапанов (экспоненциальный фильтр) ==========
    float target_opening_1 = (pwm_v1 >= PWM_NEUTRAL_THRESHOLD) ? 
        ((float)(pwm_v1 - PWM_NEUTRAL_THRESHOLD) / (float)(PWM_MAX - PWM_NEUTRAL_THRESHOLD)) : 0.0f;
    float target_opening_2 = (pwm_v2 >= PWM_NEUTRAL_THRESHOLD) ? 
        ((float)(pwm_v2 - PWM_NEUTRAL_THRESHOLD) / (float)(PWM_MAX - PWM_NEUTRAL_THRESHOLD)) : 0.0f;
    
    // Время реакции клапана
    float tau = VALVE_RESPONSE_TIME_MS / 1000.0f;
    float alpha = dt / (tau + dt);
    
    sim->valve1_opening = sim->valve1_opening + alpha * (target_opening_1 - sim->valve1_opening);
    sim->valve2_opening = sim->valve2_opening + alpha * (target_opening_2 - sim->valve2_opening);
    
    // ========== 3. Расчет давлений в камерах ==========
    // Базовое давление пропорционально открытию клапана
    float base_pressure_A = sim->valve1_opening * SYSTEM_PRESSURE_PA;
    float base_pressure_B = sim->valve2_opening * SYSTEM_PRESSURE_PA;
    
    // Добавляем нестабильность давления
    float dev = (sim->rng_state != 0) ? RND_PRESS_DEVIATION : 0.0f;
    sim->pressure_A = calc_pressure_with_deviation(base_pressure_A, dev, &sim->rng_state);
    sim->pressure_B = calc_pressure_with_deviation(base_pressure_B, dev, &sim->rng_state);
    
    // ========== 4. Расчет сил на золотнике ==========
    // Сила от давления в камере A (толкает вправо, положительное направление)
    float force_A = sim->pressure_A * SPOOL_AREA_M2;
    
    // Сила от давления в камере B (толкает влево, отрицательное направление)
    float force_B = sim->pressure_B * SPOOL_AREA_M2;
    
    // Суммарная сила от давления
    float force_pressure = force_A - force_B;
    
    // Сила пружин (возвращает к центру)
    // Пружины работают всегда, стремясь вернуть золотник в 0
    // При давлении 25 бар и площади 1см² сила = 250Н
    // Пружина с нагрузкой 12кг = 117Н должна позволять полных ход при макс давлении
    float max_spring_force = SPRING_FORCE_N; // 117Н
    float max_deflection_mm = SPOOL_STROKE_MM / 2.0f; // 4мм
    float spring_k = max_spring_force / max_deflection_mm; // ~29 Н/мм
    float spring_force = -sim->position_mm * spring_k;
    
    // Если в мертвой зоне - пружины доминируют
    if (in_deadzone) {
        force_pressure = 0.0f;
    }
    
    // Сила демпфирования (вязкое трение)
    float damping_force = -VISCOSITY_DAMPING * sim->velocity;
    
    // Суммарная сила
    float total_force = force_pressure + spring_force + damping_force;
    
    // ========== 5. Интегрирование уравнения движения ==========
    // a = F / m
    float acceleration = total_force / SPOOL_MASS_KG;
    
    // v = v0 + a * dt
    sim->velocity = sim->velocity + acceleration * dt;
    
    // x = x0 + v * dt
    sim->position_mm = sim->position_mm + sim->velocity * dt;
    
    // ========== 6. Ограничение положения ==========
    float max_mm = SPOOL_STROKE_MM / 2.0f;  // +4мм
    float min_mm = -SPOOL_STROKE_MM / 2.0f; // -4мм
    
    sim->at_limit_positive = false;
    sim->at_limit_negative = false;
    
    if (sim->position_mm > max_mm) {
        sim->position_mm = max_mm;
        sim->velocity = 0.0f;
        sim->at_limit_positive = true;
    }
    if (sim->position_mm < min_mm) {
        sim->position_mm = min_mm;
        sim->velocity = 0.0f;
        sim->at_limit_negative = true;
    }
    
    // ========== 7. Преобразование в отсчеты ==========
    // -4мм -> -12000, +4мм -> +12000
    sim->position_counts = SpoolSimulator_MmToCounts(sim->position_mm);
}

/* ============================================================================
 * ФУНКЦИИ ПОЛУЧЕНИЯ ДАННЫХ
 * ============================================================================ */

int32_t SpoolSimulator_GetPosition(const SpoolSimulator_t* sim) {
    if (sim == NULL) return 0;
    return sim->position_counts;
}

float SpoolSimulator_GetPositionMm(const SpoolSimulator_t* sim) {
    if (sim == NULL) return 0.0f;
    return sim->position_mm;
}

float SpoolSimulator_GetPressureA(const SpoolSimulator_t* sim) {
    if (sim == NULL) return 0.0f;
    return sim->pressure_A;
}

float SpoolSimulator_GetPressureB(const SpoolSimulator_t* sim) {
    if (sim == NULL) return 0.0f;
    return sim->pressure_B;
}

float SpoolSimulator_GetVelocity(const SpoolSimulator_t* sim) {
    if (sim == NULL) return 0.0f;
    return sim->velocity;
}

bool SpoolSimulator_IsAtPositiveLimit(const SpoolSimulator_t* sim) {
    if (sim == NULL) return false;
    return sim->at_limit_positive;
}

bool SpoolSimulator_IsAtNegativeLimit(const SpoolSimulator_t* sim) {
    if (sim == NULL) return false;
    return sim->at_limit_negative;
}

bool SpoolSimulator_IsInDeadZone(const SpoolSimulator_t* sim) {
    if (sim == NULL) return false;
    return (sim->pwm_valve1 < PWM_NEUTRAL_THRESHOLD) && 
           (sim->pwm_valve2 < PWM_NEUTRAL_THRESHOLD);
}

/* ============================================================================
 * УТИЛИТЫ
 * ============================================================================ */

int32_t SpoolSimulator_MmToCounts(float pos_mm) {
    // -4мм -> -12000, +4мм -> +12000
    // коэффициент: 12000 / 4 = 3000 отсчетов на мм
    float counts = pos_mm * (SPOOL_POS_MAX / (SPOOL_STROKE_MM / 2.0f));
    
    // Округление до целого
    if (counts >= 0.0f) {
        return (int32_t)(counts + 0.5f);
    } else {
        return (int32_t)(counts - 0.5f);
    }
}

float SpoolSimulator_CountsToMm(int32_t counts) {
    return (float)counts / (SPOOL_POS_MAX / (SPOOL_STROKE_MM / 2.0f));
}

uint16_t SpoolSimulator_PercentToPwm(float percent) {
    percent = clamp(percent, 0.0f, 100.0f);
    return (uint16_t)((percent / 100.0f) * (float)PWM_MAX);
}

float SpoolSimulator_PwmToPercent(uint16_t pwm) {
    return ((float)pwm / (float)PWM_MAX) * 100.0f;
}
