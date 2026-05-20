/**
 * @file spool_simulator.h
 * @brief Симулятор гидравлического золотника с управлением двумя пропорциональными клапанами
 * 
 * Особенности:
 * - Два независимых входа ШИМ (Клапан 1: 0..+12000, Клапан 2: 0..-12000)
 * - Мертвая зона: при ШИМ < 10% пружины возвращают золотник в 0
 * - Параметр нестабильности давления RND_PRESS_DEVIATION
 * - Совместимость с STM32F303 (без динамической памяти)
 */

#ifndef SPOOL_SIMULATOR_H
#define SPOOL_SIMULATOR_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================= Конфигурация ================= */

// Диапазоны сигналов
#define PWM_MIN             0U
#define PWM_MAX             7000U
#define PWM_NEUTRAL_THRESHOLD_PERCENT 10U // Мертвая зона (<10% - возврат в центр)
#define PWM_NEUTRAL_THRESHOLD ((PWM_MAX * PWM_NEUTRAL_THRESHOLD_PERCENT) / 100U) // 700 отсчетов

// Диапазон положения золотника
#define SPOOL_POS_MIN       (-12000)
#define SPOOL_POS_MAX       (12000)
#define SPOOL_POS_CENTER    0
#define SPOOL_STROKE_MM     8.0f          // Полный ход 8мм (-4..+4мм от центра)
#define SPOOL_POS_SCALE     (SPOOL_STROKE_MM / (float)SPOOL_POS_MAX) // мм на отсчет

// Физические параметры
#define SYSTEM_PRESSURE_BAR 25.0f         // Рабочее давление 25 бар
#define SYSTEM_PRESSURE_PA  (SYSTEM_PRESSURE_BAR * 1e5f) // Па
#define TUBE_DIAMETER_MM    2.0f
#define TUBE_LENGTH_MM      100.0f
#define SPOOL_AREA_CM2      1.0f
#define SPOOL_AREA_M2       (SPOOL_AREA_CM2 * 1e-4f)
#define SPRING_FORCE_KG     12.0f         // Нагрузка пружин 12кг
#define SPRING_FORCE_N      (SPRING_FORCE_KG * 9.81f)
#define OIL_BULK_MODULUS    1.4e9f        // Модуль объемной упругости масла (Па)
#define OIL_DENSITY         850.0f        // Плотность масла (кг/м³)
#define VISCOSITY_DAMPING   500.0f        // Коэффициент демпфирования

// Нестабильность давления (%)
#ifndef RND_PRESS_DEVIATION
#define RND_PRESS_DEVIATION 2.0f          // По умолчанию ±2%
#endif

// Параметры симуляции
#define VALVE_RESPONSE_TIME_MS 15.0f      // Время реакции клапана (мс)
#define SPOOL_MASS_KG       0.3f          // Масса золотника (примерно)

// Сила пружины на полном ходе (для расчета жесткости)
// Пружина должна возвращать золотник в центр, но не мешать полному ходу при макс давлении
// При 25 бар и площади 1см² сила = 250Н, пружина должна быть слабее
#define SPRING_STIFFNESS_N_PER_MM  (SPRING_FORCE_N / 4.0f) // Н/мм (на полный ход 4мм)

/* ================= Типы данных ================= */

/**
 * @brief Структура конфигурации симулятора
 */
typedef struct {
    float dt;                 // Шаг симуляции (сек)
    float pressure_deviation; // Разброс давления (%)
    uint32_t seed;            // Начальное значение для ГСЧ
} SpoolConfig_t;

/**
 * @brief Структура состояния симулятора
 */
typedef struct {
    // Входы
    uint16_t pwm_valve1;      // ШИМ клапана 1 (движение в +)
    uint16_t pwm_valve2;      // ШИМ клапана 2 (движение в -)
    
    // Состояния клапанов (фильтрованные)
    float valve1_opening;     // 0.0 .. 1.0
    float valve2_opening;     // 0.0 .. 1.0
    
    // Давления в камерах (Па)
    float pressure_A;         // Камера A (клапан 1)
    float pressure_B;         // Камера B (клапан 2)
    
    // Состояние золотника
    float position_mm;        // Положение в мм (-4.0 .. +4.0)
    float velocity;           // Скорость (м/с)
    int32_t position_counts;  // Положение в отсчетах (-12000 .. +12000)
    
    // Внутренние переменные
    float sim_time;
    uint32_t rng_state;
    
    // Флаги
    bool at_limit_positive;
    bool at_limit_negative;
} SpoolSimulator_t;

/* ================= Инициализация ================= */

/**
 * @brief Инициализация симулятора
 * @param sim Указатель на структуру симулятора
 * @param config Конфигурация (можно NULL для значений по умолчанию)
 */
void SpoolSimulator_Init(SpoolSimulator_t* sim, const SpoolConfig_t* config);

/**
 * @brief Сброс симулятора в начальное состояние
 * @param sim Указатель на структуру симулятора
 */
void SpoolSimulator_Reset(SpoolSimulator_t* sim);

/* ================= Основной цикл ================= */

/**
 * @brief Шаг симуляции
 * @param sim Указатель на структуру симулятора
 * @param pwm_v1 Значение ШИМ клапана 1 (0-7000)
 * @param pwm_v2 Значение ШИМ клапана 2 (0-7000)
 * @param dt Шаг времени в секундах (рекомендуется 0.001 для 1мс)
 */
void SpoolSimulator_Step(SpoolSimulator_t* sim, uint16_t pwm_v1, uint16_t pwm_v2, float dt);

/* ================= Получение данных ================= */

/**
 * @brief Получить текущее положение золотника в отсчетах
 * @param sim Указатель на структуру симулятора
 * @return Положение от -12000 до +12000
 */
int32_t SpoolSimulator_GetPosition(const SpoolSimulator_t* sim);

/**
 * @brief Получить текущее положение золотника в мм
 * @param sim Указатель на структуру симулятора
 * @return Положение от -4.0 до +4.0 мм
 */
float SpoolSimulator_GetPositionMm(const SpoolSimulator_t* sim);

/**
 * @brief Получить давление в камере A (клапан 1)
 * @param sim Указатель на структуру симулятора
 * @return Давление в Паскалях
 */
float SpoolSimulator_GetPressureA(const SpoolSimulator_t* sim);

/**
 * @brief Получить давление в камере B (клапан 2)
 * @param sim Указатель на структуру симулятора
 * @return Давление в Паскалях
 */
float SpoolSimulator_GetPressureB(const SpoolSimulator_t* sim);

/**
 * @brief Получить скорость золотника
 * @param sim Указатель на структуру симулятора
 * @return Скорость в м/с
 */
float SpoolSimulator_GetVelocity(const SpoolSimulator_t* sim);

/**
 * @brief Проверка достижения положительного предела
 * @param sim Указатель на структуру симулятора
 * @return true если достигнут предел +12000
 */
bool SpoolSimulator_IsAtPositiveLimit(const SpoolSimulator_t* sim);

/**
 * @brief Проверка достижения отрицательного предела
 * @param sim Указатель на структуру симулятора
 * @return true если достигнут предел -12000
 */
bool SpoolSimulator_IsAtNegativeLimit(const SpoolSimulator_t* sim);

/**
 * @brief Проверка нахождения в мертвой зоне
 * @param sim Указатель на структуру симулятора
 * @return true если оба ШИМ < 10%
 */
bool SpoolSimulator_IsInDeadZone(const SpoolSimulator_t* sim);

/* ================= Утилиты ================= */

/**
 * @brief Преобразование положения в мм в отсчеты
 * @param pos_mm Положение в мм
 * @return Положение в отсчетах
 */
int32_t SpoolSimulator_MmToCounts(float pos_mm);

/**
 * @brief Преобразование отсчетов в мм
 * @param counts Положение в отсчетах
 * @return Положение в мм
 */
float SpoolSimulator_CountsToMm(int32_t counts);

/**
 * @brief Преобразование процентов ШИМ в отсчеты
 * @param percent Процент (0-100)
 * @return Отсчеты ШИМ (0-7000)
 */
uint16_t SpoolSimulator_PercentToPwm(float percent);

/**
 * @brief Преобразование отсчетов ШИМ в проценты
 * @param pwm Отсчеты ШИМ (0-7000)
 * @return Процент (0-100)
 */
float SpoolSimulator_PwmToPercent(uint16_t pwm);

#ifdef __cplusplus
}
#endif

#endif /* SPOOL_SIMULATOR_H */
