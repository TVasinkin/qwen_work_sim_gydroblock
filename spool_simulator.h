/**
 * @file spool_simulator.h
 * @brief Модуль симуляции воздействия ШИМ сигнала на пропорциональные клапаны
 *        и положения золотника для STM32F303
 * 
 * Описание системы:
 * - Золотник подпружинен с обеих сторон (нагрузка до 12кг)
 * - Два пропорциональных клапана PPRV-04-S-25-D24 (24В, 25 бар)
 * - Трубки: внутренний диаметр 2мм, длина 100мм
 * - Площадь давления на золотник: 1 см²
 * - Ход золотника: 8мм
 * 
 * Интерфейс:
 * - Вход: значение ШИМ 0-7000 (0% - 100%)
 * - Выход: положение золотника 0-12000 (соответствует 0-8мм)
 * 
 * Характеристики:
 * - Страгивание начинается при 20% ШИМ (~1400 отсчетов)
 * - Полное открытие при 80% ШИМ (~5600 отсчетов)
 * - Учитывается инерция гидравлической системы и задержки
 */

#ifndef SPOOL_SIMULATOR_H
#define SPOOL_SIMULATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * КОНСТАНТЫ И ПАРАМЕТРЫ СИСТЕМЫ
 * ============================================================================ */

// Параметры ШИМ
#define PWM_MIN_VALUE           0U
#define PWM_MAX_VALUE           7000U
#define PWM_THRESHOLD_START     1400U   // 20% от 7000 - начало страгивания
#define PWM_THRESHOLD_FULL      5600U   // 80% от 7000 - полное открытие

// Параметры положения золотника
#define SPOOL_POS_MIN           0U
#define SPOOL_POS_MAX           12000U
#define SPOOL_STROKE_MM         8.0f    // Полный ход в мм
#define SPOOL_POS_PER_MM        1500.0f // 12000 / 8 = 1500 отсчетов на мм

// Физические параметры системы
#define SYSTEM_PRESSURE_BAR     25.0f   // Рабочее давление 25 бар
#define SYSTEM_PRESSURE_PA      (SYSTEM_PRESSURE_BAR * 100000.0f) // 2.5 МПа
#define PISTON_AREA_M2          0.0001f // 1 см² = 0.0001 м²
#define SPRING_FORCE_MAX_N      117.6f  // 12кг * 9.8 м/с²
#define SPRING_STIFFNESS_N_M    5000.0f // Жесткость пружины Н/м

// Параметры трубок
#define TUBE_DIAMETER_M         0.002f  // 2мм
#define TUBE_LENGTH_M           0.1f    // 100мм
#define TUBE_AREA_M2            (3.14159f * TUBE_DIAMETER_M * TUBE_DIAMETER_M / 4.0f)
#define TUBE_VOLUME_M3          (TUBE_AREA_M2 * TUBE_LENGTH_M)

// Параметры гидравлической жидкости (трансформаторное масло)
#define OIL_BULK_MODULUS_PA     1.4e9f  // Модуль объемной упругости ~1400 МПа
#define OIL_DENSITY_KG_M3       850.0f  // Плотность масла кг/м³
#define OIL_VISCOSITY_PAS       0.032f  // Динамическая вязкость ~32 сСт

// Параметры динамики системы
#define VALVE_RESPONSE_TIME_MS  15.0f   // Время реакции клапана мс
#define HYDRAULIC_DELAY_MS      5.0f    // Гидравлическая задержка мс
#define SYSTEM_DAMPING          0.7f    // Коэффициент демпфирования

// Параметры симуляции
#define SIMULATION_DT_MS        1.0f    // Шаг симуляции мс
#define SIMULATION_FREQ_HZ      1000U   // Частота обновления симуляции

/* ============================================================================
 * ТИПЫ ДАННЫХ
 * ============================================================================ */

/**
 * @brief Структура состояния симулятора
 */
typedef struct {
    // Входные данные
    uint16_t pwm_value;             // Текущее значение ШИМ (0-7000)
    
    // Состояние клапанов
    float valve_open_left;          // Открытие левого клапана (0.0-1.0)
    float valve_open_right;         // Открытие правого клапана (0.0-1.0)
    float valve_target_left;        // Целевое открытие левого клапана
    float valve_target_right;       // Целевое открытие правого клапана
    
    // Давления в камерах
    float pressure_left;            // Давление в левой камере (Па)
    float pressure_right;           // Давление в правой камере (Па)
    
    // Состояние золотника
    float spool_position;           // Положение золотника (0.0-12000.0)
    float spool_velocity;           // Скорость золотника (отсчетов/с)
    float spool_acceleration;       // Ускорение золотника (отсчетов/с²)
    
    // Силы
    float force_left;               // Сила слева (Н)
    float force_right;              // Сила справа (Н)
    float force_spring;             // Сила пружины (Н)
    float force_damping;            // Сила демпфирования (Н)
    float force_net;                // Результирующая сила (Н)
    
    // Временные параметры
    uint32_t last_update_time;      // Время последнего обновления (мс)
    float simulation_time;          // Время симуляции (с)
    
    // Флаги состояния
    bool initialized;               // Флаг инициализации
    bool spool_moving;              // Золотник в движении
    bool at_limit_left;             // Достигнут левый предел
    bool at_limit_right;            // Достигнут правый предел
    
} SpoolSimulator_t;

/**
 * @brief Конфигурация симулятора
 */
typedef struct {
    float dt_ms;                    // Шаг симуляции в мс
    float valve_response_time_ms;   // Время реакции клапана
    float hydraulic_delay_ms;       // Гидравлическая задержка
    float damping_coefficient;      // Коэффициент демпфирования
    float mass_kg;                  // Масса золотника (кг)
} SpoolConfig_t;

/* ============================================================================
 * ФУНКЦИИ ИНИЦИАЛИЗАЦИИ
 * ============================================================================ */

/**
 * @brief Инициализация симулятора
 * @param sim Указатель на структуру симулятора
 * @param config Конфигурация симулятора
 * @return true при успешной инициализации
 */
bool SpoolSimulator_Init(SpoolSimulator_t* sim, const SpoolConfig_t* config);

/**
 * @brief Сброс симулятора в начальное состояние
 * @param sim Указатель на структуру симулятора
 */
void SpoolSimulator_Reset(SpoolSimulator_t* sim);

/* ============================================================================
 * ОСНОВНЫЕ ФУНКЦИИ СИМУЛЯЦИИ
 * ============================================================================ */

/**
 * @brief Обновление состояния симулятора
 * @param sim Указатель на структуру симулятора
 * @param pwm_value Новое значение ШИМ (0-7000)
 * @param dt_ms Время прошедшее с последнего обновления (мс)
 */
void SpoolSimulator_Update(SpoolSimulator_t* sim, uint16_t pwm_value, float dt_ms);

/**
 * @brief Получение текущего положения золотника
 * @param sim Указатель на структуру симулятора
 * @return Положение золотника (0-12000)
 */
uint16_t SpoolSimulator_GetPosition(const SpoolSimulator_t* sim);

/**
 * @brief Получение положения золотника в мм
 * @param sim Указатель на структуру симулятора
 * @return Положение золотника в мм (0.0-8.0)
 */
float SpoolSimulator_GetPositionMm(const SpoolSimulator_t* sim);

/**
 * @brief Получение скорости золотника
 * @param sim Указатель на структуру симулятора
 * @return Скорость золотника (отсчетов/с)
 */
float SpoolSimulator_GetVelocity(const SpoolSimulator_t* sim);

/* ============================================================================
 * ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
 * ============================================================================ */

/**
 * @brief Преобразование значения ШИМ в процент открытия клапана
 * @param pwm_value Значение ШИМ (0-7000)
 * @return Процент открытия (0.0-1.0)
 */
float SpoolSimulator_PWMToValveOpen(uint16_t pwm_value);

/**
 * @brief Преобразование положения в мм
 * @param position Положение в отсчетах (0-12000)
 * @return Положение в мм
 */
float SpoolSimulator_PositionToMm(uint16_t position);

/**
 * @brief Преобразование мм в положение
 * @param mm Положение в мм
 * @return Положение в отсчетах
 */
uint16_t SpoolSimulator_MmToPosition(float mm);

/**
 * @brief Проверка инициализирован ли симулятор
 * @param sim Указатель на структуру симулятора
 * @return true если симулятор инициализирован
 */
bool SpoolSimulator_IsInitialized(const SpoolSimulator_t* sim);

/**
 * @brief Получение флага движения золотника
 * @param sim Указатель на структуру симулятора
 * @return true если золотник движется
 */
bool SpoolSimulator_IsMoving(const SpoolSimulator_t* sim);

/**
 * @brief Получение давления в левой камере
 * @param sim Указатель на структуру симулятора
 * @return Давление в Па
 */
float SpoolSimulator_GetPressureLeft(const SpoolSimulator_t* sim);

/**
 * @brief Получение давления в правой камере
 * @param sim Указатель на структуру симулятора
 * @return Давление в Па
 */
float SpoolSimulator_GetPressureRight(const SpoolSimulator_t* sim);

#ifdef __cplusplus
}
#endif

#endif /* SPOOL_SIMULATOR_H */
