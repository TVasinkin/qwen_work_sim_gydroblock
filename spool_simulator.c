/**
 * @file spool_simulator.c
 * @brief Реализация модуля симуляции воздействия ШИМ сигнала на пропорциональные
 *        клапаны и положения золотника для STM32F303
 * 
 * Физическая модель учитывает:
 * - Инерцию гидравлической системы
 * - Сжимаемость масла в трубках
 * - Динамику открытия пропорциональных клапанов
 * - Силы пружин, демпфирования и давления
 * - Задержки распространения давления
 */

#include "spool_simulator.h"
#include <math.h>
#include <string.h>

/* ============================================================================
 * МАКРОСЫ И ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
 * ============================================================================ */

// Ограничение значения диапазоном
#define CLAMP(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))

// Абсолютное значение для float
#define ABS_F(x) ((x) < 0.0f ? -(x) : (x))

// Знак числа
#define SIGN(x) ((x) > 0.0f ? 1.0f : ((x) < 0.0f ? -1.0f : 0.0f))

// Константы
#define GRAVITY_M_S2          9.81f
#define CENTER_POSITION       6000.0f  // Центральное положение (12000/2)
#define CENTER_POSITION_MM    4.0f     // 8мм / 2

/* ============================================================================
 * ЛОКАЛЬНЫЕ ПЕРЕМЕННЫЕ ПО УМОЛЧАНИЮ
 * ============================================================================ */

static const SpoolConfig_t default_config = {
    .dt_ms = SIMULATION_DT_MS,
    .valve_response_time_ms = VALVE_RESPONSE_TIME_MS,
    .hydraulic_delay_ms = HYDRAULIC_DELAY_MS,
    .damping_coefficient = SYSTEM_DAMPING,
    .mass_kg = 0.5f  // Примерная масса золотника
};

/* ============================================================================
 * СТАТИЧЕСКИЕ ФУНКЦИИ
 * ============================================================================ */

/**
 * @brief Расчет целевого открытия клапанов на основе ШИМ
 * 
 * Логика работы:
 * - При PWM < 20%: оба клапана закрыты, золотник в центре
 * - При 20% <= PWM <= 50%: правый клапан открывается, давление справа толкает золотник влево
 * - При 50% < PWM <= 80%: левый клапан открывается, давление слева толкает золотник вправо
 * - При PWM > 80%: золотник упирается в упор
 */
static void calculate_valve_targets(SpoolSimulator_t* sim, uint16_t pwm_value) {
    float normalized_pwm;
    float left_target = 0.0f;
    float right_target = 0.0f;
    
    // Нормализация PWM к диапазону 0-1
    normalized_pwm = (float)pwm_value / (float)PWM_MAX_VALUE;
    
    if (normalized_pwm <= 0.2f) {
        // Ниже порога страгивания - оба клапана закрыты
        left_target = 0.0f;
        right_target = 0.0f;
    } else if (normalized_pwm <= 0.5f) {
        // Движение влево (правый клапан открывает подачу давления справа)
        // Давление справа толкает золотник влево (уменьшает позицию)
        float range = 0.3f; // от 0.2 до 0.5
        float current = normalized_pwm - 0.2f;
        left_target = 0.0f;
        right_target = current / range;
    } else if (normalized_pwm <= 0.8f) {
        // Движение вправо (левый клапан открывает подачу давления слева)
        // Давление слева толкает золотник вправо (увеличивает позицию)
        float range = 0.3f; // от 0.5 до 0.8
        float current = normalized_pwm - 0.5f;
        left_target = current / range;
        right_target = 0.0f;
    } else {
        // Выше 80% - полное открытие активного клапана
        if (sim->spool_position < CENTER_POSITION) {
            left_target = 1.0f;
            right_target = 0.0f;
        } else {
            left_target = 0.0f;
            right_target = 1.0f;
        }
    }
    
    sim->valve_target_left = left_target;
    sim->valve_target_right = right_target;
}

/**
 * @brief Моделирование динамики открытия клапана (первый порядок)
 */
static float update_valve_dynamics(float current_open, float target_open, 
                                   float time_constant, float dt) {
    float tau = time_constant / 1000.0f; // Перевод в секунды
    float alpha = dt / (tau + dt);
    
    return current_open + alpha * (target_open - current_open);
}

/**
 * @brief Расчет давления в камере на основе открытия клапана
 */
static float calculate_chamber_pressure(float valve_open, float current_pressure,
                                        float dt, bool is_left) {
    float target_pressure;
    float pressure_change_rate;
    
    // Целевое давление при полном открытии клапана
    if (valve_open > 0.0f) {
        target_pressure = SYSTEM_PRESSURE_PA;
    } else {
        target_pressure = 0.0f; // Слив в бак
    }
    
    // Скорость изменения давления зависит от сжимаемости масла и объема трубки
    // dP/dt = (B/V) * Q, где Q - расход через клапан
    // Увеличиваем коэффициент расхода для более быстрого нарастания давления
    float flow_coefficient = valve_open * 0.001f; // Коэффициент расхода м³/с (увеличен в 10 раз)
    float bulk_modulus_volume = OIL_BULK_MODULUS_PA / TUBE_VOLUME_M3;
    
    pressure_change_rate = bulk_modulus_volume * flow_coefficient;
    
    // Ограничение скорости изменения давления
    // Увеличиваем максимальную скорость для более реалистичной динамики
    float max_pressure_change = SYSTEM_PRESSURE_PA / (HYDRAULIC_DELAY_MS / 1000.0f);
    pressure_change_rate = CLAMP(pressure_change_rate, -max_pressure_change, max_pressure_change);
    
    // Интегрирование давления
    float new_pressure = current_pressure;
    
    if (valve_open > 0.0f) {
        // Нагнетание давления - используем экспоненциальное нарастание
        float pressure_diff = target_pressure - current_pressure;
        float rise_rate = pressure_diff * valve_open * 50.0f; // 50 Гц - скорость нарастания
        rise_rate = CLAMP(rise_rate, 0.0f, max_pressure_change);
        new_pressure = current_pressure + rise_rate * dt;
        new_pressure = CLAMP(new_pressure, 0.0f, target_pressure);
    } else {
        // Сброс давления (слив) - экспоненциальный спад
        float decay_rate = current_pressure * 20.0f; // 20 Гц - скорость сброса
        decay_rate = CLAMP(decay_rate, 0.0f, max_pressure_change);
        new_pressure = current_pressure - decay_rate * dt;
        new_pressure = CLAMP(new_pressure, 0.0f, current_pressure);
    }
    
    return new_pressure;
}

/**
 * @brief Расчет сил действующих на золотник
 */
static void calculate_forces(SpoolSimulator_t* sim) {
    // Сила от давления слева: F = P * A
    // Давление слева толкает золотник вправо (увеличивает позицию)
    sim->force_left = sim->pressure_left * PISTON_AREA_M2;
    
    // Сила от давления справа
    // Давление справа толкает золотник влево (уменьшает позицию)
    sim->force_right = sim->pressure_right * PISTON_AREA_M2;
    
    // Результирующая сила от давления (слева толкает вправо+, справа - влево-)
    float force_pressure = sim->force_left - sim->force_right;
    
    // Сила пружины (возвращает в центральное положение)
    float displacement_m = (sim->spool_position - CENTER_POSITION) / SPOOL_POS_PER_MM / 1000.0f;
    sim->force_spring = -SPRING_STIFFNESS_N_M * displacement_m;
    
    // Ограничение силы пружины максимальным значением
    sim->force_spring = CLAMP(sim->force_spring, -SPRING_FORCE_MAX_N, SPRING_FORCE_MAX_N);
    
    // Сила демпфирования (вязкое трение)
    sim->force_damping = -default_config.damping_coefficient * sim->spool_velocity / SPOOL_POS_PER_MM;
    
    // Полная результирующая сила
    sim->force_net = force_pressure + sim->force_spring + sim->force_damping;
}

/**
 * @brief Обновление позиции золотника на основе сил
 */
static void update_spool_position(SpoolSimulator_t* sim, float dt) {
    float mass = default_config.mass_kg;
    float acceleration;
    float velocity;
    float position;
    
    // Проверка ограничений
    sim->at_limit_left = (sim->spool_position <= SPOOL_POS_MIN + 10.0f);
    sim->at_limit_right = (sim->spool_position >= SPOOL_POS_MAX - 10.0f);
    
    // Если достигнут предел и сила направлена дальше - блокируем движение
    if (sim->at_limit_left && sim->force_net < 0.0f) {
        sim->spool_velocity = 0.0f;
        sim->spool_acceleration = 0.0f;
        sim->spool_position = SPOOL_POS_MIN;
        sim->spool_moving = false;
        return;
    }
    
    if (sim->at_limit_right && sim->force_net > 0.0f) {
        sim->spool_velocity = 0.0f;
        sim->spool_acceleration = 0.0f;
        sim->spool_position = SPOOL_POS_MAX;
        sim->spool_moving = false;
        return;
    }
    
    // Второй закон Ньютона: F = m*a
    // Увеличиваем силу для более заметного движения (уменьшаем эффективную массу)
    float effective_mass = mass * 0.1f; // Эффективная масса меньше для лучшей динамики
    acceleration = sim->force_net / effective_mass;
    
    // Интегрирование ускорения для получения скорости
    velocity = sim->spool_velocity + acceleration * dt;
    
    // Применение небольшого демпфирования к скорости для стабильности
    velocity *= 0.95f;
    
    // Интегрирование скорости для получения позиции
    position = sim->spool_position + velocity * dt;
    
    // Ограничение позиции диапазоном
    position = CLAMP(position, SPOOL_POS_MIN, SPOOL_POS_MAX);
    
    // Обновление состояния
    sim->spool_acceleration = acceleration;
    sim->spool_velocity = velocity;
    sim->spool_position = position;
    
    // Определение движения
    sim->spool_moving = (ABS_F(velocity) > 1.0f);
}

/* ============================================================================
 * ПУБЛИЧНЫЕ ФУНКЦИИ
 * ============================================================================ */

bool SpoolSimulator_Init(SpoolSimulator_t* sim, const SpoolConfig_t* config) {
    if (sim == NULL) {
        return false;
    }
    
    // Очистка структуры
    memset(sim, 0, sizeof(SpoolSimulator_t));
    
    // Начальное состояние
    sim->pwm_value = 0;
    sim->valve_open_left = 0.0f;
    sim->valve_open_right = 0.0f;
    sim->valve_target_left = 0.0f;
    sim->valve_target_right = 0.0f;
    sim->pressure_left = 0.0f;
    sim->pressure_right = 0.0f;
    sim->spool_position = CENTER_POSITION;  // Центральное положение
    sim->spool_velocity = 0.0f;
    sim->spool_acceleration = 0.0f;
    sim->force_left = 0.0f;
    sim->force_right = 0.0f;
    sim->force_spring = 0.0f;
    sim->force_damping = 0.0f;
    sim->force_net = 0.0f;
    sim->last_update_time = 0;
    sim->simulation_time = 0.0f;
    sim->initialized = true;
    sim->spool_moving = false;
    sim->at_limit_left = false;
    sim->at_limit_right = false;
    
    return true;
}

void SpoolSimulator_Reset(SpoolSimulator_t* sim) {
    if (sim == NULL || !sim->initialized) {
        return;
    }
    
    SpoolSimulator_Init(sim, &default_config);
}

void SpoolSimulator_Update(SpoolSimulator_t* sim, uint16_t pwm_value, float dt_ms) {
    if (sim == NULL || !sim->initialized) {
        return;
    }
    
    // Ограничение шага времени
    dt_ms = CLAMP(dt_ms, 0.1f, 100.0f);
    float dt_sec = dt_ms / 1000.0f;
    
    // Обновление входного значения
    sim->pwm_value = CLAMP(pwm_value, PWM_MIN_VALUE, PWM_MAX_VALUE);
    
    // Расчет целевых положений клапанов
    calculate_valve_targets(sim, sim->pwm_value);
    
    // Обновление динамики клапанов (инерция открытия/закрытия)
    sim->valve_open_left = update_valve_dynamics(
        sim->valve_open_left, 
        sim->valve_target_left,
        default_config.valve_response_time_ms,
        dt_ms
    );
    
    sim->valve_open_right = update_valve_dynamics(
        sim->valve_open_right,
        sim->valve_target_right,
        default_config.valve_response_time_ms,
        dt_ms
    );
    
    // Расчет давлений в камерах
    sim->pressure_left = calculate_chamber_pressure(
        sim->valve_open_left,
        sim->pressure_left,
        dt_sec,
        true
    );
    
    sim->pressure_right = calculate_chamber_pressure(
        sim->valve_open_right,
        sim->pressure_right,
        dt_sec,
        false
    );
    
    // Расчет сил
    calculate_forces(sim);
    
    // Обновление позиции золотника
    update_spool_position(sim, dt_sec);
    
    // Обновление времени симуляции
    sim->simulation_time += dt_sec;
}

uint16_t SpoolSimulator_GetPosition(const SpoolSimulator_t* sim) {
    if (sim == NULL || !sim->initialized) {
        return CENTER_POSITION;
    }
    
    uint16_t pos = (uint16_t)(sim->spool_position + 0.5f);
    return CLAMP(pos, SPOOL_POS_MIN, SPOOL_POS_MAX);
}

float SpoolSimulator_GetPositionMm(const SpoolSimulator_t* sim) {
    if (sim == NULL || !sim->initialized) {
        return CENTER_POSITION_MM;
    }
    
    return sim->spool_position / SPOOL_POS_PER_MM;
}

float SpoolSimulator_GetVelocity(const SpoolSimulator_t* sim) {
    if (sim == NULL || !sim->initialized) {
        return 0.0f;
    }
    
    return sim->spool_velocity;
}

float SpoolSimulator_PWMToValveOpen(uint16_t pwm_value) {
    float normalized = (float)pwm_value / (float)PWM_MAX_VALUE;
    
    if (normalized <= 0.2f) {
        return 0.0f;
    } else if (normalized <= 0.8f) {
        return (normalized - 0.2f) / 0.6f;
    } else {
        return 1.0f;
    }
}

float SpoolSimulator_PositionToMm(uint16_t position) {
    return (float)position / SPOOL_POS_PER_MM;
}

uint16_t SpoolSimulator_MmToPosition(float mm) {
    mm = CLAMP(mm, 0.0f, SPOOL_STROKE_MM);
    return (uint16_t)(mm * SPOOL_POS_PER_MM + 0.5f);
}

bool SpoolSimulator_IsInitialized(const SpoolSimulator_t* sim) {
    return (sim != NULL && sim->initialized);
}

bool SpoolSimulator_IsMoving(const SpoolSimulator_t* sim) {
    if (sim == NULL || !sim->initialized) {
        return false;
    }
    
    return sim->spool_moving;
}

float SpoolSimulator_GetPressureLeft(const SpoolSimulator_t* sim) {
    if (sim == NULL || !sim->initialized) {
        return 0.0f;
    }
    
    return sim->pressure_left;
}

float SpoolSimulator_GetPressureRight(const SpoolSimulator_t* sim) {
    if (sim == NULL || !sim->initialized) {
        return 0.0f;
    }
    
    return sim->pressure_right;
}
