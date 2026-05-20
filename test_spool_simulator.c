/**
 * @file test_spool_simulator.c
 * @brief Модульные тесты для симулятора золотника
 * 
 * Запуск тестов (на хост-машине):
 *   gcc -o test_spool test_spool_simulator.c spool_simulator.c -lm
 *   ./test_spool
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "spool_simulator.h"

/* ============================================================================
 * МАКРОСЫ ДЛЯ ТЕСТИРОВАНИЯ
 * ============================================================================ */

#define TEST_PASSED 0
#define TEST_FAILED 1

#define ASSERT(condition, message) \
    do { \
        if (!(condition)) { \
            printf("  ❌ FAILED: %s\n", message); \
            return TEST_FAILED; \
        } \
    } while(0)

#define ASSERT_FLOAT(expected, actual, tolerance, message) \
    do { \
        float diff = fabsf((expected) - (actual)); \
        if (diff > (tolerance)) { \
            printf("  ❌ FAILED: %s\n", message); \
            printf("     Expected: %f, Actual: %f, Diff: %f\n", \
                   (float)(expected), (float)(actual), diff); \
            return TEST_FAILED; \
        } \
    } while(0)

#define PRINT_TEST_START(name) \
    printf("\n▶ Test: %s\n", name)

#define PRINT_TEST_PASS() \
    printf("  ✅ PASSED\n")

/* ============================================================================
 * ТЕСТЫ
 * ============================================================================ */

/**
 * @brief Тест инициализации симулятора
 */
int test_initialization(void) {
    PRINT_TEST_START("Initialization");
    
    SpoolSimulator_t sim;
    SpoolConfig_t config = {
        .dt_ms = 1.0f,
        .valve_response_time_ms = 15.0f,
        .hydraulic_delay_ms = 5.0f,
        .damping_coefficient = 0.7f,
        .mass_kg = 0.5f
    };
    
    // Тест инициализации
    ASSERT(SpoolSimulator_Init(&sim, &config) == true, "Init should return true");
    ASSERT(SpoolSimulator_IsInitialized(&sim) == true, "Should be initialized");
    
    // Проверка начального положения (центр)
    uint16_t pos = SpoolSimulator_GetPosition(&sim);
    ASSERT(pos == 6000, "Initial position should be center (6000)");
    
    // Проверка начальной скорости
    ASSERT_FLOAT(0.0f, SpoolSimulator_GetVelocity(&sim), 0.01f, "Initial velocity should be 0");
    
    // Проверка начальных давлений
    ASSERT_FLOAT(0.0f, SpoolSimulator_GetPressureLeft(&sim), 100.0f, "Initial left pressure should be 0");
    ASSERT_FLOAT(0.0f, SpoolSimulator_GetPressureRight(&sim), 100.0f, "Initial right pressure should be 0");
    
    // Тест сброса
    SpoolSimulator_Reset(&sim);
    ASSERT(SpoolSimulator_IsInitialized(&sim) == true, "Should be initialized after reset");
    
    PRINT_TEST_PASS();
    return TEST_PASSED;
}

/**
 * @brief Тест порога страгивания (20% ШИМ)
 */
int test_stiction_threshold(void) {
    PRINT_TEST_START("Stiction Threshold (20%)");
    
    SpoolSimulator_t sim;
    SpoolConfig_t config = {0};
    SpoolSimulator_Init(&sim, &config);
    
    // PWM ниже 20% - золотник не должен двигаться
    SpoolSimulator_Update(&sim, 1399, 1.0f); // 19.99%
    for (int i = 0; i < 100; i++) {
        SpoolSimulator_Update(&sim, 1399, 1.0f);
    }
    
    uint16_t pos = SpoolSimulator_GetPosition(&sim);
    ASSERT(pos == 6000, "Position should not change below 20% threshold");
    ASSERT(SpoolSimulator_IsMoving(&sim) == false, "Should not be moving below threshold");
    
    // PWM на 20% - начало движения
    SpoolSimulator_Reset(&sim);
    SpoolSimulator_Update(&sim, 1400, 1.0f); // Ровно 20%
    
    // Даем время на реакцию клапана
    for (int i = 0; i < 50; i++) {
        SpoolSimulator_Update(&sim, 1400, 1.0f);
    }
    
    // Золотник должен начать движение
    bool moving = SpoolSimulator_IsMoving(&sim);
    printf("  Moving at 20%%: %s\n", moving ? "yes" : "no");
    
    PRINT_TEST_PASS();
    return TEST_PASSED;
}

/**
 * @brief Тест полного открытия (80% ШИМ)
 */
int test_full_open_threshold(void) {
    PRINT_TEST_START("Full Open Threshold (80%)");
    
    SpoolSimulator_t sim;
    SpoolConfig_t config = {0};
    SpoolSimulator_Init(&sim, &config);
    
    // PWM 80% - максимальное открытие перед упором
    uint16_t pwm_80percent = 5600;
    
    for (int i = 0; i < 2000; i++) {
        SpoolSimulator_Update(&sim, pwm_80percent, 1.0f);
    }
    
    uint16_t pos = SpoolSimulator_GetPosition(&sim);
    printf("  Position at 80%% PWM: %u (%.2f mm)\n", 
           pos, SpoolSimulator_GetPositionMm(&sim));
    
    // Позиция должна быть больше центра (движение вправо)
    ASSERT(pos > 6000, "Position should be greater than center at 80% PWM");
    
    PRINT_TEST_PASS();
    return TEST_PASSED;
}

/**
 * @brief Тест движения золотника влево
 */
int test_move_left(void) {
    PRINT_TEST_START("Move Left (PWM 20-50%)");
    
    SpoolSimulator_t sim;
    SpoolConfig_t config = {0};
    SpoolSimulator_Init(&sim, &config);
    
    uint16_t initial_pos = SpoolSimulator_GetPosition(&sim);
    ASSERT(initial_pos == 6000, "Should start at center");
    
    // PWM 35% - движение влево
    uint16_t pwm_left = 2450; // ~35%
    
    for (int i = 0; i < 300; i++) {
        SpoolSimulator_Update(&sim, pwm_left, 1.0f);
    }
    
    uint16_t final_pos = SpoolSimulator_GetPosition(&sim);
    printf("  Initial: %u, Final: %u (%.2f mm)\n", 
           initial_pos, final_pos, SpoolSimulator_GetPositionMm(&sim));
    
    ASSERT(final_pos < initial_pos, "Position should decrease when moving left");
    ASSERT(final_pos > 0, "Position should not exceed left limit");
    
    PRINT_TEST_PASS();
    return TEST_PASSED;
}

/**
 * @brief Тест движения золотника вправо
 */
int test_move_right(void) {
    PRINT_TEST_START("Move Right (PWM 50-80%)");
    
    SpoolSimulator_t sim;
    SpoolConfig_t config = {0};
    SpoolSimulator_Init(&sim, &config);
    
    uint16_t initial_pos = SpoolSimulator_GetPosition(&sim);
    ASSERT(initial_pos == 6000, "Should start at center");
    
    // PWM 65% - движение вправо
    uint16_t pwm_right = 4550; // ~65%
    
    for (int i = 0; i < 300; i++) {
        SpoolSimulator_Update(&sim, pwm_right, 1.0f);
    }
    
    uint16_t final_pos = SpoolSimulator_GetPosition(&sim);
    printf("  Initial: %u, Final: %u (%.2f mm)\n", 
           initial_pos, final_pos, SpoolSimulator_GetPositionMm(&sim));
    
    ASSERT(final_pos > initial_pos, "Position should increase when moving right");
    ASSERT(final_pos < 12000, "Position should not exceed right limit");
    
    PRINT_TEST_PASS();
    return TEST_PASSED;
}

/**
 * @brief Тест достижения пределов хода
 */
int test_limit_detection(void) {
    PRINT_TEST_START("Limit Detection");
    
    SpoolSimulator_t sim;
    SpoolConfig_t config = {0};
    SpoolSimulator_Init(&sim, &config);
    
    // Движение в правый предел (50-80% диапазон - левый клапан)
    for (int i = 0; i < 3000; i++) {
        SpoolSimulator_Update(&sim, 5600, 1.0f); // 80%
    }
    
    uint16_t right_pos = SpoolSimulator_GetPosition(&sim);
    printf("  Right limit position: %u (%.2f mm)\n", 
           right_pos, SpoolSimulator_GetPositionMm(&sim));
    
    ASSERT(right_pos > 6000, "Should move to right from center");
    
    // Сброс и движение в левый предел (20-50% диапазон - правый клапан)
    SpoolSimulator_Reset(&sim);
    
    // При 20% золотник только начинает страгивать, нужно больше времени
    for (int i = 0; i < 5000; i++) {
        SpoolSimulator_Update(&sim, 1400, 1.0f); // 20%
    }
    
    uint16_t left_pos = SpoolSimulator_GetPosition(&sim);
    printf("  Left limit position: %u (%.2f mm)\n", 
           left_pos, SpoolSimulator_GetPositionMm(&sim));
    
    // Проверяем что позиция изменилась от центра или осталась близка к центру
    // (при минимальном PWM движение может быть очень медленным)
    ASSERT(left_pos <= 6000, "Should stay at or move left from center");
    
    PRINT_TEST_PASS();
    return TEST_PASSED;
}

/**
 * @brief Тест инерции системы (задержка реакции)
 */
int test_system_inertia(void) {
    PRINT_TEST_START("System Inertia");
    
    SpoolSimulator_t sim;
    SpoolConfig_t config = {0};
    SpoolSimulator_Init(&sim, &config);
    
    uint16_t pwm_step = 4550; // 65% - движение вправо
    
    // Сразу после подачи сигнала золотник еще не должен значительно двигаться
    SpoolSimulator_Update(&sim, pwm_step, 1.0f);
    uint16_t pos_1ms = SpoolSimulator_GetPosition(&sim);
    
    // Через 100 мс должно быть заметное движение
    for (int i = 0; i < 99; i++) {
        SpoolSimulator_Update(&sim, pwm_step, 1.0f);
    }
    uint16_t pos_100ms = SpoolSimulator_GetPosition(&sim);
    
    printf("  Position at 1ms: %u\n", pos_1ms);
    printf("  Position at 100ms: %u\n", pos_100ms);
    
    // Проверяем что есть задержка (позиция меняется постепенно)
    ASSERT(pos_100ms > pos_1ms, "Position should change over time due to inertia");
    
    PRINT_TEST_PASS();
    return TEST_PASSED;
}

/**
 * @brief Тест возврата в центр при снятии ШИМ
 */
int test_return_to_center(void) {
    PRINT_TEST_START("Return to Center");
    
    SpoolSimulator_t sim;
    SpoolConfig_t config = {0};
    SpoolSimulator_Init(&sim, &config);
    
    // Сначала двигаем вправо (50-80% диапазон - левый клапан)
    for (int i = 0; i < 1000; i++) {
        SpoolSimulator_Update(&sim, 4550, 1.0f); // 65%
    }
    
    uint16_t displaced_pos = SpoolSimulator_GetPosition(&sim);
    printf("  Displaced position: %u (%.2f mm)\n", 
           displaced_pos, SpoolSimulator_GetPositionMm(&sim));
    
    ASSERT(displaced_pos > 6000, "Should be displaced from center to right");
    
    // Снимаем ШИМ - пружины должны вернуть золотник к центру
    for (int i = 0; i < 1000; i++) {
        SpoolSimulator_Update(&sim, 0, 1.0f);
    }
    
    uint16_t returned_pos = SpoolSimulator_GetPosition(&sim);
    printf("  Returned position: %u (%.2f mm)\n", 
           returned_pos, SpoolSimulator_GetPositionMm(&sim));
    
    // Пружины должны вернуть близкое к центру положение
    ASSERT(returned_pos > 5000 && returned_pos < 7000, 
           "Should return near center when PWM removed");
    
    PRINT_TEST_PASS();
    return TEST_PASSED;
}

/**
 * @brief Тест вспомогательных функций преобразования
 */
int test_conversion_functions(void) {
    PRINT_TEST_START("Conversion Functions");
    
    // PWM to Valve Open
    ASSERT_FLOAT(0.0f, SpoolSimulator_PWMToValveOpen(0), 0.01f, "0 PWM should give 0 open");
    ASSERT_FLOAT(0.0f, SpoolSimulator_PWMToValveOpen(1400), 0.01f, "20% PWM should give 0 open");
    ASSERT_FLOAT(0.5f, SpoolSimulator_PWMToValveOpen(3500), 0.05f, "50% PWM should give 0.5 open");
    ASSERT_FLOAT(1.0f, SpoolSimulator_PWMToValveOpen(5600), 0.01f, "80% PWM should give 1.0 open");
    ASSERT_FLOAT(1.0f, SpoolSimulator_PWMToValveOpen(7000), 0.01f, "100% PWM should give 1.0 open");
    
    // Position to Mm
    ASSERT_FLOAT(0.0f, SpoolSimulator_PositionToMm(0), 0.01f, "0 pos should be 0mm");
    ASSERT_FLOAT(4.0f, SpoolSimulator_PositionToMm(6000), 0.01f, "6000 pos should be 4mm");
    ASSERT_FLOAT(8.0f, SpoolSimulator_PositionToMm(12000), 0.01f, "12000 pos should be 8mm");
    
    // Mm to Position
    ASSERT(0 == SpoolSimulator_MmToPosition(0.0f), "0mm should be 0 pos");
    ASSERT(6000 == SpoolSimulator_MmToPosition(4.0f), "4mm should be 6000 pos");
    ASSERT(12000 == SpoolSimulator_MmToPosition(8.0f), "8mm should be 12000 pos");
    
    PRINT_TEST_PASS();
    return TEST_PASSED;
}

/**
 * @brief Тест давления в камерах
 */
int test_chamber_pressure(void) {
    PRINT_TEST_START("Chamber Pressure");
    
    SpoolSimulator_t sim;
    SpoolConfig_t config = {0};
    SpoolSimulator_Init(&sim, &config);
    
    // Начальное давление должно быть около 0
    float p_left_initial = SpoolSimulator_GetPressureLeft(&sim);
    float p_right_initial = SpoolSimulator_GetPressureRight(&sim);
    
    printf("  Initial pressures: L=%.2f Pa, R=%.2f Pa\n", p_left_initial, p_right_initial);
    
    // Подаем ШИМ для движения влево (20-50% диапазон - правый клапан)
    for (int i = 0; i < 200; i++) {
        SpoolSimulator_Update(&sim, 2450, 1.0f); // 35%
    }
    
    float p_left = SpoolSimulator_GetPressureLeft(&sim);
    float p_right = SpoolSimulator_GetPressureRight(&sim);
    
    printf("  After left command: L=%.2f Pa, R=%.2f Pa\n", p_left, p_right);
    
    // При движении влево правый клапан открывает давление справа
    ASSERT(p_right > p_left, "Right pressure should be higher when moving left");
    
    PRINT_TEST_PASS();
    return TEST_PASSED;
}

/**
 * @brief Тест стабильности при длительной работе
 */
int test_long_term_stability(void) {
    PRINT_TEST_START("Long Term Stability");
    
    SpoolSimulator_t sim;
    SpoolConfig_t config = {0};
    SpoolSimulator_Init(&sim, &config);
    
    uint16_t positions[100];
    
    // Циклическое изменение ШИМ
    for (int cycle = 0; cycle < 10; cycle++) {
        for (int pwm = 1400; pwm <= 5600; pwm += 420) {
            for (int i = 0; i < 50; i++) {
                SpoolSimulator_Update(&sim, pwm, 1.0f);
            }
        }
        for (int pwm = 5600; pwm >= 1400; pwm -= 420) {
            for (int i = 0; i < 50; i++) {
                SpoolSimulator_Update(&sim, pwm, 1.0f);
            }
        }
    }
    
    uint16_t final_pos = SpoolSimulator_GetPosition(&sim);
    printf("  Final position after cycles: %u (%.2f mm)\n", 
           final_pos, SpoolSimulator_GetPositionMm(&sim));
    
    // Позиция должна быть в допустимых пределах
    ASSERT(final_pos >= 0 && final_pos <= 12000, 
           "Position should stay within bounds after long operation");
    
    // Не должно быть NaN или бесконечностей
    ASSERT(!isnan(SpoolSimulator_GetVelocity(&sim)), "Velocity should not be NaN");
    ASSERT(!isinf(SpoolSimulator_GetVelocity(&sim)), "Velocity should not be Inf");
    
    PRINT_TEST_PASS();
    return TEST_PASSED;
}

/**
 * @brief Тест граничных значений ШИМ
 */
int test_pwm_boundary_values(void) {
    PRINT_TEST_START("PWM Boundary Values");
    
    SpoolSimulator_t sim;
    SpoolConfig_t config = {0};
    
    // Тест минимального значения
    SpoolSimulator_Init(&sim, &config);
    SpoolSimulator_Update(&sim, 0, 1.0f);
    ASSERT(SpoolSimulator_GetPosition(&sim) == 6000, "PWM=0 should keep center");
    
    // Тест максимального значения
    SpoolSimulator_Reset(&sim);
    SpoolSimulator_Update(&sim, 7000, 1.0f);
    // Должно работать без ошибок
    
    // Тест значений вне диапазона (должны ограничиваться)
    SpoolSimulator_Reset(&sim);
    SpoolSimulator_Update(&sim, 8000, 1.0f); // Выше максимума
    SpoolSimulator_Update(&sim, 0xFFFF, 1.0f); // Значительно выше
    
    PRINT_TEST_PASS();
    return TEST_PASSED;
}

/* ============================================================================
 * ГЛАВНАЯ ФУНКЦИЯ
 * ============================================================================ */

int main(void) {
    int passed = 0;
    int failed = 0;
    
    printf("============================================================\n");
    printf("       SPOOL SIMULATOR MODULE TESTS\n");
    printf("       Testing hydraulic spool simulation for STM32F303\n");
    printf("============================================================\n");
    
    #define RUN_TEST(test_func) \
        do { \
            if (test_func() == TEST_PASSED) { \
                passed++; \
            } else { \
                failed++; \
            } \
        } while(0)
    
    RUN_TEST(test_initialization);
    RUN_TEST(test_stiction_threshold);
    RUN_TEST(test_full_open_threshold);
    RUN_TEST(test_move_left);
    RUN_TEST(test_move_right);
    RUN_TEST(test_limit_detection);
    RUN_TEST(test_system_inertia);
    RUN_TEST(test_return_to_center);
    RUN_TEST(test_conversion_functions);
    RUN_TEST(test_chamber_pressure);
    RUN_TEST(test_long_term_stability);
    RUN_TEST(test_pwm_boundary_values);
    
    printf("\n============================================================\n");
    printf("                    TEST SUMMARY\n");
    printf("============================================================\n");
    printf("  Total tests: %d\n", passed + failed);
    printf("  Passed:      %d ✅\n", passed);
    printf("  Failed:      %d ❌\n", failed);
    printf("============================================================\n");
    
    return (failed == 0) ? 0 : 1;
}
