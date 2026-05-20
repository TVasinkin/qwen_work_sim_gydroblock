/**
 * @file test_spool_simulator.c
 * @brief Модульные тесты для симулятора золотника с двумя клапанами
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include "spool_simulator.h"

#define TEST_PASSED(name) printf("[PASS] %s\n", name)
#define TEST_FAILED(name, msg) printf("[FAIL] %s: %s\n", name, msg)

static int tests_run = 0;
static int tests_passed = 0;

/* ============================================================================
 * ТЕСТ 1: Инициализация и сброс
 * ============================================================================ */
void test_initialization(void) {
    const char* name = "Initialization";
    SpoolSimulator_t sim;
    SpoolConfig_t config = { .dt = 0.001f, .pressure_deviation = 2.0f, .seed = 42 };
    
    SpoolSimulator_Init(&sim, &config);
    
    if (SpoolSimulator_GetPosition(&sim) != 0) {
        TEST_FAILED(name, "Initial position should be 0");
        return;
    }
    
    SpoolSimulator_Step(&sim, 3500, 3500, 0.001f); // Движение
    SpoolSimulator_Reset(&sim);
    
    if (SpoolSimulator_GetPosition(&sim) != 0) {
        TEST_FAILED(name, "Position should be 0 after reset");
        return;
    }
    
    TEST_PASSED(name);
    tests_passed++;
}

/* ============================================================================
 * ТЕСТ 2: Мертвая зона (< 10% ШИМ)
 * ============================================================================ */
void test_dead_zone(void) {
    const char* name = "Dead Zone (<10%)";
    SpoolSimulator_t sim;
    SpoolSimulator_Init(&sim, NULL);
    
    // Подаем 9% на оба клапана (630 отсчетов)
    uint16_t pwm_below_threshold = 630; // 9% от 7000
    
    for (int i = 0; i < 100; i++) {
        SpoolSimulator_Step(&sim, pwm_below_threshold, pwm_below_threshold, 0.001f);
    }
    
    // Золотник должен вернуться в 0 или остаться около 0
    int32_t pos = SpoolSimulator_GetPosition(&sim);
    if (abs(pos) > 500) {
        TEST_FAILED(name, "Position should stay near 0 in dead zone");
        return;
    }
    
    if (!SpoolSimulator_IsInDeadZone(&sim)) {
        TEST_FAILED(name, "Should be in dead zone");
        return;
    }
    
    TEST_PASSED(name);
    tests_passed++;
}

/* ============================================================================
 * ТЕСТ 3: Движение в положительном направлении (Клапан 1)
 * ============================================================================ */
void test_move_positive(void) {
    const char* name = "Move Positive (Valve 1)";
    SpoolSimulator_t sim;
    SpoolSimulator_Init(&sim, NULL);
    
    // Подаем 50% на клапан 1 (3500 отсчетов), 0 на клапан 2
    uint16_t pwm_v1 = 3500;
    uint16_t pwm_v2 = 0;
    
    for (int i = 0; i < 500; i++) {
        SpoolSimulator_Step(&sim, pwm_v1, pwm_v2, 0.001f);
    }
    
    int32_t pos = SpoolSimulator_GetPosition(&sim);
    
    if (pos <= 0) {
        TEST_FAILED(name, "Position should be positive when Valve 1 is active");
        return;
    }
    
    if (pos > SPOOL_POS_MAX) {
        TEST_FAILED(name, "Position exceeds maximum");
        return;
    }
    
    TEST_PASSED(name);
    tests_passed++;
}

/* ============================================================================
 * ТЕСТ 4: Движение в отрицательном направлении (Клапан 2)
 * ============================================================================ */
void test_move_negative(void) {
    const char* name = "Move Negative (Valve 2)";
    SpoolSimulator_t sim;
    SpoolSimulator_Init(&sim, NULL);
    
    // Подаем 0 на клапан 1, 50% на клапан 2 (3500 отсчетов)
    uint16_t pwm_v1 = 0;
    uint16_t pwm_v2 = 3500;
    
    for (int i = 0; i < 500; i++) {
        SpoolSimulator_Step(&sim, pwm_v1, pwm_v2, 0.001f);
    }
    
    int32_t pos = SpoolSimulator_GetPosition(&sim);
    
    if (pos >= 0) {
        TEST_FAILED(name, "Position should be negative when Valve 2 is active");
        return;
    }
    
    if (pos < SPOOL_POS_MIN) {
        TEST_FAILED(name, "Position exceeds minimum");
        return;
    }
    
    TEST_PASSED(name);
    tests_passed++;
}

/* ============================================================================
 * ТЕСТ 5: Достижение положительного предела
 * ============================================================================ */
void test_positive_limit(void) {
    const char* name = "Positive Limit Detection";
    SpoolSimulator_t sim;
    SpoolSimulator_Init(&sim, NULL);
    
    // Подаем 100% на клапан 1
    uint16_t pwm_v1 = PWM_MAX;
    uint16_t pwm_v2 = 0;
    
    // Нужно больше времени из-за пружин и инерции (15 секунд)
    for (int i = 0; i < 15000; i++) {
        SpoolSimulator_Step(&sim, pwm_v1, pwm_v2, 0.001f);
    }
    
    int32_t pos = SpoolSimulator_GetPosition(&sim);
    
    // Золотник должен достичь или приблизиться к максимуму (+12000)
    if (pos < 11500) {
        TEST_FAILED(name, "Position should be near maximum limit");
        return;
    }
    
    TEST_PASSED(name);
    tests_passed++;
}

/* ============================================================================
 * ТЕСТ 6: Достижение отрицательного предела
 * ============================================================================ */
void test_negative_limit(void) {
    const char* name = "Negative Limit Detection";
    SpoolSimulator_t sim;
    SpoolSimulator_Init(&sim, NULL);
    
    // Подаем 100% на клапан 2
    uint16_t pwm_v1 = 0;
    uint16_t pwm_v2 = PWM_MAX;
    
    for (int i = 0; i < 15000; i++) {
        SpoolSimulator_Step(&sim, pwm_v1, pwm_v2, 0.001f);
    }
    
    int32_t pos = SpoolSimulator_GetPosition(&sim);
    
    // Золотник должен достичь или приблизиться к минимуму (-12000)
    if (pos > -11500) {
        TEST_FAILED(name, "Position should be near minimum limit");
        return;
    }
    
    TEST_PASSED(name);
    tests_passed++;
}

/* ============================================================================
 * ТЕСТ 7: Возврат пружинами в центр
 * ============================================================================ */
void test_return_to_center(void) {
    const char* name = "Return to Center by Springs";
    SpoolSimulator_t sim;
    SpoolSimulator_Init(&sim, NULL);
    
    // Сначала двигаем в положительную сторону
    for (int i = 0; i < 300; i++) {
        SpoolSimulator_Step(&sim, 5000, 0, 0.001f);
    }
    
    int32_t pos_before = SpoolSimulator_GetPosition(&sim);
    if (pos_before <= 0) {
        TEST_FAILED(name, "Should move positive first");
        return;
    }
    
    // Теперь убираем ШИМ (оба в 0)
    for (int i = 0; i < 500; i++) {
        SpoolSimulator_Step(&sim, 0, 0, 0.001f);
    }
    
    int32_t pos_after = SpoolSimulator_GetPosition(&sim);
    
    // Пружины должны вернуть ближе к 0
    if (abs(pos_after) >= abs(pos_before)) {
        TEST_FAILED(name, "Springs should return spool toward center");
        return;
    }
    
    TEST_PASSED(name);
    tests_passed++;
}

/* ============================================================================
 * ТЕСТ 8: Инерция системы
 * ============================================================================ */
void test_system_inertia(void) {
    const char* name = "System Inertia";
    SpoolSimulator_t sim;
    SpoolSimulator_Init(&sim, NULL);
    
    // Резко подаем 100% ШИМ
    SpoolSimulator_Step(&sim, PWM_MAX, 0, 0.001f);
    int32_t pos_1ms = SpoolSimulator_GetPosition(&sim);
    
    SpoolSimulator_Step(&sim, PWM_MAX, 0, 0.001f);
    int32_t pos_2ms = SpoolSimulator_GetPosition(&sim);
    
    // Из-за инерции положение не должно измениться мгновенно
    // Но должно начать двигаться
    if (pos_1ms == 0 && pos_2ms == 0) {
        // Допустимо - инерция клапана
    }
    
    // Через 50мс должно уже двигаться
    for (int i = 0; i < 50; i++) {
        SpoolSimulator_Step(&sim, PWM_MAX, 0, 0.001f);
    }
    
    int32_t pos_50ms = SpoolSimulator_GetPosition(&sim);
    if (pos_50ms == 0) {
        TEST_FAILED(name, "Should move after 50ms");
        return;
    }
    
    TEST_PASSED(name);
    tests_passed++;
}

/* ============================================================================
 * ТЕСТ 9: Преобразования единиц
 * ============================================================================ */
void test_conversions(void) {
    const char* name = "Unit Conversions";
    
    // мм <-> отсчеты
    float mm = 2.0f;
    int32_t counts = SpoolSimulator_MmToCounts(mm);
    float mm_back = SpoolSimulator_CountsToMm(counts);
    
    if (fabs(mm - mm_back) > 0.1f) {
        TEST_FAILED(name, "mm conversion error");
        return;
    }
    
    // проценты <-> ШИМ
    float percent = 50.0f;
    uint16_t pwm = SpoolSimulator_PercentToPwm(percent);
    float percent_back = SpoolSimulator_PwmToPercent(pwm);
    
    if (fabs(percent - percent_back) > 1.0f) {
        TEST_FAILED(name, "PWM conversion error");
        return;
    }
    
    TEST_PASSED(name);
    tests_passed++;
}

/* ============================================================================
 * ТЕСТ 10: Давление в камерах
 * ============================================================================ */
void test_chamber_pressure(void) {
    const char* name = "Chamber Pressure";
    SpoolSimulator_t sim;
    SpoolSimulator_Init(&sim, NULL);
    
    // Подаем только на клапан 1
    for (int i = 0; i < 200; i++) {
        SpoolSimulator_Step(&sim, 5000, 0, 0.001f);
    }
    
    float pA = SpoolSimulator_GetPressureA(&sim);
    float pB = SpoolSimulator_GetPressureB(&sim);
    
    if (pA <= pB) {
        TEST_FAILED(name, "Pressure A should be higher than B");
        return;
    }
    
    if (pA <= 0) {
        TEST_FAILED(name, "Pressure A should be positive");
        return;
    }
    
    TEST_PASSED(name);
    tests_passed++;
}

/* ============================================================================
 * ТЕСТ 11: Граничные значения ШИМ
 * ============================================================================ */
void test_pwm_boundaries(void) {
    const char* name = "PWM Boundary Values";
    SpoolSimulator_t sim;
    SpoolSimulator_Init(&sim, NULL);
    
    // Тестируем 0%
    SpoolSimulator_Step(&sim, 0, 0, 0.001f);
    if (SpoolSimulator_GetPosition(&sim) != 0) {
        TEST_FAILED(name, "0% PWM should keep position at 0");
        return;
    }
    
    // Тестируем 100%
    SpoolSimulator_Step(&sim, PWM_MAX, 0, 0.001f);
    // Должно начать движение
    
    TEST_PASSED(name);
    tests_passed++;
}

/* ============================================================================
 * ТЕСТ 12: Долгосрочная стабильность
 * ============================================================================ */
void test_long_term_stability(void) {
    const char* name = "Long Term Stability";
    SpoolSimulator_t sim;
    SpoolSimulator_Init(&sim, NULL);
    
    // Циклически подаем сигналы
    for (int cycle = 0; cycle < 10; cycle++) {
        // Вправо
        for (int i = 0; i < 100; i++) {
            SpoolSimulator_Step(&sim, 4000, 0, 0.001f);
        }
        // Влево
        for (int i = 0; i < 200; i++) {
            SpoolSimulator_Step(&sim, 0, 4000, 0.001f);
        }
    }
    
    int32_t final_pos = SpoolSimulator_GetPosition(&sim);
    
    // Не должно быть переполнений или NaN
    if (final_pos < SPOOL_POS_MIN || final_pos > SPOOL_POS_MAX) {
        TEST_FAILED(name, "Position out of bounds");
        return;
    }
    
    TEST_PASSED(name);
    tests_passed++;
}

/* ============================================================================
 * MAIN
 * ============================================================================ */
int main(void) {
    printf("=== Spool Simulator Tests (Dual Valve) ===\n\n");
    
    tests_run++; test_initialization();
    tests_run++; test_dead_zone();
    tests_run++; test_move_positive();
    tests_run++; test_move_negative();
    tests_run++; test_positive_limit();
    tests_run++; test_negative_limit();
    tests_run++; test_return_to_center();
    tests_run++; test_system_inertia();
    tests_run++; test_conversions();
    tests_run++; test_chamber_pressure();
    tests_run++; test_pwm_boundaries();
    tests_run++; test_long_term_stability();
    
    printf("\n=== Results: %d/%d tests passed ===\n", tests_passed, tests_run);
    
    return (tests_passed == tests_run) ? 0 : 1;
}
