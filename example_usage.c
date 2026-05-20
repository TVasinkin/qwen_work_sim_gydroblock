/**
 * @file example_usage.c
 * @brief Пример использования симулятора золотника с двумя клапанами
 */

#include <stdio.h>
#include "spool_simulator.h"

int main(void) {
    printf("=== Пример использования Spool Simulator ===\n\n");
    
    // 1. Инициализация
    SpoolSimulator_t sim;
    SpoolConfig_t config = {
        .dt = 0.001f,              // Шаг симуляции 1мс
        .pressure_deviation = 2.0f,// Нестабильность давления ±2%
        .seed = 42                 // Seed для ГСЧ
    };
    
    SpoolSimulator_Init(&sim, &config);
    
    printf("Начальное положение: %d отсчетов (%.2f мм)\n\n", 
           SpoolSimulator_GetPosition(&sim),
           SpoolSimulator_GetPositionMm(&sim));
    
    // 2. Движение в положительную сторону (Клапан 1)
    printf("Подаем 70% на Клапан 1...\n");
    uint16_t pwm_v1 = SpoolSimulator_PercentToPwm(70.0f); // ~4900
    uint16_t pwm_v2 = 0;
    
    for (int i = 0; i < 3000; i++) {
        SpoolSimulator_Step(&sim, pwm_v1, pwm_v2, 0.001f);
    }
    
    printf("Положение: %d отсчетов (%.2f мм)\n", 
           SpoolSimulator_GetPosition(&sim),
           SpoolSimulator_GetPositionMm(&sim));
    printf("Давление A: %.1f бар, Давление B: %.1f бар\n\n",
           SpoolSimulator_GetPressureA(&sim) / 1e5f,
           SpoolSimulator_GetPressureB(&sim) / 1e5f);
    
    // 3. Возврат в центр (оба клапана в 0)
    printf("Убираем ШИМ (оба клапана 0%%) - пружины возвращают в центр...\n");
    for (int i = 0; i < 2000; i++) {
        SpoolSimulator_Step(&sim, 0, 0, 0.001f);
    }
    
    printf("Положение: %d отсчетов (%.2f мм)\n\n", 
           SpoolSimulator_GetPosition(&sim),
           SpoolSimulator_GetPositionMm(&sim));
    
    // 4. Движение в отрицательную сторону (Клапан 2)
    printf("Подаем 50% на Клапан 2...\n");
    pwm_v1 = 0;
    pwm_v2 = SpoolSimulator_PercentToPwm(50.0f); // 3500
    
    for (int i = 0; i < 3000; i++) {
        SpoolSimulator_Step(&sim, pwm_v1, pwm_v2, 0.001f);
    }
    
    printf("Положение: %d отсчетов (%.2f мм)\n", 
           SpoolSimulator_GetPosition(&sim),
           SpoolSimulator_GetPositionMm(&sim));
    printf("Давление A: %.1f бар, Давление B: %.1f бар\n\n",
           SpoolSimulator_GetPressureA(&sim) / 1e5f,
           SpoolSimulator_GetPressureB(&sim) / 1e5f);
    
    // 5. Проверка мертвой зоны
    printf("Проверка мертвой зоны (<10%%):\n");
    printf("В мертвой зоне: %s\n\n",
           SpoolSimulator_IsInDeadZone(&sim) ? "Да" : "Нет");
    
    // 6. Преобразования
    printf("Преобразования:\n");
    printf("50%% ШИМ = %d отсчетов\n", SpoolSimulator_PercentToPwm(50.0f));
    printf("+2мм = %d отсчетов\n", SpoolSimulator_MmToCounts(2.0f));
    printf("-3мм = %d отсчетов\n", SpoolSimulator_MmToCounts(-3.0f));
    printf("6000 отсчетов = %.2f мм\n", SpoolSimulator_CountsToMm(6000));
    printf("-6000 отсчетов = %.2f мм\n", SpoolSimulator_CountsToMm(-6000));
    
    return 0;
}
