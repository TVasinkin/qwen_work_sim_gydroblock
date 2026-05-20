#include <stdio.h>
#include "spool_simulator.h"

int main() {
    SpoolSimulator_t sim;
    SpoolSimulator_Init(&sim, NULL);
    
    printf("Force analysis:\n");
    printf("Pressure force at 25 bar: %.1f N\n", SYSTEM_PRESSURE_PA * SPOOL_AREA_M2);
    printf("Spring force at 4mm: %.1f N\n", SPRING_FORCE_N);
    printf("Net force at full extension: %.1f N\n", 
           SYSTEM_PRESSURE_PA * SPOOL_AREA_M2 - SPRING_FORCE_N);
    
    // Запускаем симуляцию с пошаговым выводом
    for (int i = 0; i < 10000; i++) {
        SpoolSimulator_Step(&sim, PWM_MAX, 0, 0.001f);
        if (i % 1000 == 0) {
            printf("Step %d: pos=%.1fmm (%d), vel=%.3f m/s\n", 
                   i, 
                   SpoolSimulator_GetPositionMm(&sim),
                   SpoolSimulator_GetPosition(&sim),
                   SpoolSimulator_GetVelocity(&sim));
        }
    }
    
    printf("\nFinal: pos=%.2fmm (%d)\n", 
           SpoolSimulator_GetPositionMm(&sim),
           SpoolSimulator_GetPosition(&sim));
    
    return 0;
}
