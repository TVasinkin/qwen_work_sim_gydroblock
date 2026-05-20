#include <stdio.h>
#include "spool_simulator.h"

int main() {
    SpoolSimulator_t sim;
    SpoolSimulator_Init(&sim, NULL);
    
    printf("Running 5000 steps with PWM_MAX on Valve 1...\n");
    for (int i = 0; i < 5000; i++) {
        SpoolSimulator_Step(&sim, PWM_MAX, 0, 0.001f);
    }
    
    printf("Position: %d\n", SpoolSimulator_GetPosition(&sim));
    printf("Position mm: %.2f\n", SpoolSimulator_GetPositionMm(&sim));
    printf("Pressure A: %.0f Pa (%.2f bar)\n", 
           SpoolSimulator_GetPressureA(&sim), 
           SpoolSimulator_GetPressureA(&sim) / 1e5f);
    printf("Pressure B: %.0f Pa (%.2f bar)\n", 
           SpoolSimulator_GetPressureB(&sim), 
           SpoolSimulator_GetPressureB(&sim) / 1e5f);
    printf("At positive limit: %s\n", 
           SpoolSimulator_IsAtPositiveLimit(&sim) ? "yes" : "no");
    printf("At negative limit: %s\n", 
           SpoolSimulator_IsAtNegativeLimit(&sim) ? "yes" : "no");
    
    return 0;
}
