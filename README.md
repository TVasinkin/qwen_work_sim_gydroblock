# Модуль симуляции гидравлического золотника для STM32F303

## Описание

Модуль симулирует воздействие ШИМ сигнала на два пропорциональных клапана PPRV-04-S-25-D24, 
которые управляют положением гидравлического золотника.

## Характеристики системы

- **Золотник**: подпружинен с обеих сторон (нагрузка до 12кг)
- **Клапаны**: PPRV-04-S-25-D24 (24В, 25 бар)
- **Трубки**: Ø2мм, длина 100мм
- **Площадь давления**: 1 см²
- **Ход золотника**: 8мм

## Интерфейс

### Входные данные
- **ШИМ сигнал**: 0-7000 отсчетов
  - 0 = 0%
  - 7000 = 100%
  - 1400 (20%) = начало страгивания
  - 5600 (80%) = полное открытие

### Выходные данные
- **Положение золотника**: 0-12000 отсчетов
  - 0 = левый предел (0мм)
  - 6000 = центральное положение (4мм)
  - 12000 = правый предел (8мм)

## Файлы модуля

| Файл | Описание |
|------|----------|
| `spool_simulator.h` | Заголовочный файл с объявлениями функций и типов |
| `spool_simulator.c` | Реализация симулятора |
| `test_spool_simulator.c` | Модульные тесты |

## Быстрый старт

### 1. Инициализация

```c
#include "spool_simulator.h"

SpoolSimulator_t simulator;
SpoolConfig_t config = {
    .dt_ms = 1.0f,                    // Шаг симуляции 1мс
    .valve_response_time_ms = 15.0f,  // Время реакции клапана
    .hydraulic_delay_ms = 5.0f,       // Гидравлическая задержка
    .damping_coefficient = 0.7f,      // Коэффициент демпфирования
    .mass_kg = 0.5f                   // Масса золотника
};

// Инициализация
if (!SpoolSimulator_Init(&simulator, &config)) {
    // Обработка ошибки
}
```

### 2. Обновление в цикле

```c
// Вызывать периодически (например, каждые 1мс)
uint16_t pwm_value = get_pwm_from_adc(); // 0-7000
SpoolSimulator_Update(&simulator, pwm_value, 1.0f);

// Получение положения
uint16_t position = SpoolSimulator_GetPosition(&simulator);
float position_mm = SpoolSimulator_GetPositionMm(&simulator);
```

### 3. Пример использования в прерывании таймера

```c
void TIMx_IRQHandler(void) {
    static uint16_t last_pwm = 0;
    uint16_t current_pwm = ADC_GetValue(); // Чтение АЦП
    
    // Обновление симуляции
    SpoolSimulator_Update(&simulator, current_pwm, 1.0f);
    
    // Использование результата
    uint16_t spool_pos = SpoolSimulator_GetPosition(&simulator);
    
    // Отправка в CAN/UART или использование в системе управления
    send_position_to_controller(spool_pos);
    
    TIM_ClearITPendingBit(TIMx, TIM_IT_Update);
}
```

## API функции

### Инициализация
- `bool SpoolSimulator_Init(SpoolSimulator_t* sim, const SpoolConfig_t* config)` - Инициализация
- `void SpoolSimulator_Reset(SpoolSimulator_t* sim)` - Сброс в начальное состояние

### Обновление и чтение
- `void SpoolSimulator_Update(SpoolSimulator_t* sim, uint16_t pwm_value, float dt_ms)` - Обновление состояния
- `uint16_t SpoolSimulator_GetPosition(const SpoolSimulator_t* sim)` - Получить позицию (0-12000)
- `float SpoolSimulator_GetPositionMm(const SpoolSimulator_t* sim)` - Получить позицию в мм
- `float SpoolSimulator_GetVelocity(const SpoolSimulator_t* sim)` - Получить скорость

### Вспомогательные
- `float SpoolSimulator_PWMToValveOpen(uint16_t pwm_value)` - Конвертация PWM в открытие клапана
- `float SpoolSimulator_PositionToMm(uint16_t position)` - Позиция в мм
- `uint16_t SpoolSimulator_MmToPosition(float mm)` - Мм в позицию
- `bool SpoolSimulator_IsInitialized(const SpoolSimulator_t* sim)` - Проверка инициализации
- `bool SpoolSimulator_IsMoving(const SpoolSimulator_t* sim)` - Проверка движения

## Логика работы

### Диапазоны ШИМ

| PWM (%) | PWM (отсчеты) | Действие |
|---------|---------------|----------|
| 0-20% | 0-1400 | Оба клапана закрыты, золотник в центре |
| 20-50% | 1400-3500 | Правый клапан открывается, движение влево |
| 50-80% | 3500-5600 | Левый клапан открывается, движение вправо |
| 80-100% | 5600-7000 | Полное открытие активного клапана |

### Физическая модель

Модель учитывает:
1. **Инерцию клапанов** - экспоненциальное открытие/закрытие (15мс)
2. **Сжимаемость масла** - модуль упругости 1400 МПа
3. **Гидравлическую задержку** - 5мс на распространение давления
4. **Силы пружин** - возврат в центральное положение
5. **Демпфирование** - вязкое трение в гидросистеме

## Компиляция и тестирование

### На хост-машине (Linux/Mac)

```bash
# Компиляция тестов
gcc -o test_spool test_spool_simulator.c spool_simulator.c -lm

# Запуск тестов
./test_spool
```

### Для STM32F303

Добавьте файлы в ваш проект:
```
Core/Src/spool_simulator.c
Core/Inc/spool_simulator.h
```

Убедитесь, что подключена библиотека math (`-lm` в линковщике).

## Требования к ресурсам STM32F303

- **Flash**: ~4-6 КБ
- **RAM**: ~200 байт на экземпляр симулятора
- **CPU**: ~10-20 мкс на вызов Update (при 72 МГц)

## Примечания

1. Модель является упрощенной и может требовать калибровки под конкретную систему
2. Параметры в `SpoolConfig_t` можно настраивать для соответствия реальной системе
3. Рекомендуется вызывать Update с постоянным шагом (1-5 мс)

## Лицензия

MIT License
