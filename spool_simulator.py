"""
Модуль симуляции воздействия PWM сигнала на двух пропорциональных клапанах
на положение золотника гидравлического распределителя.

Система:
- Золотник в центральном положении, подпружинен с обеих сторон (нагрузка до 12кг)
- Два пропорциональных клапана PPRV-04-S-25-D24 (24В, 25 бар)
- Трубки: внутренний диаметр 2мм, длина 100мм
- Площадь давления на золотник: 1 см²
"""

import math
from dataclasses import dataclass
from typing import Tuple


@dataclass
class SystemParameters:
    """Параметры физической системы"""
    # Параметры золотника
    spool_mass: float = 0.5  # кг (примерная масса золотника)
    spring_force_max: float = 117.6  # Н (12 кг * 9.8 м/с²)
    spring_stiffness: float = 5000.0  # Н/м (жесткость пружины)
    damping_coefficient: float = 10.0  # Н*с/м (коэффициент демпфирования)
    spool_area: float = 0.0001  # м² (1 см²)
    max_spool_travel: float = 0.005  # м (5 мм максимальное перемещение)
    
    # Параметры трубок
    tube_inner_diameter: float = 0.002  # м (2 мм)
    tube_length: float = 0.1  # м (100 мм)
    tube_volume: float = None  # вычисляется
    
    # Параметры клапанов
    supply_pressure: float = 25e5  # Па (25 бар)
    valve_nominal_voltage: float = 24.0  # В
    valve_response_time: float = 0.02  # с (время реакции клапана)
    valve_flow_coefficient: float = 1e-8  # м³/(с*Па) (коэффициент расхода)
    
    # Параметры жидкости
    fluid_bulk_modulus: float = 1.4e9  # Па (модуль объемной упругости масла)
    fluid_density: float = 850.0  # кг/м³ (плотность масла)
    
    def __post_init__(self):
        if self.tube_volume is None:
            radius = self.tube_inner_diameter / 2
            self.tube_volume = math.pi * radius**2 * self.tube_length


class PWMValve:
    """
    Модель пропорционального клапана с PWM управлением.
    PPRV-04-S-25-D24: 24В, 25 бар, пропорциональный редукционный клапан.
    """
    
    def __init__(self, params: SystemParameters, is_left: bool = True):
        self.params = params
        self.is_left = is_left  # Левый или правый клапан
        
        # Состояние клапана
        self.pwm_duty_cycle: float = 0.0  # 0.0 - 1.0
        self.effective_opening: float = 0.0  # 0.0 - 1.0 (с учетом динамики)
        self.output_pressure: float = 0.0  # Па
        
        # Фильтр первого порядка для моделирования динамики клапана
        self.time_constant: float = self.params.valve_response_time / 3.0
    
    def update_pwm(self, duty_cycle: float, dt: float):
        """
        Обновление PWM сигнала с учетом динамики клапана.
        
        Args:
            duty_cycle: Коэффициент заполнения PWM (0.0 - 1.0)
            dt: Шаг времени, с
        """
        self.pwm_duty_cycle = max(0.0, min(1.0, duty_cycle))
        
        # Моделирование динамики открытия клапана (фильтр первого порядка)
        target_opening = self.pwm_duty_cycle
        self.effective_opening += (target_opening - self.effective_opening) * (dt / self.time_constant)
        self.effective_opening = max(0.0, min(1.0, self.effective_opening))
    
    def calculate_flow(self, upstream_pressure: float, downstream_pressure: float) -> float:
        """
        Расчет расхода через клапан.
        
        Args:
            upstream_pressure: Давление на входе, Па
            downstream_pressure: Давление на выходе, Па
            
        Returns:
            Расход, м³/с (положительный - от входа к выходу)
        """
        pressure_diff = upstream_pressure - downstream_pressure
        
        if pressure_diff <= 0:
            return 0.0
        
        # Упрощенная модель расхода через щель
        flow = (self.effective_opening * 
                self.params.valve_flow_coefficient * 
                math.sqrt(pressure_diff))
        
        return flow
    
    def set_output_pressure(self, pressure: float):
        """Установка выходного давления (для обратной связи)"""
        self.output_pressure = max(0.0, pressure)


class TubeDynamics:
    """
    Модель гидравлической трубки с учетом сжимаемости жидкости.
    """
    
    def __init__(self, params: SystemParameters):
        self.params = params
        self.pressure: float = 0.0  # Давление в трубке, Па
    
    def update_pressure(self, inflow: float, outflow: float, dt: float):
        """
        Обновление давления в трубке с учетом сжимаемости жидкости.
        
        Args:
            inflow: Входящий расход от клапана, м³/с
            outflow: Исходящий расход к золотнику, м³/с
            dt: Шаг времени, с
        """
        net_flow = inflow - outflow
        
        # Изменение давления из-за сжимаемости жидкости
        dp = (self.params.fluid_bulk_modulus / self.params.tube_volume) * net_flow * dt
        
        self.pressure += dp
        self.pressure = max(0.0, self.pressure)  # Давление не может быть отрицательным


class SpoolDynamics:
    """
    Модель динамики золотника под действием сил давления и пружин.
    """
    
    def __init__(self, params: SystemParameters):
        self.params = params
        
        # Состояние золотника
        self.position: float = 0.0  # м (0 - центральное положение)
        self.velocity: float = 0.0  # м/с
        self.acceleration: float = 0.0  # м/с²
    
    def calculate_forces(self, left_pressure: float, right_pressure: float) -> Tuple[float, float, float]:
        """
        Расчет сил, действующих на золотник.
        
        Args:
            left_pressure: Давление слева, Па
            right_pressure: Давление справа, Па
            
        Returns:
            Кортеж (сила_давления, сила_пружин, сила_демпфирования)
        """
        # Сила от давления жидкости
        pressure_force = (left_pressure - right_pressure) * self.params.spool_area
        
        # Сила от пружин (линейная модель)
        # При смещении вправо левая пружина сжимается, правая растягивается
        spring_force = -self.params.spring_stiffness * self.position
        
        # Сила демпфирования
        damping_force = -self.params.damping_coefficient * self.velocity
        
        return pressure_force, spring_force, damping_force
    
    def update(self, left_pressure: float, right_pressure: float, dt: float):
        """
        Обновление положения и скорости золотника.
        
        Args:
            left_pressure: Давление слева, Па
            right_pressure: Давление справа, Па
            dt: Шаг времени, с
        """
        pressure_force, spring_force, damping_force = self.calculate_forces(
            left_pressure, right_pressure
        )
        
        total_force = pressure_force + spring_force + damping_force
        
        # Второй закон Ньютона
        self.acceleration = total_force / self.params.spool_mass
        
        # Интегрирование (метод Эйлера)
        self.velocity += self.acceleration * dt
        self.position += self.velocity * dt
        
        # Ограничение перемещения
        self.position = max(-self.params.max_spool_travel, 
                           min(self.params.max_spool_travel, self.position))
        
        # Если золотник достиг предела, обнуляем скорость
        if abs(self.position) >= self.params.max_spool_travel:
            self.velocity = 0.0
            self.acceleration = 0.0


class HydraulicSystemSimulator:
    """
    Основной класс симулятора гидравлической системы.
    Объединяет клапаны, трубки и золотник.
    """
    
    def __init__(self, params: SystemParameters = None):
        self.params = params or SystemParameters()
        
        # Компоненты системы
        self.left_valve = PWMValve(self.params, is_left=True)
        self.right_valve = PWMValve(self.params, is_left=False)
        
        self.left_tube = TubeDynamics(self.params)
        self.right_tube = TubeDynamics(self.params)
        
        self.spool = SpoolDynamics(self.params)
        
        # Время симуляции
        self.simulation_time: float = 0.0
        
        # История данных
        self.history = {
            'time': [],
            'left_pwm': [],
            'right_pwm': [],
            'left_pressure': [],
            'right_pressure': [],
            'spool_position': [],
            'spool_velocity': [],
        }
    
    def set_pwm_signals(self, left_duty: float, right_duty: float):
        """
        Установка PWM сигналов на оба клапана.
        
        Args:
            left_duty: Коэффициент заполнения левого клапана (0.0 - 1.0)
            right_duty: Коэффициент заполнения правого клапана (0.0 - 1.0)
        """
        self.left_valve.pwm_duty_cycle = max(0.0, min(1.0, left_duty))
        self.right_valve.pwm_duty_cycle = max(0.0, min(1.0, right_duty))
    
    def step(self, dt: float, left_duty: float = None, right_duty: float = None):
        """
        Выполнение одного шага симуляции.
        
        Args:
            dt: Шаг времени, с
            left_duty: Новый PWM левого клапана (опционально)
            right_duty: Новый PWM правого клапана (опционально)
        """
        # Обновление PWM сигналов если указаны
        if left_duty is not None:
            self.left_valve.update_pwm(left_duty, dt)
        else:
            self.left_valve.update_pwm(self.left_valve.pwm_duty_cycle, dt)
            
        if right_duty is not None:
            self.right_valve.update_pwm(right_duty, dt)
        else:
            self.right_valve.update_pwm(self.right_valve.pwm_duty_cycle, dt)
        
        # Расчет расходов через клапаны
        left_flow_in = self.left_valve.calculate_flow(
            self.params.supply_pressure, 
            self.left_tube.pressure
        )
        
        right_flow_in = self.right_valve.calculate_flow(
            self.params.supply_pressure,
            self.right_tube.pressure
        )
        
        # Расчет потока к золотнику (упрощенно - пропорционально перепаду давления)
        # Поток из левой трубки к золотнику
        left_flow_to_spool = 0.0
        if self.left_tube.pressure > 0:
            # Упрощенная модель: поток зависит от перепада давления и площади
            left_flow_to_spool = (self.left_tube.pressure * self.params.spool_area * 
                                  self.spool.velocity * dt if self.spool.velocity > 0 else 0)
        
        # Поток из правой трубки к золотнику
        right_flow_to_spool = 0.0
        if self.right_tube.pressure > 0:
            right_flow_to_spool = (self.right_tube.pressure * self.params.spool_area * 
                                   self.spool.velocity * dt if self.spool.velocity < 0 else 0)
        
        # Обновление давлений в трубках
        self.left_tube.update_pressure(left_flow_in, left_flow_to_spool, dt)
        self.right_tube.update_pressure(right_flow_in, right_flow_to_spool, dt)
        
        # Обновление положения золотника
        self.spool.update(self.left_tube.pressure, self.right_tube.pressure, dt)
        
        # Обновление времени
        self.simulation_time += dt
        
        # Сохранение истории
        self._save_history()
    
    def _save_history(self):
        """Сохранение текущего состояния в историю"""
        self.history['time'].append(self.simulation_time)
        self.history['left_pwm'].append(self.left_valve.pwm_duty_cycle)
        self.history['right_pwm'].append(self.right_valve.pwm_duty_cycle)
        self.history['left_pressure'].append(self.left_tube.pressure / 1e5)  # в барах
        self.history['right_pressure'].append(self.right_tube.pressure / 1e5)  # в барах
        self.history['spool_position'].append(self.spool.position * 1000)  # в мм
        self.history['spool_velocity'].append(self.spool.velocity * 1000)  # в мм/с
    
    def get_state(self) -> dict:
        """
        Получение текущего состояния системы.
        
        Returns:
            Словарь с текущими параметрами
        """
        return {
            'time': self.simulation_time,
            'left_pwm': self.left_valve.pwm_duty_cycle,
            'right_pwm': self.right_valve.pwm_duty_cycle,
            'left_pressure_bar': self.left_tube.pressure / 1e5,
            'right_pressure_bar': self.right_tube.pressure / 1e5,
            'spool_position_mm': self.spool.position * 1000,
            'spool_velocity_mm_s': self.spool.velocity * 1000,
        }
    
    def reset(self):
        """Сброс системы в начальное состояние"""
        self.left_valve = PWMValve(self.params, is_left=True)
        self.right_valve = PWMValve(self.params, is_left=False)
        self.left_tube = TubeDynamics(self.params)
        self.right_tube = TubeDynamics(self.params)
        self.spool = SpoolDynamics(self.params)
        self.simulation_time = 0.0
        self.history = {
            'time': [],
            'left_pwm': [],
            'right_pwm': [],
            'left_pressure': [],
            'right_pressure': [],
            'spool_position': [],
            'spool_velocity': [],
        }


def run_simulation(
    duration: float = 1.0,
    dt: float = 0.001,
    left_pwm_func=None,
    right_pwm_func=None
) -> HydraulicSystemSimulator:
    """
    Запуск симуляции на заданное время.
    
    Args:
        duration: Длительность симуляции, с
        dt: Шаг времени, с
        left_pwm_func: Функция времени для левого PWM (t) -> duty
        right_pwm_func: Функция времени для правого PWM (t) -> duty
        
    Returns:
        Симулятор с заполненной историей
    """
    simulator = HydraulicSystemSimulator()
    
    t = 0.0
    while t < duration:
        left_duty = left_pwm_func(t) if left_pwm_func else 0.0
        right_duty = right_pwm_func(t) if right_pwm_func else 0.0
        
        simulator.step(dt, left_duty, right_duty)
        t += dt
    
    return simulator
