"""
Модульные тесты для симулятора гидравлической системы.
Тестирование воздействия PWM сигнала на двух пропорциональных клапанах
на положение золотника.
"""

import pytest
import math
from spool_simulator import (
    SystemParameters,
    PWMValve,
    TubeDynamics,
    SpoolDynamics,
    HydraulicSystemSimulator,
    run_simulation,
)


class TestSystemParameters:
    """Тесты для класса параметров системы"""
    
    def test_default_parameters(self):
        """Проверка значений параметров по умолчанию"""
        params = SystemParameters()
        
        assert params.spool_mass == 0.5
        assert params.spring_force_max == 117.6  # 12 кг * 9.8
        assert params.spool_area == 0.0001  # 1 см² в м²
        assert params.tube_inner_diameter == 0.002  # 2 мм
        assert params.tube_length == 0.1  # 100 мм
        assert params.supply_pressure == 25e5  # 25 бар
        
    def test_tube_volume_calculation(self):
        """Проверка расчета объема трубки"""
        params = SystemParameters()
        
        # Объем цилиндра: π * r² * h
        expected_volume = math.pi * (0.001 ** 2) * 0.1  # радиус 1мм, длина 100мм
        
        assert params.tube_volume == pytest.approx(expected_volume, rel=1e-10)
    
    def test_custom_parameters(self):
        """Проверка установки пользовательских параметров"""
        params = SystemParameters(
            spool_mass=1.0,
            supply_pressure=20e5,
        )
        
        assert params.spool_mass == 1.0
        assert params.supply_pressure == 20e5


class TestPWMValve:
    """Тесты для модели пропорционального клапана"""
    
    @pytest.fixture
    def valve(self):
        """Создание тестового клапана"""
        params = SystemParameters()
        return PWMValve(params, is_left=True)
    
    def test_initial_state(self, valve):
        """Проверка начального состояния клапана"""
        assert valve.pwm_duty_cycle == 0.0
        assert valve.effective_opening == 0.0
        assert valve.output_pressure == 0.0
    
    def test_pwm_update_instantaneous(self, valve):
        """Проверка обновления PWM сигнала"""
        dt = 0.01
        valve.update_pwm(0.5, dt)
        
        assert valve.pwm_duty_cycle == 0.5
        # Effective opening должен измениться с учетом динамики
        assert valve.effective_opening > 0.0
        # Effective opening может превышать target из-за быстрой динамики
        assert valve.effective_opening <= 1.0
    
    def test_pwm_clamping(self, valve):
        """Проверка ограничения PWM сигнала диапазоном [0, 1]"""
        dt = 0.01
        
        valve.update_pwm(1.5, dt)  # Больше 1
        assert valve.pwm_duty_cycle == 1.0
        
        valve.update_pwm(-0.5, dt)  # Меньше 0
        assert valve.pwm_duty_cycle == 0.0
    
    def test_valve_dynamics(self, valve):
        """Проверка динамики открытия клапана"""
        dt = 0.001
        
        # Устанавливаем 100% PWM
        valve.update_pwm(1.0, dt)
        opening_1 = valve.effective_opening
        
        # Еще один шаг с тем же PWM
        valve.update_pwm(1.0, dt)
        opening_2 = valve.effective_opening
        
        # Открытие должно расти или оставаться на максимуме
        assert opening_2 >= opening_1
        assert opening_2 > 0.0
    
    def test_flow_calculation_zero_opening(self, valve):
        """Проверка расхода при закрытом клапане"""
        params = SystemParameters()
        valve = PWMValve(params)
        
        flow = valve.calculate_flow(25e5, 0)
        assert flow == 0.0
    
    def test_flow_calculation_positive_pressure_diff(self, valve):
        """Проверка расхода при положительном перепаде давления"""
        # Сначала откроем клапан
        for _ in range(100):
            valve.update_pwm(1.0, 0.001)
        
        flow = valve.calculate_flow(25e5, 0)
        assert flow > 0.0
    
    def test_flow_calculation_negative_pressure_diff(self, valve):
        """Проверка отсутствия обратного потока"""
        for _ in range(100):
            valve.update_pwm(1.0, 0.001)
        
        # Давление на выходе больше чем на входе
        flow = valve.calculate_flow(1e5, 25e5)
        assert flow == 0.0


class TestTubeDynamics:
    """Тесты для модели гидравлической трубки"""
    
    @pytest.fixture
    def tube(self):
        """Создание тестовой трубки"""
        params = SystemParameters()
        return TubeDynamics(params)
    
    def test_initial_pressure(self, tube):
        """Проверка начального давления"""
        assert tube.pressure == 0.0
    
    def test_pressure_increase_with_inflow(self, tube):
        """Проверка роста давления при входящем потоке"""
        dt = 0.001
        inflow = 1e-6  # м³/с
        outflow = 0.0
        
        initial_pressure = tube.pressure
        tube.update_pressure(inflow, outflow, dt)
        
        assert tube.pressure > initial_pressure
    
    def test_pressure_decrease_with_outflow(self, tube):
        """Проверка снижения давления при исходящем потоке"""
        # Сначала создадим давление
        tube.pressure = 10e5
        
        dt = 0.001
        inflow = 0.0
        outflow = 1e-6
        
        initial_pressure = tube.pressure
        tube.update_pressure(inflow, outflow, dt)
        
        assert tube.pressure < initial_pressure
    
    def test_pressure_non_negative(self, tube):
        """Проверка что давление не становится отрицательным"""
        tube.pressure = 1e5
        
        dt = 0.01
        inflow = 0.0
        outflow = 1e-4  # Большой исходящий поток
        
        tube.update_pressure(inflow, outflow, dt)
        
        assert tube.pressure >= 0.0


class TestSpoolDynamics:
    """Тесты для модели золотника"""
    
    @pytest.fixture
    def spool(self):
        """Создание тестового золотника"""
        params = SystemParameters()
        return SpoolDynamics(params)
    
    def test_initial_position(self, spool):
        """Проверка начального положения"""
        assert spool.position == 0.0
        assert spool.velocity == 0.0
        assert spool.acceleration == 0.0
    
    def test_movement_with_pressure_difference(self, spool):
        """Проверка перемещения при разности давлений"""
        dt = 0.001
        left_pressure = 10e5  # 10 бар
        right_pressure = 0
        
        spool.update(left_pressure, right_pressure, dt)
        
        # Золотник должен двигаться вправо (положительное направление)
        assert spool.position > 0.0
        assert spool.velocity > 0.0
    
    def test_movement_opposite_direction(self, spool):
        """Проверка перемещения в противоположном направлении"""
        dt = 0.001
        left_pressure = 0
        right_pressure = 10e5  # 10 бар
        
        spool.update(left_pressure, right_pressure, dt)
        
        # Золотник должен двигаться влево (отрицательное направление)
        assert spool.position < 0.0
        assert spool.velocity < 0.0
    
    def test_spring_restoring_force(self, spool):
        """Проверка возвращающей силы пружины"""
        # Смещаем золотник вручную
        spool.position = 0.001  # 1 мм
        
        dt = 0.001
        left_pressure = 0
        right_pressure = 0
        
        # Без внешнего давления пружина должна возвращать золотник
        spool.update(left_pressure, right_pressure, dt)
        
        # Позиция должна уменьшаться (возвращаться к нулю)
        assert abs(spool.position) < 0.001
    
    def test_position_limits(self, spool):
        """Проверка ограничения перемещения"""
        dt = 0.1
        left_pressure = 25e5  # Максимальное давление
        right_pressure = 0
        
        # Много шагов для достижения предела
        for _ in range(1000):
            spool.update(left_pressure, right_pressure, dt)
        
        # Позиция не должна превышать максимальное перемещение
        assert abs(spool.position) <= spool.params.max_spool_travel
    
    def test_velocity_zero_at_limit(self, spool):
        """Проверка обнуления скорости на пределе"""
        dt = 0.1
        left_pressure = 25e5
        right_pressure = 0
        
        for _ in range(1000):
            spool.update(left_pressure, right_pressure, dt)
        
        # На пределе скорость должна быть нулевой
        assert spool.velocity == 0.0


class TestHydraulicSystemSimulator:
    """Тесты для основного симулятора"""
    
    @pytest.fixture
    def simulator(self):
        """Создание тестового симулятора"""
        return HydraulicSystemSimulator()
    
    def test_initial_state(self, simulator):
        """Проверка начального состояния системы"""
        state = simulator.get_state()
        
        assert state['time'] == 0.0
        assert state['left_pwm'] == 0.0
        assert state['right_pwm'] == 0.0
        assert state['spool_position_mm'] == 0.0
    
    def test_single_step_simulation(self, simulator):
        """Проверка одного шага симуляции"""
        dt = 0.001
        simulator.step(dt, left_duty=0.5, right_duty=0.0)
        
        state = simulator.get_state()
        assert state['time'] == pytest.approx(dt, rel=1e-10)
        assert len(simulator.history['time']) == 1
    
    def test_pwm_signal_propagation(self, simulator):
        """Проверка распространения PWM сигнала через систему"""
        dt = 0.001
        
        # Подаем PWM только на левый клапан
        for _ in range(100):
            simulator.step(dt, left_duty=1.0, right_duty=0.0)
        
        state = simulator.get_state()
        
        # Левый клапан должен быть открыт
        assert state['left_pwm'] == 1.0
        # Давление слева должно расти
        assert state['left_pressure_bar'] > 0.0
        # Золотник должен сместиться вправо
        assert state['spool_position_mm'] > 0.0
    
    def test_symmetric_pwm_no_movement(self, simulator):
        """Проверка что симметричный PWM не вызывает перемещения"""
        dt = 0.001
        
        # Одинаковый PWM на оба клапана
        for _ in range(100):
            simulator.step(dt, left_duty=0.5, right_duty=0.5)
        
        state = simulator.get_state()
        
        # При симметричном давлении золотник должен остаться близко к центру
        # (небольшое отклонение возможно из-за численных погрешностей)
        assert abs(state['spool_position_mm']) < 0.1  # менее 0.1 мм
    
    def test_reset_functionality(self, simulator):
        """Проверка сброса симулятора"""
        dt = 0.001
        
        # Проведем симуляцию
        for _ in range(100):
            simulator.step(dt, left_duty=1.0, right_duty=0.0)
        
        # Проверим что система изменилась
        state_before = simulator.get_state()
        assert state_before['time'] > 0.0
        
        # Сбросим
        simulator.reset()
        
        # Проверим возврат к начальному состоянию
        state_after = simulator.get_state()
        assert state_after['time'] == 0.0
        assert state_after['spool_position_mm'] == 0.0
        assert len(simulator.history['time']) == 0
    
    def test_history_recording(self, simulator):
        """Проверка записи истории"""
        dt = 0.001
        num_steps = 50
        
        for i in range(num_steps):
            simulator.step(dt, left_duty=0.5, right_duty=0.3)
        
        # Проверим длину истории
        assert len(simulator.history['time']) == num_steps
        assert len(simulator.history['left_pwm']) == num_steps
        assert len(simulator.history['right_pwm']) == num_steps
        assert len(simulator.history['left_pressure']) == num_steps
        assert len(simulator.history['right_pressure']) == num_steps
        assert len(simulator.history['spool_position']) == num_steps
        assert len(simulator.history['spool_velocity']) == num_steps


class TestRunSimulation:
    """Тесты для функции запуска симуляции"""
    
    def test_run_simulation_default(self):
        """Проверка запуска симуляции по умолчанию"""
        simulator = run_simulation(duration=0.1, dt=0.001)
        
        assert simulator.simulation_time == pytest.approx(0.1, rel=0.01)
        assert len(simulator.history['time']) > 0
    
    def test_run_simulation_with_pwm_functions(self):
        """Проверка запуска с функциями PWM"""
        def left_pwm(t):
            return 1.0 if t < 0.05 else 0.0
        
        def right_pwm(t):
            return 0.0
        
        simulator = run_simulation(
            duration=0.1,
            dt=0.001,
            left_pwm_func=left_pwm,
            right_pwm_func=right_pwm
        )
        
        # Проверим что PWM менялся
        assert any(p > 0.9 for p in simulator.history['left_pwm'])
        assert any(p == 0.0 for p in simulator.history['left_pwm'])
    
    def test_run_simulation_spool_movement(self):
        """Проверка перемещения золотника в симуляции"""
        def left_pwm(t):
            return 1.0
        
        def right_pwm(t):
            return 0.0
        
        simulator = run_simulation(
            duration=0.5,
            dt=0.001,
            left_pwm_func=left_pwm,
            right_pwm_func=right_pwm
        )
        
        # Золотник должен сместиться
        final_position = simulator.history['spool_position'][-1]
        assert final_position > 0.0


class TestIntegrationScenarios:
    """Интеграционные тесты различных сценариев работы"""
    
    def test_step_response_left_valve(self):
        """Реакция на ступенчатое воздействие левого клапана"""
        simulator = HydraulicSystemSimulator()
        dt = 0.001
        
        # Ступенчатое воздействие: 0 -> 100% PWM
        for i in range(200):
            pwm = 1.0 if i >= 10 else 0.0
            simulator.step(dt, left_duty=pwm, right_duty=0.0)
        
        # После включения клапана давление и позиция должны расти
        pressures = simulator.history['left_pressure']
        positions = simulator.history['spool_position']
        
        # Давление после 10 шагов должно расти
        assert pressures[50] > pressures[10]
        # Позиция должна увеличиваться
        assert positions[100] > positions[10]
    
    def test_oscillating_pwm(self):
        """Тест с осциллирующим PWM сигналом"""
        simulator = HydraulicSystemSimulator()
        dt = 0.001
        
        for i in range(200):
            # PWM колеблется между 0 и 1
            pwm = 1.0 if (i // 20) % 2 == 0 else 0.0
            simulator.step(dt, left_duty=pwm, right_duty=pwm)
        
        # При симметричном осциллирующем сигнале золотник должен оставаться около центра
        final_position = simulator.history['spool_position'][-1]
        assert abs(final_position) < 1.0  # менее 1 мм
    
    def test_ramp_response(self):
        """Реакция на линейно нарастающий PWM сигнал"""
        simulator = HydraulicSystemSimulator()
        dt = 0.001
        
        for i in range(100):
            pwm = i / 100.0  # От 0 до 1
            simulator.step(dt, left_duty=pwm, right_duty=0.0)
        
        # Позиция должна в целом расти (допускаем небольшие локальные снижения)
        positions = simulator.history['spool_position']
        
        # Проверяем что конечная позиция больше начальной
        assert positions[-1] > positions[0]
        # Проверяем что была монотонная фаза роста
        growth_count = sum(1 for i in range(1, len(positions)) if positions[i] >= positions[i-1])
        assert growth_count > len(positions) * 0.5  # Более 50% шагов были ростом
    
    def test_opposite_valves_control(self):
        """Управление противоположными клапанами"""
        simulator = HydraulicSystemSimulator()
        dt = 0.001
        
        # Левый клапан открывается, правый закрывается
        for i in range(100):
            left_pwm = i / 100.0
            right_pwm = 1.0 - left_pwm
            simulator.step(dt, left_duty=left_pwm, right_duty=right_pwm)
        
        # Золотник должен значительно сместиться
        final_position = simulator.history['spool_position'][-1]
        assert abs(final_position) > 0.1  # более 0.1 мм


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
