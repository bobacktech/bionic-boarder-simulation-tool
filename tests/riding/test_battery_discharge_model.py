import pytest
from augmented_skateboarding_simulator.riding.battery_discharge_model import (
    BatteryDischargeModel,
)


class TestBatteryDischargeModel:
    @pytest.fixture
    def model(self):
        return BatteryDischargeModel(battery_max_nominal_voltage=37)

    def test_initialization(self, model: BatteryDischargeModel):
        assert (
            model.get_watt_hours_consumed() == 0
        ), "Initial watt-hours consumed should be 0"

    def test_current_draw_property(self, model: BatteryDischargeModel):
        assert model.current_draw == 1.0, "Initial current draw should be 1.0"
        model.current_draw = 2.0
        assert model.current_draw == 2.0, "Current draw should be updated to 2.0"

    def test_discharge_increases_energy_consumed(self, model: BatteryDischargeModel):
        initial_energy = model.get_watt_hours_consumed()
        model.discharge(time_duration_ms=1000)  # Discharge for 1 second
        assert (
            model.get_watt_hours_consumed() > initial_energy
        ), "Energy consumed should increase after discharge"

    def test_energy_consumed_calculation(self, model: BatteryDischargeModel):
        model.current_draw = 2  # Set current draw to 2A
        model.discharge(time_duration_ms=3600000)  # Discharge for 1 hour
        expected_energy_consumed = 2 * 37  # P=IV, so E = Pt where t is 1 hour
        assert (
            model.get_watt_hours_consumed() == expected_energy_consumed
        ), "Energy consumed calculation is incorrect"

    def test_thread_safety(self, model: BatteryDischargeModel):
        from threading import Thread

        initial_energy = model.get_watt_hours_consumed()

        def discharge_model():
            model.discharge(time_duration_ms=500)  # Discharge for 0.5 second

        threads = [Thread(target=discharge_model) for _ in range(10)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

        # Check if the energy consumed is correctly updated after concurrent discharges
        assert (
            model.get_watt_hours_consumed() > initial_energy
        ), "Concurrent discharge should increase energy consumed correctly"
