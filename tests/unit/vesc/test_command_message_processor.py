import pytest
from bionic_boarder_simulation_tool.vesc.command_message_processor import (
    CommandMessageProcessor,
)


class TestCommandMessageProcessor(CommandMessageProcessor):
    def __init__(self, com_port, baud_rate, command_byte_size):
        super().__init__(com_port, baud_rate, command_byte_size)
        self.__cmd_id_name = {
            2: CommandMessageProcessor.CURRENT,
            3: CommandMessageProcessor.RPM,
            4: CommandMessageProcessor.HEARTBEAT,
            5: CommandMessageProcessor.FIRMWARE,
            6: CommandMessageProcessor.STATE,
            7: CommandMessageProcessor.BIONIC_BOARDER,
            8: CommandMessageProcessor.MOTOR_CONTROLLER_CONFIGURATION,
        }

    @property
    def _command_id_name(self):
        """
        Returns a dictionary of the command id associated to the name of the command.
        """
        return self.__cmd_id_name

    def _get_command_id(self, command: bytes) -> int:
        return 1  # Mocked to always return a specific command ID

    def _publish_state(self):
        pass

    def _publish_bionic_boarder(self):
        pass

    def _publish_firmware(self):
        pass

    def _publish_motor_controller_configuration(self):
        pass

    def _update_current(self, command):
        pass

    def _update_rpm(self, command):
        pass


@pytest.fixture
def mock_serial(mocker):
    return mocker.patch("serial.Serial", autospec=True)


@pytest.fixture
def processor(mock_serial):
    mock_serial.return_value.read.side_effect = [b"some_bytes", StopIteration()]
    return TestCommandMessageProcessor("COM1", 230400, 8)


def test_set_heartbeat_timeout_sec(processor):
    processor.set_heartbeat_timeout_sec(2.0)
    assert processor._CommandMessageProcessor__heartbeat_timeout_sec == 2.0


def test_handle_command_current(processor, mocker):
    mocker.patch.object(processor, "_update_current", autospec=True)
    mocker.patch.object(processor, "_get_command_id", return_value=2)
    with pytest.raises(StopIteration):
        processor.handle_command()
    processor._update_current.assert_called_once()


def test_handle_command_rpm(processor, mocker):
    mocker.patch.object(processor, "_update_rpm", autospec=True)
    mocker.patch.object(processor, "_get_command_id", return_value=3)
    with pytest.raises(StopIteration):
        processor.handle_command()
    processor._update_rpm.assert_called_once()


def test_handle_command_heartbeat(processor, mocker):
    mocker.patch.object(processor, "heartbeat", autospec=True)
    mocker.patch.object(processor, "_get_command_id", return_value=4)
    with pytest.raises(StopIteration):
        processor.handle_command()
    processor.heartbeat.assert_called_once()


def test_handle_command_firmware(processor, mocker):
    mocker.patch.object(processor, "_publish_firmware", autospec=True)
    mocker.patch.object(processor, "_get_command_id", return_value=5)
    with pytest.raises(StopIteration):
        processor.handle_command()
    processor._publish_firmware.assert_called_once()


def test_handle_command_motor_controller_configuration(processor, mocker):
    mocker.patch.object(processor, "_publish_motor_controller_configuration", autospec=True)
    mocker.patch.object(processor, "_get_command_id", return_value=8)
    with pytest.raises(StopIteration):
        processor.handle_command()
    processor._publish_motor_controller_configuration.assert_called_once()


def test_handle_command_bionic_boarder(processor, mocker):
    mocker.patch.object(processor, "_publish_bionic_boarder", autospec=True)
    mocker.patch.object(processor, "_get_command_id", return_value=7)
    with pytest.raises(StopIteration):
        processor.handle_command()
    processor._publish_bionic_boarder.assert_called_once()


def test_handle_command_state(processor, mocker):
    mocker.patch.object(processor, "_publish_state", autospec=True)
    mocker.patch.object(processor, "_get_command_id", return_value=6)
    with pytest.raises(StopIteration):
        processor.handle_command()
    processor._publish_state.assert_called_once()


def test_command_id_names(processor):
    assert processor._command_id_name[2] == CommandMessageProcessor.CURRENT
    assert processor._command_id_name[3] == CommandMessageProcessor.RPM
    assert processor._command_id_name[4] == CommandMessageProcessor.HEARTBEAT
    assert processor._command_id_name[5] == CommandMessageProcessor.FIRMWARE
    assert processor._command_id_name[6] == CommandMessageProcessor.STATE
    assert processor._command_id_name[8] == CommandMessageProcessor.MOTOR_CONTROLLER_CONFIGURATION
