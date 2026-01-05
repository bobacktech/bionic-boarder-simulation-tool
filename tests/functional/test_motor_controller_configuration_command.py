from . import motor_controller_configuration_msg_requester
import pytest
import asyncio


@pytest.mark.skip(reason="Skip is the default behavior. Remove this line to enable the test.")
@pytest.mark.asyncio
async def test_motor_controller_configuration_command_response(activate_sim_and_ble_client):
    client = activate_sim_and_ble_client
    mcc_requester = motor_controller_configuration_msg_requester.MotorControllerConfigurationMsgRequester(client)
    await mcc_requester.set_up_response_handler()
    await mcc_requester.send_command()
    while mcc_requester.response_received == False:
        await asyncio.sleep(0.1)
    assert len(mcc_requester.response_buffer) == 1
    motor_controller_config_data = mcc_requester.response_buffer[0][1]
    assert motor_controller_config_data[0] == 3  # Start byte
    assert (
        int.from_bytes(motor_controller_config_data[1:3], byteorder="big") == 696
    )  # Length byte of Motor Controller Configuration Message Data
    assert motor_controller_config_data[3] == 14  # Message ID for motor controller configuration response
