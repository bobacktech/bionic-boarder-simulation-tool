from . import vesc_state_msg_requester
import pytest
import asyncio


NUMBER_VESC_STATE_MSG_REQUESTS = 10


@pytest.mark.asyncio
async def test_bionic_boarder_command_response_ble(activate_sim_and_ble_client):
    client = activate_sim_and_ble_client
    vs_requester = vesc_state_msg_requester.VescStateMsgRequesterBLE(client)
    await vs_requester.set_up_response_handler()
    await vs_requester.send_command()
    i = 0
    while i < NUMBER_VESC_STATE_MSG_REQUESTS:
        while vs_requester.response_received == False:
            await asyncio.sleep(0.1)
        await vs_requester.send_command()
        i += 1
    assert len(vs_requester.response_buffer) == NUMBER_VESC_STATE_MSG_REQUESTS
