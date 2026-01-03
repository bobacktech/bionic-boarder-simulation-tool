from . import bionic_boarder_msg_requester
import pytest
import asyncio

NUMBER_VESC_BIONIC_BOARDER_MSG_REQUESTS = 10


@pytest.mark.skip(reason="Skip is the default behavior. Remove this line to enable the test.")
@pytest.mark.asyncio
async def test_bionic_boarder_command_response(activate_sim_and_ble_client):
    client = activate_sim_and_ble_client
    bb_requester = bionic_boarder_msg_requester.VescBionicBoarderMsgRequester(client)
    await bb_requester.set_up_response_handler()
    await bb_requester.send_command()
    i = 0
    while i < NUMBER_VESC_BIONIC_BOARDER_MSG_REQUESTS:
        while bb_requester.response_received == False:
            await asyncio.sleep(0.1)
        await bb_requester.send_command()
        i += 1
    assert len(bb_requester.response_buffer) == NUMBER_VESC_BIONIC_BOARDER_MSG_REQUESTS
