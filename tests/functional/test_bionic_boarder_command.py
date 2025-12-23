from . import bionic_boarder_msg_requester
import pytest
import time
import asyncio

NUMBER_VESC_BIONIC_BOARDER_MSG_REQUESTS = 10


@pytest.mark.skip(reason="This test is currently disabled.")
def test_bionic_boarder_command_response(activate_sim_and_bluetooth_socket):
    app, socket = activate_sim_and_bluetooth_socket
    bbmr = bionic_boarder_msg_requester.VescBionicBoarderMsgRequester(socket)
    bbmr.send_bionic_boarder_msg_request()
    count = 0
    time.sleep(4)
    while count < NUMBER_VESC_BIONIC_BOARDER_MSG_REQUESTS:
        while bbmr.bionic_boarder_msg_received == False:
            app.processEvents()
        bbmr.send_bionic_boarder_msg_request()
        count += 1
    assert len(bbmr.bionic_boarder_msg_buffer) == 40


@pytest.mark.asyncio
async def test_bionic_boarder_command_response_ble(activate_sim_and_ble_client):
    client = activate_sim_and_ble_client
    bb_requester = bionic_boarder_msg_requester.VescBionicBoarderMsgRequesterBLE(client)
    await bb_requester.set_up_response_handler()
    await bb_requester.send_command()
    i = 0
    while i < NUMBER_VESC_BIONIC_BOARDER_MSG_REQUESTS:
        while bb_requester.response_received == False:
            await asyncio.sleep(0.1)
        await bb_requester.send_command()
        i += 1
    assert len(bb_requester.response_buffer) == 10
