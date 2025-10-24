from . import bionic_boarder_msg_requester
import pytest
import time

NUMBER_VESC_BIONIC_BOARDER_MSG_REQUESTS = 40


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
