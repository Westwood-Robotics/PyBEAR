from unittest import mock

from pybear.Packet import PKT


class SilentSerial:
    """A serial port that never answers."""
    in_waiting = 0

    def __init__(self):
        self.writes = 0

    def reset_input_buffer(self):
        pass

    def write(self, data):
        self.writes += 1

    def read(self, n):
        return b''


def make_pkt(**kwargs):
    with mock.patch.object(PKT, 'open_port'):
        pkt = PKT('/dev/null', 8000000, **kwargs)
    pkt.ser = SilentSerial()
    return pkt


def test_bulk_read_uses_bulk_try_num():
    pkt = make_pkt(single_try_num=2, bulk_timeout=0.0, bulk_try_num=5)
    data, err = pkt.bulk_read_packet([0xFF, 0xFF, 0xFE, 0, 0x08, 0], bear_num=2)
    assert (data, err) == (None, None)
    assert pkt.ser.writes == 5  # one send plus four resends
