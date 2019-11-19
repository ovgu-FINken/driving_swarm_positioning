import logging
import itertools
import struct


class Package:
    def __init__(self, sender_id=0, msg_id=0, payload=[]):
        self.startbyte = 0x99
        # payload + start, len, msg_id, sender_id + checkSumA, B
        self.length = len(payload) + 6
        self.sender_id = sender_id
        self.msg_id = msg_id
        self.payload = payload
        self.checkSumA, self.checkSumB = self.calculate_checksum()

    @classmethod
    def from_pprz(cls, pprz):
        if len(pprz) < 6:
            logging.debug("Package to small")
        if pprz[0] != 0x99:
            logging.debug("No startbyte found in pprz string")
            return None
        pkg = Package()
        pkg.length = pprz[1]
        pkg.sender_id = pprz[2]
        pkg.msg_id = pprz[3]
        pkg.payload = pprz[4:-2]
        pkg.checkSumA = pprz[-2]
        pkg.checkSumB = pprz[-1]
        return pkg

    def header(self):
        header = bytearray( struct.pack(
                            'BBBB',
                            self.startbyte,
                            self.length,
                            self.sender_id,
                            self.msg_id))
        return header 

    def __bytes__(self):
        x = bytearray(self.payload) + bytearray([self.checkSumA, self.checkSumB])
        return self.header() + x

    def __repr__(self):
        return "header: %s\npaylaod: %s\nA: %x, B: %x, matching: %s\nmsg_id: %s, sender_id: %s" % (
                self.header(),
                self.payload,
                self.checkSumA,
                self.checkSumB,
                self.valid(),
                self.msg_id,
                self.sender_id)

    def calculate_checksum(self):
        checkSumA = checkSumB = 0
        for c in itertools.chain(
                self.header()[1:],
                bytearray(self.payload)
                ):
            checkSumA = (checkSumA + c) % 256
            checkSumB = (checkSumB + checkSumA) % 256
        return checkSumA, checkSumB

    def check_length(self):
        return self.length == len(bytearray(self.payload)) + 6

    def valid(self):
        return (self.checkSumA, self.checkSumB) == self.calculate_checksum()


class PprzParser:
    def default_callback(pkg):
        print(pkg)

    def __init__(self, callback=default_callback, callback_args=(), callback_kwargs={}):
        self.callback = callback
        self.data = bytearray([])
        self.callback_args = callback_args
        self.callback_kwargs = callback_kwargs

    def data_in(self, data):
        self.data = self.data + bytearray(data)
        while(len(self.data) > 6):
            if self.data[0] != 0x99:
                self.data = self.data[1:]
            else:
                break
        if len(self.data) < 6:
            return
        if len(self.data) < self.data[1]:
            return
        pkg = Package.from_pprz(self.data[:self.data[1]])
        self.data = self.data[self.data[1]:]
        self.callback(pkg, *self.callback_args, **self.callback_kwargs)
        self.data_in(bytearray([]))

    def reset_data(self):
        self.data = bytearray([])

def parse_range(pkg, output=True):
    if pkg.msg_id == 254:
        src, dest, dist = struct.unpack("=BBd", pkg.payload)
        if output:
            print("received RANGE from %s" % pkg.sender_id)
            print("src: " + str(src) + " --> " + str(dest) + " :  " + str(dist))
        return dist, src, dest
    elif pkg.msg_id == 2 and output:
        print("received ALIVE from %s" % pkg.sender_id)
    return None

def parse_height(pkg, output=True):
    if pkg.msg_id == 253:
        height = struct.unpack("h", pkg.payload)
        if output:
            print("recieved Height from %s" % pkg.sender_id)
            print(height)
        return height


def change_address_package(addr=2):
    return Package(msg_id=2, sender_id=addr, payload=[0,0,0,0])

