import subprocess
import socket
import serial
import struct
import pathlib
import sys
import shutil
import time
import argparse

ROOT = (pathlib.Path(__file__).parent / "..").resolve()

PROTOBUF_PATH = ROOT / "Libraries" / "protobuf"
MESSAGES_PATH = ROOT / "Libraries" / "proto-messages" / "proto_c"

if sys.platform == "win32" or sys.platform == "cygwin":
    EXE_EXT = ".exe"
else:
    EXE_EXT = ""

SOURCE = [str(ROOT / "tests" / "test.c"),
          str(PROTOBUF_PATH / "protobuf-c" / "protobuf-c.c"),
          str(MESSAGES_PATH / "parsed_vision" / "parsed_vision.pb-c.c"),
          str(MESSAGES_PATH / "robot_action" / "robot_action.pb-c.c"),
          ]
EXE = ROOT / "tests" / f"message{EXE_EXT}"
BUILD_DIR = ROOT / "tests" / "build"

MSVC = {"exe": "cl.exe", "inc": "/I{}", "out": "/Fe:{}", "flags": ["/nologo", f"/Fo:{BUILD_DIR}/"]}
GCC = {"exe": f"gcc{EXE_EXT}", "inc": "-I{}", "out": "-o{}", "flags": ["-g"]}
CLANG = {"exe": "clang{EXE_EXT}", "inc" : "-I{}", "out": "-o{}", "flags": []}

if sys.platform == "win32":
    COMPILERS = [MSVC, GCC, CLANG]
elif sys.platform == "darwin":
    COMPILERS = [CLANG, GCC]
elif sys.platform == "cygwin":
    COMPILERS = [GCC, CLANG, MSVC]
else:
    COMPILERS = [GCC, CLANG]

def compile_c():
    if EXE.exists():
        last_mod = 0.0
        for file in SOURCE:
            m = pathlib.Path(file).lstat().st_mtime
            if m > last_mod:
                last_mod = m
        if EXE.lstat().st_mtime > last_mod:
            return
    BUILD_DIR.mkdir(exist_ok=True)
    for comp in COMPILERS:
        exe = shutil.which(comp['exe'])
        if exe is None:
            continue
        args = [exe] + comp["flags"] + \
               [comp['inc'].format(PROTOBUF_PATH.resolve()),
                comp['inc'].format(MESSAGES_PATH.resolve()),
                comp['out'].format(EXE)] + SOURCE
        res = subprocess.run(args)
        if res.returncode != 0 or not EXE.exists():
            raise RuntimeError("Failed to compile program")
        shutil.rmtree(BUILD_DIR)
        return
    raise RuntimeError("Could not find compiler")


class Writer:
    def __init__(self, con: str) -> None:
        parts = con.split(':', 1)
        if len(parts) == 1:
            self._dev = serial.Serial(con, 115200)
            self.write = self._serial
        else:
            self.write = self._udp
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._addr = ((parts[0], int(parts[1])))


    def _udp(self, msg: bytes) -> None:
        self._socket.sendto(msg, self._addr)

    def _serial(self, msg: bytes) -> None:
        data = struct.pack('<H', len(msg) + 1) + b'\x00' + msg
        time.sleep(1)
        self._dev.write(data)

def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("-n", action="store", default=1, type=int)
    parser.add_argument("-d", action="store", default=20, type=int)
    parser.add_argument('--con', action="store", default="192.168.0.100:10020")

    args = parser.parse_args()

    compile_c()

    data = subprocess.run([str(EXE)], stdout=subprocess.PIPE)
    raw = data.stdout.decode('utf-8')
    bts = raw.split(',')

    var = [int(b) for b in bts if b]
    data = struct.pack('B' * len(var), *var)
    
    writer = Writer(args.con)

    delay = float(args.d) / 1000

    print(f"Sending {len(var)} bytes")

    stamp = time.time()

    try:
        while True:
            if writer._dev.in_waiting > 0:
                out = writer._dev.read(writer._dev.in_waiting)
                s = out.decode('ascii')
                print(s, end="")
            if time.time() - stamp > 1.0:
                print(f"---Writing {len(data)} msg")
                writer.write(data)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
