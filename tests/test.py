import subprocess
import socket
import struct
import pathlib
import sys
import os
import shutil

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
GCC = {"exe": f"gcc{EXE_EXT}", "inc": "-I{}", "out": "-o{}", "flags": []}
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


def main():
    compile_c()
    return
    data = subprocess.run(['test.exe'], stdout=subprocess.PIPE)
    raw = data.stdout.decode('utf-8')
    bts = raw.split(',')

    var = [int(b) for b in bts if b]
    print(var)
    data = struct.pack('B' * len(var), *var)
    print(data, len(data))


    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    sock.sendto(data, ('192.168.0.100', 10020))

if __name__ == "__main__":
    main()
