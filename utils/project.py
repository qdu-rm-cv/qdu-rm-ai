# -*- coding:utf-8 -*-
# ! /usr/bin/python3

import os
import sys
import logging
import pwd

COMMAND = "/usr/bin/cmake --no-warn-unused-cli -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=TRUE " \
    "-B/home/eric/env/qdu-rm-ai/build -G Ninja -DCMAKE_C_COMPILER:FILEPATH=/usr/bin/gcc " \
    "-DCMAKE_CXX_COMPILER:FILEPATH=/usr/bin/g++ -S/home/eric/env/qdu-rm-ai"

DEBUG_COMMAND = COMMAND + " -DCMAKE_BUILD_TYPE:STRING=Debug "
RELEASE_COMMAND = COMMAND + " -DCMAKE_BUILD_TYPE:STRING=Release "
VERSION = "0.1.0"
LOG_FILE_NAME = "project.log"

root = os.path.dirname(os.path.abspath(__file__))[0:-5]
build_dir = os.path.join(root, 'build')
log_fullname = f"{build_dir}/{LOG_FILE_NAME}"

logger = logging.getLogger("logger")
formatter = logging.Formatter(
    fmt="%(asctime)s - %(levelname)s - [%(filename)s:%(lineno)s]  %(message)s")
stream_handler = logging.StreamHandler(sys.stdout)
stream_handler.setLevel(logging.WARNING)
stream_handler.setFormatter(formatter)
file_handler = logging.FileHandler(f"{log_fullname}", mode='a', encoding='utf-8')
file_handler.setFormatter(formatter)
file_handler.setLevel(logging.DEBUG)
logger.addHandler(file_handler)
logger.addHandler(stream_handler)
logger.setLevel(logging.DEBUG)


def refresh():
    logger.info("[refresh] start")
    logger.debug(build_dir)
    if os.path.exists(build_dir):
        os.system(f"mv {log_fullname} {root}")
        os.system(f"rm -rf {build_dir}")
    os.mkdir(build_dir)
    if os.path.exists(f"{root}{LOG_FILE_NAME}"):
        os.chdir(build_dir)
        os.mkdir("generated")
        os.system(f"mv {root}/project.log {build_dir}/generated")
    os.system(DEBUG_COMMAND)
    logger.info("[refresh] success")


def init():
    logger.info("[init] start")
    os.system("git submodule update --init --recursive")
    # os.chdir(f"{root}/third_party/yolov5")
    # os.system("pip3 install -r requirements.txt")
    logger.warning("[init] success")


def build():
    logger.info("[build] start")
    os.system(RELEASE_COMMAND)
    logger.info("[build] success")


def version():
    logger.info("[version] start")
    print(f"qdu-rm-ai --branch=2023 --version={VERSION}")
    logger.info("[version] success")


def help():
    logger.info("[help] start")
    print("----------------------------------------------------------------")
    print("- qdu-rm-ai project.py")
    print("-  run 'python ./utils/project.py <COMMAND>'\n-")
    print("-- <COMMAND> : description\n--")
    print("-- help      : print help info")
    print("-- version   : print version")
    print("-- build     : cd build && cmake ..")
    print("-- refresh   : rmdir 'build' dir and build")
    print("-- init      : git submodule update --init --recursive")
    print("----------------------------------------------------------------")
    logger.info("[help] success")


def menu(command):
    if command in ["help", "--help", "-h", "-H", "--h", "--H", "-?", "--?"]:
        help()
    elif command in ["version", "--version", "-v", "-V", "--v", "--V"]:
        version()
    elif command in ["build", "--build", "-b", "-B", "--b", "--B"]:
        build()
    elif command in ["refresh", "--refresh", "-r", "-R", "--r", "--R"]:
        refresh()
    elif command in ["init", "--init", "-i", "-I", "--i", "--I"]:
        init()
    else:
        help()


if __name__ == '__main__':
    cmd = sys.argv
    logger.info(f"<<user: {pwd.getpwuid(os.getuid())[0]}, command: {cmd[-1]}>>")
    if len(cmd) > 2:
        help()
    else:
        menu(cmd[-1])