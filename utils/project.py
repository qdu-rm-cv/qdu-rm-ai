# -*- coding:utf-8 -*-
# ! /usr/bin/python3

# run 'python ./utils/project.py <COMMAND>'
# -- <COMMAND> : description\n--")
# -- help      : print help info
# -- version   : print version
# -- env       : get environment quickly
# -- init      : git submodule update --init --recursive
# -- pack      : pack install generation files
# -- refresh   : rmdir 'build' dir and build
# -- build     : cd build && cmake ..
# -- test      : run ctest

import os
import sys
import logging
import pwd

VERSION = "0.1.6"
LOG_FILE_NAME = "project.log"
LOG_FMT = "%(asctime)s - %(levelname)s - [%(filename)s:%(lineno)s]  %(message)s"

root = os.path.dirname(os.path.abspath(__file__))[0:-5]
build_dir = os.path.join(root, 'build')
log_fullname = f"{build_dir}/{LOG_FILE_NAME}"

if not os.path.exists(build_dir):
    os.mkdir(build_dir)

COMMAND_CMAKE = f"/usr/bin/cmake --no-warn-unused-cli -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=TRUE " \
    f"-B{build_dir} -G Ninja -DCMAKE_C_COMPILER:FILEPATH=/usr/bin/gcc " \
    f"-DCMAKE_CXX_COMPILER:FILEPATH=/usr/bin/g++ -S{root} -DCMAKE_BUILD_TYPE:STRING="
COMMAND_DEBUG = COMMAND_CMAKE + "Debug "
COMMAND_RELEASE = COMMAND_CMAKE + "Release "
COMMAND_SERVER = COMMAND_RELEASE + " -DWITH_UI=OFF -DWITH_MCU=OFF -DWITH_CAMERA=OFF"
COMMAND_BUILD = f"/usr/bin/cmake --build {build_dir} --config Debug -j8 --target all --"
COMMAND_INSTALL = f"/usr/bin/cmake --build {build_dir} --config Release -j8 --target install --"
COMMAND_TEST = f"/usr/bin/ctest -j14 -C Release -T test --output-on-failure --test-dir {build_dir}"

logger = logging.getLogger("logger")
formatter = logging.Formatter(fmt=LOG_FMT)
stream_handler = logging.StreamHandler(sys.stdout)
stream_handler.setLevel(logging.WARNING)
stream_handler.setFormatter(formatter)
file_handler = logging.FileHandler(log_fullname, mode='a', encoding='utf-8')
file_handler.setFormatter(formatter)
file_handler.setLevel(logging.DEBUG)
logger.addHandler(file_handler)
logger.addHandler(stream_handler)
logger.setLevel(logging.DEBUG)


def help():
    logger.info("[help] start")
    print("----------------------------------------------------------------")
    print("- qdu-rm-ai project.py")
    print("-  run 'python ./utils/project.py <COMMAND>'\n-")
    print("-- <COMMAND> : description\n--")
    print("-- help      : print help info")
    print("-- version   : print version")
    print("-- env       : get environment quickly")
    print("-- init      : git submodule update --init --recursive")
    print("-- pack      : pack install generation files")
    print("-- refresh   : rmdir 'build' dir and build")
    print("-- build     : cd build && cmake ..")
    print("-- test      : run ctest")
    print("----------------------------------------------------------------")
    logger.info("[help] success")


def version():
    logger.info("[version] start")
    print(f"qdu-rm-ai --branch=2023 --version={VERSION}")
    logger.info("[version] success")


def env():
    logger.info("[env] start")
    env_path = os.path.join(root, "../env")
    if os.path.exists(f"{env_path}"):
        os.system(f"rm -rf {env_path}")
        print(f"{env_path} exist")
    os.system(f"git clone https://github.com/qdu-rm-cv/environment.git {env_path}")
    # os.system(f"git clone https://gitee.com/qdu-rm-2022/environment.git {env_path}")
    os.system(f"sudo chmod 777 {env_path}/shell/*")
    os.system(f"{env_path}/shell/env_dep_install.sh")
    logger.info("[env] success")
    pass


def init():
    logger.info("[init] start")
    os.system("git submodule update --init --recursive")
    # os.chdir(f"{root}/third_party/yolov5")
    # os.system("pip3 install -r requirements.txt")
    os.system("sudo apt install -y python-is-python3 python3-pip clang-format")
    os.system("sudo python -m pip install pylint cpplint autopep8 yapf")
    logger.warning("[init] success")


def pack():
    logger.info("[pack] start")
    os.system(COMMAND_RELEASE)
    os.system(COMMAND_INSTALL)
    os.system(f"zip -r {root}/Release.zip {root}/runtime {build_dir}/install")
    logger.info("[pack] success")


def refresh():
    logger.info("[refresh] start")
    logger.debug(build_dir)
    if os.path.exists(build_dir):
        os.system(f"mv {log_fullname} {root}")
        os.system(f"rm -rf {build_dir}")
    os.mkdir(build_dir)
    if os.path.exists(f"{root}/{LOG_FILE_NAME}"):
        os.system(f"mv {root}/{LOG_FILE_NAME} {log_fullname}")
    os.system(COMMAND_DEBUG)
    logger.info("[refresh] success")


def build():
    logger.info("[build] start")
    os.system(COMMAND_SERVER)
    os.system(COMMAND_BUILD)
    logger.info("[build] success")


def test():
    logger.info("[test] start")
    os.system(COMMAND_TEST)
    logger.info("[test] success")


def menu(command):
    if command in ["help", "--help", "-h", "-H", "--h", "--H", "-?", "--?"]:
        help()
    elif command in ["version", "--version", "-v", "-V", "--v", "--V"]:
        version()
    elif command in ["env", "--env", "-e", "-E", "--e", "--E"]:
        env()
    elif command in ["init", "--init", "-i", "-I", "--i", "--I"]:
        init()
    elif command in ["refresh", "--refresh", "-r", "-R", "--r", "--R"]:
        refresh()
    elif command in ["build", "--build", "-b", "-B", "--b", "--B"]:
        build()
    elif command in ["test", "--test", "-t", "-T", "--t", "--T"]:
        test()
    elif command in ["pack", "--pack", "-p", "-P", "--p", "--P"]:
        pack()
    else:
        help()


if __name__ == '__main__':
    cmd = sys.argv
    logger.info(f"[user: {pwd.getpwuid(os.getuid())[0]}, command: {cmd[-1]}]")
    if len(cmd) > 2:
        help()
    else:
        menu(cmd[-1])