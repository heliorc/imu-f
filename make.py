#!/usr/bin/env python
# -*- coding: utf-8 -*-
#Cubic spline fitter

from __future__ import print_function

import re
import sys
import threading
import subprocess
import glob
import sys
import ntpath
import os
import platform
import argparse
import ctypes as c

from collections import namedtuple

try:
    # Python 3 name
    import queue
except ImportError:
    # Python 2 name
    import Queue as queue

class ColorFallback(object):
    def __getattr__(self, attr):
        return ""

try:
    from colorama import init, Fore, Back, Style
    init(convert=True)
except ImportError:
    Fore = Style = ColorFallback()

# Magic code
# add new method for python string object
# used
# STM32F1_MCU_DIR.path
class PyObject_HEAD(c.Structure):
    _fields_ = [
        ('HEAD', c.c_ubyte * (object.__basicsize__ - c.sizeof(c.c_void_p))),
        ('ob_type', c.c_void_p)
    ]

_get_dict = c.pythonapi._PyObject_GetDictPtr
_get_dict.restype = c.POINTER(c.py_object)
_get_dict.argtypes = [c.py_object]

def get_dict(object):
    return _get_dict(object).contents.value

@property
def get_path_method(self):
    if platform.system() == 'Windows':
        return self.replace("/", "\\")
    return self

get_dict(str)['path'] = get_path_method



this_dir = os.path.dirname(os.path.abspath(__file__))
OUTPUT_PATH = os.path.join(this_dir, "output")
LIBRARY_PATH = os.path.join(this_dir, "vendor")

parser = argparse.ArgumentParser(description='')
parser.add_argument('-C', "--clean", help="clean up output folder", action='store_true')
parser.add_argument('-D', "--debug", help="build debug target", action='store_true')
parser.add_argument('-j', "--threads", help="number of threads to run", default=4, type=int)
parser.add_argument('-v', "--verbose", help="print compiler calls", action='store_true')
parser.add_argument("-T", "--target", help="target controller to build", default="", nargs='*')
args = parser.parse_args()

IS_CLEANUP = args.clean

#if IS_CLEANUP:
if os.path.exists(OUTPUT_PATH):
    for root, dirs, files in os.walk(OUTPUT_PATH, topdown=False):
        for name in files:
            os.remove(os.path.join(root, name))
        for name in dirs:
            os.rmdir(os.path.join(root, name))
#    sys.exit(0)

TargetConfig = namedtuple('TargetConfig', [
    'target',
    'sourcefiles', 'sourcedirs',
    'cflags', 'asmflags', 'ldflags', 'useColor'
])

excluded_files = [
    ".*_template.c",
]

def configure_target(TARGET):
    STM32F4_ARCH_FLAGS_ADD = ""

    ################################################################################
    # Determine target variables and features


    FC_NAME = "C3PU"

    if TARGET == "F3":
        PROJECT = "C3PU"
        TARGET_DEVICE = "STM32F302x8"
        TARGET_SCRIPT = "stm32_flash_f30x_0x08002000_24k.ld"
        OPTIMIZE_FLAGS = "-O2"
        HSE_SPEED = str(16000000)
        THIS_ADDRESS = str(0x08002000)
        APP_ADDRESS  = str(0x08002000) #6k bl

    elif TARGET == "F3BL":
        PROJECT = "C3PUBL"
        TARGET_DEVICE = "STM32F302x8"
        TARGET_SCRIPT = "stm32_flash_f30x_0x08000000_8k.ld"
        OPTIMIZE_FLAGS = "-Os"
        HSE_SPEED = str(16000000)
        THIS_ADDRESS = str(0x08000000)
        APP_ADDRESS  = str(0x08002000) #6k bl

    elif TARGET == "F3SING":
        PROJECT = "C3PU"
        TARGET_DEVICE = "STM32F302x8"
        TARGET_SCRIPT = "stm32_flash_f30x_0x08000000_32k.ld"
        OPTIMIZE_FLAGS = "-O2"
        HSE_SPEED = str(16000000)
        THIS_ADDRESS = str(0x08000000)
        APP_ADDRESS  = str(0x08000000) #12k bl

    elif TARGET == "F3BING":
        PROJECT = "C3PUBL"
        TARGET_DEVICE = "STM32F302x8"
        TARGET_SCRIPT = "stm32_flash_f30x_0x08000000_12k.ld"
        OPTIMIZE_FLAGS = "-Og"
        HSE_SPEED = str(16000000)
        THIS_ADDRESS = str(0x08000000)
        APP_ADDRESS  = str(0x08003000) #12k bl

    elif TARGET == "F3_TEST":
        PROJECT = "C3PU"
        TARGET_SCRIPT = "stm32_flash_f30x_22k.ld"
        OPTIMIZE_FLAGS = "-O2"
        HSE_SPEED = str(8000000)
        THIS_ADDRESS = str(0x08000000)

    elif TARGET == "F3BL_TEST":
        PROJECT = "C3PUBL"
        TARGET_SCRIPT = "stm32_flash_f30x_12k.ld"
        OPTIMIZE_FLAGS = "-Og"
        HSE_SPEED = str(8000000)
        THIS_ADDRESS = str(0x08000000)

    else:
        print("ERROR - Select a target")
        exit(1)

    if (args.debug):
        os.system("PID=\"$(ps -elf | grep  openocd | grep -v 'grep' | sed -e 's/    / /g' | sed -e 's/   / /g' | sed -e 's/  / /g' | cut -d ' ' -f 3)\";kill $PID")
        os.system("openocd -s /usr/local/share/openocd/scripts -f /usr/local/share/openocd/scripts/interface/stlink-v2.cfg -f /usr/local/share/openocd/scripts/target/stm32f3x.cfg &> redirection &")

    TARGET_PROCESSOR_TYPE = "F3"

    #DFU_ADDRESS = str(0x1FF00000)
    #Not allowing DFU mode for the F3s
    DFU_ADDRESS  = str(0x08000000)
    BL_ADDRESS   = str(0x08000000)
    FLASH_END    = str(0x08008000)

    #extra D flags
    EXTRA_DEF_FLAGS = " -std=c99 -D__FPU_USED=1 -D__FPU_PRESENT=1 -DUSE_STDPERIPH_DRIVER"

    #extra source files to include not in the below dirs
    SOURCE_FILES = [
        this_dir + "/assembly/startup/startup_stm32f303xc.s",
        LIBRARY_PATH + "/CMSIS_std/DSP_Lib/Source/TransformFunctions/arm_bitreversal2.s"
    ]

    #All include dirs
    INCLUDE_DIRS = [
        "src",
        os.path.join("src", "stm32"),
        os.path.join("src", "target"),
        os.path.join("src", "board_comm"),
        os.path.join("src", "gyro"),
        os.path.join("src", "imu"),
        os.path.join("src", "bootloader"),
        os.path.join("src", "filter"),
        LIBRARY_PATH + "/CMSIS_std/Device/ST/STM32F30x/Include",
        LIBRARY_PATH + "/STM32F30x_StdPeriph_Driver/inc",
        LIBRARY_PATH + "/CMSIS_std/Include"
    ]
    #source dirs for all flie inclusion
    SOURCE_DIRS = [
        "src",
        os.path.join("src", "stm32"),
        os.path.join("src", "target"),
        os.path.join("src", "board_comm"),
        LIBRARY_PATH + "/CMSIS_std/Device/ST/STM32F30x/Source",
        LIBRARY_PATH + "/STM32F30x_StdPeriph_Driver/src"
    ]

    if PROJECT == "C3PUBL":
        print("C3PUBL - C3PUBL")
        SOURCE_DIRS.append(os.path.join("src", "bootloader"))
    elif PROJECT == "C3PU":
        print("C3PU - C3PU")
        SOURCE_DIRS.append(os.path.join("src", "imu"))
        SOURCE_DIRS.append(os.path.join("src", "gyro"))
        SOURCE_DIRS.append(os.path.join("src", "filter"))
        SOURCE_FILES.append(LIBRARY_PATH + "/CMSIS_std/DSP_Lib/Source/CommonTables/arm_common_tables.c")
        SOURCE_FILES.append(LIBRARY_PATH + "/CMSIS_std/DSP_Lib/Source/FastMathFunctions/arm_cos_f32.c")
        SOURCE_FILES.append(LIBRARY_PATH + "/CMSIS_std/DSP_Lib/Source/FastMathFunctions/arm_sin_f32.c")
        SOURCE_FILES.append(LIBRARY_PATH + "/CMSIS_std/DSP_Lib/Source/BasicMathFunctions/arm_mult_f32.c")
        SOURCE_FILES.append(LIBRARY_PATH + "/CMSIS_std/DSP_Lib/Source/TransformFunctions/arm_rfft_fast_f32.c")
        SOURCE_FILES.append(LIBRARY_PATH + "/CMSIS_std/DSP_Lib/Source/TransformFunctions/arm_cfft_f32.c")
        SOURCE_FILES.append(LIBRARY_PATH + "/CMSIS_std/DSP_Lib/Source/TransformFunctions/arm_cfft_q31.c")
        SOURCE_FILES.append(LIBRARY_PATH + "/CMSIS_std/DSP_Lib/Source/TransformFunctions/arm_rfft_fast_init_f32.c")
        SOURCE_FILES.append(LIBRARY_PATH + "/CMSIS_std/DSP_Lib/Source/TransformFunctions/arm_cfft_radix8_f32.c")
        SOURCE_FILES.append(LIBRARY_PATH + "/CMSIS_std/DSP_Lib/Source/ComplexMathFunctions/arm_cmplx_mag_f32.c")
    else:
        print("ERROR - Unknown Project")
        exit(1)



    ################################################################################
    # Set per target compilation options
    FLAGS = [
        " -D" + PROJECT,
        "PROJECT=" + PROJECT,
        "THIS_ADDRESS=" + THIS_ADDRESS,
        "DFU_ADDRESS=" + DFU_ADDRESS,
        "BL_ADDRESS=" + BL_ADDRESS,
        "APP_ADDRESS=" + APP_ADDRESS,
        "FLASH_END=" + FLASH_END
    ]

    EXTRA_DEF_FLAGS = EXTRA_DEF_FLAGS + " -D".join(FLAGS)

    DEF_FLAGS = "-DUSE_HAL_DRIVER -DHSE_VALUE="+HSE_SPEED+" -D" + FC_NAME +" -D" + TARGET_DEVICE + " -DARM_MATH_CM4 -D" + TARGET + " -D" + TARGET_DEVICE.lower() + " -D" + TARGET_PROCESSOR_TYPE + EXTRA_DEF_FLAGS

    ARCH_FLAGS = "-mthumb -mcpu=cortex-m4 -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -mtune=cortex-m4"

    ################################################################################
    # Set source and includes directories
    ###### Done during target selection now

    #exclude these files in above source dirs
    excluded_files = [
        ".*_template.c",
    ]

    linkerDir = os.path.join(this_dir, "scripts", "linker")
    ldScript = os.path.join(linkerDir, TARGET_SCRIPT)

    ################################################################################
    # compiler options

    # INCLUDES = " ".join("-I" + include for include in INCLUDE_DIRS) #this is stupid.
    INCLUDES = " -I" + " -I".join(INCLUDE_DIRS)

    LTO_FLAGS = "-flto -fuse-linker-plugin"
    DEBUG_FLAGS = "-ggdb3 -DDEBUG -Og"

    CFLAGS = " ".join([
        ARCH_FLAGS,
        LTO_FLAGS,
        DEF_FLAGS,
        DEBUG_FLAGS if args.debug else OPTIMIZE_FLAGS,
        INCLUDES,
        "-Wno-packed-bitfield-compat -Wno-unused -Wall -Wextra -Wmaybe-uninitialized -fno-unsafe-math-optimizations -Wdouble-promotion "
        "-ffunction-sections -fdata-sections -MMD -MP"
    ])

    ASMFLAGS = " ".join([
        ARCH_FLAGS,
        "-x assembler-with-cpp",
        INCLUDES,
        "-MMD -MP"
    ])

    mapFile = os.path.join("output", TARGET + ".map")
    LDFLAGS = " ".join([
        "-lm -nostartfiles --specs=nano.specs -lc -lnosys",
        ARCH_FLAGS,
        LTO_FLAGS,
        DEBUG_FLAGS if args.debug else OPTIMIZE_FLAGS,
        "-static",
        "-Wl,-gc-sections,-Map," + mapFile,
        "-Wl,-L" + linkerDir,
        "-Wl,--cref",
        "-T" + ldScript
    ])

    # if we're at a tty, then tell gcc to use colors
    if sys.stdout.isatty():
        colorFlag = "-fdiagnostics-color"
    else:
        colorFlag = ""

    ################################################################################
    # build return object with all needed parameters

    target_config = TargetConfig(
        target=TARGET,
        sourcefiles=SOURCE_FILES,
        sourcedirs=SOURCE_DIRS,
        cflags=CFLAGS,
        asmflags=ASMFLAGS,
        ldflags=LDFLAGS,
        useColor=colorFlag,
    )

    return target_config



asm_command = "arm-none-eabi-gcc -c {USECOLOR} -o output/{OUTPUT_FILE} {ASMFLAGS} {INPUT_FILE}"

compile_command = "arm-none-eabi-gcc -c {USECOLOR} -o output/{OUTPUT_FILE} {CFLAGS} {INPUT_FILE}"

link_command = "arm-none-eabi-gcc {USECOLOR} -o output/{OUTPUT_NAME}.elf {OBJS} {LDFLAGS}"

size_command = "arm-none-eabi-size output/{OUTPUT_NAME}.elf"

copy_obj_command = "arm-none-eabi-objcopy -O binary output/{OUTPUT_NAME}.elf output/{OUTPUT_NAME}.bin"


#excluded_files = [
#    ".*_template.c",
#]

THREAD_LIMIT = args.threads
threadLimiter = threading.BoundedSemaphore(THREAD_LIMIT)
locker = threading.Lock()
threadRunning = list()
isStop = False

def find_between( s, first, last ):
    try:
        start = s.index( first )
        end = s.index( last, start )
        return s[start:end]
    except ValueError:
        return ""

class CommandRunnerThread(threading.Thread):

    def __init__(self, command, output, target, *args, **kwargs):
        self.command = command.path  # we need to be sure all '/' are properly converted for Windows
        self.output = output.path    # store the output of the command for printing purposes
        self.target = target
        self.queue = kwargs.pop("queue", None)
        self.deps = kwargs.pop("dependencies", None)
        self.proc = None
        super(CommandRunnerThread, self).__init__(*args, **kwargs)
        self.stop_event = threading.Event()

    def run(self):
        if self.deps:
            while self.deps:
                if isStop:
                    return
                # wrap in try, in case dependency threads haven't started yet
                try:
                    # wait for first thread to be done
                    self.deps[0].join()
                    # it's done, pop it off deps
                    self.deps.pop(0)
                except RuntimeError:
                    pass

        with threadLimiter:
            with locker:
                threadRunning.append(self)

            try:
                self.run_command()
            finally:
                with locker:
                    threadRunning.remove(self)

    def run_command(self):
        if not self.command:
            return

        with locker:
            if isStop:
                return

            # figure out the output file path
            basedir, basename = os.path.split(self.output)
            _, ext = os.path.splitext(basename)
            # if the base directory doesn't exist, make it
            if not os.path.exists(basedir):
                os.makedirs(basedir)

        self.proc = subprocess.Popen(self.command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
        stdout_value, stderr_value = self.proc.communicate()

        # "arm-none-eabi-XXXX" -> "XXXX"
        executable = self.command.split(" ")[0]
        toolChainCmd = executable.split("-")[-1]

        with locker:
            # output all .c/.s file outputs with green, others with red
            if ext == ".o":
                foreColor = Fore.GREEN
            else:
                foreColor = Fore.RED
            print(Fore.MAGENTA + "%% {:s} ".format(self.target) + foreColor + toolChainCmd + " " + Style.RESET_ALL + basename)

            if (args.verbose):
                print(self.command)

            if stdout_value:
                print(stdout_value.decode())
            if stderr_value:
                print(stderr_value.decode())

            sys.stdout.flush()

        if self.queue:
            self.queue.put(self.proc.returncode)
        else:
            print(self.proc.returncode)

        self.proc = None

    def stop_command(self):
        if self.proc:
            try:
                self.proc.kill()
                self.proc.wait()
            except OSError:
                pass

        self.stop_event.set()


    def stopped(self):
        return self.stop_event.isSet()


def FileModified(fileName, target_config):
    # get the output file `output/.../filename.o`
    outputFile = os.path.join("output", makeObject(fileName.path, target_config.target))
    # if we haven't compiled, then return true
    if not os.path.exists(outputFile):
        return True

    # if input file is more recent than the output, return true
    if os.path.getmtime(fileName) > os.path.getmtime(outputFile):
        return True

    # if the target file is more recent than the output, return true
    target_file = "low_level_driver/boarddef.h".path
    if os.path.getmtime(target_file) > os.path.getmtime(outputFile):
        return True

    # get the dependency file `output/.../filename.d`
    outputBase, _ = os.path.splitext(outputFile)
    depFile = outputBase + ".d"

    # if we don't have a dependency file, return true
    if not os.path.exists(depFile):
        return True

    # check the dependency file
    with open(depFile, 'r') as f:
        for line in f:
            # the lines with dependencies start with a space
            if line[0] != " ":
                continue

            for dep in line.split():
                # we'll get the line continuation in this: "\"
                if dep == "\\":
                    continue
                # all dependencies should exist
                if not os.path.exists(dep):
                    return True
                # check if dependency is more recent
                if os.path.getmtime(dep) > os.path.getmtime(outputFile):
                    return True

    return False

def makeObject(fileName, target_dir):
    head, tail = os.path.split(fileName)
    root, ext = os.path.splitext(tail)
    if ext.lower() in (".c", ".s"):
        root = os.path.join(target_dir, root)
        return root + ".o"

    print("Unknown file type: " + tail)
    return tail

#This works, but the linker doesn't know how to find the files
#    root, ext = os.path.splitext(fileName)
#
#    # strip first directory from  "src/..." or "..//..."
#    _, root = root.split(os.sep, 1)
#    # send it to "output/target/"
#    root = os.path.join(TARGET_BOARD, root)
#
#    if ext.lower() in (".c", ".s"):
#        return root + ".o"
#
#    print("Unknown file type: " + tail)
#    return root

def ProcessList(fileNames, target_config):
    linkerObjs = []
    commands = []

    for fileName in fileNames:
        if any(re.match(ex_pattern, os.path.basename(fileName)) for ex_pattern in excluded_files):
            continue

        linkerObjs.append(os.path.join("output", makeObject(fileName, target_config.target)))
        if FileModified(fileName, target_config):
            if fileName[-2:] == ".s":
                commands.append(asm_command.format(
                    INPUT_FILE=fileName.path,
                    OUTPUT_FILE=makeObject(fileName.path, target_config.target),
                    ASMFLAGS=target_config.asmflags,
                    USECOLOR=target_config.useColor,
                ))
            elif fileName[-2:] == ".c":
                commands.append(compile_command.format(
                    INPUT_FILE=fileName.path,
                    OUTPUT_FILE=makeObject(fileName.path, target_config.target),
                    CFLAGS=target_config.cflags,
                    USECOLOR=target_config.useColor,
                ))
            else:
                raise Exception("Bad file type:", fileName)
    return commands, linkerObjs


def main():
    global isStop

    if not args.target:
        raise Exception("Output target must be specified!")

    try:
        os.mkdir("output")
    except:
        pass

    threads = []
    thread_queue = queue.Queue()

    for target in args.target:
        target_config = configure_target(target)

        commands = []
        linkerObjs = []

        # parse all directories for .c and .s files
        for directory in target_config.sourcedirs:
            # process each file, add commands and output files to list
            command, linkerObj = ProcessList(glob.glob(os.path.join(directory, "*.c")), target_config)
            commands.extend(command)
            linkerObjs.extend(linkerObj)

            command, linkerObj = ProcessList(glob.glob(os.path.join(directory, "*.s")), target_config)
            commands.extend(command)
            linkerObjs.extend(linkerObj)

        command, linkerObj = ProcessList(target_config.sourcefiles, target_config)
        commands.extend(command)
        linkerObjs.extend(linkerObj)

        # generate list of threads from all commands
        linkerThreads = []
        for command, linkerObj in zip(commands, linkerObjs):
            thread = CommandRunnerThread(command=command, output=linkerObj, target=target_config.target, queue=thread_queue)
            threads.append(thread)
            linkerThreads.append(thread)

        linkTarget = link_command.format(
            OUTPUT_NAME=target_config.target,
            OBJS=" ".join(linkerObjs),
            LDFLAGS=target_config.ldflags,
            USECOLOR=target_config.useColor,
        )
        linkOutput = "output/{OUTPUT_NAME}.elf".format(OUTPUT_NAME=target_config.target)
        linkThread = CommandRunnerThread(command=linkTarget, output=linkOutput, target=target_config.target, queue=thread_queue, dependencies=linkerThreads)
        threads.append(linkThread)

        sizeTarget = size_command.format(
            OUTPUT_NAME=target_config.target,
        )
        sizeOutput = "output/{OUTPUT_NAME}.elf".format(OUTPUT_NAME=target_config.target)
        sizeThread = CommandRunnerThread(command=sizeTarget, output=sizeOutput, target=target_config.target, queue=thread_queue, dependencies=[linkThread])
        threads.append(sizeThread)

        copyTarget = copy_obj_command.format(
            OUTPUT_NAME=target_config.target
        )
        copyOutput = "output/{OUTPUT_NAME}.bin".format(OUTPUT_NAME=target_config.target)
        copyThread = CommandRunnerThread(command=copyTarget, output=copyOutput, target=target_config.target, queue=thread_queue, dependencies=[sizeThread])
        threads.append(copyThread)

    # all threads are created, start them up
    for thread in threads:
        thread.start()

    while len(threadRunning) > 0:
        try:
            returncode = thread_queue.get(timeout=5)
        except queue.Empty:
            continue
        if returncode > 0:
            with locker:
                isStop = True
                for thread in threads:
                    thread.stop_command()
            break

    for thread in threads:
        thread.join()

    exit(0)

if __name__ == "__main__":
    main()