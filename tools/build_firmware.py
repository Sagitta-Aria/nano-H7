"""Build only with Keil ARM Compiler 5; never downloads firmware to a board."""
from pathlib import Path
import argparse
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
FW = ROOT / "firmware"


def sources():
    """Return the complete link inputs, shared with the generated Keil project."""
    kernel = FW / "vendor/FreeRTOS"
    return [*sorted((FW / "src").glob("*.c")),
            FW / "vendor/CMSIS/Device/ST/STM32H7xx/Source/Templates/system_stm32h7xx.c",
            *[kernel / name for name in ("tasks.c", "queue.c", "list.c",
                "portable/MemMang/heap_4.c", "portable/RVDS/ARM_CM4F/port.c")]]


def includes():
    return [FW / p for p in ("include", "vendor/CMSIS/Include",
        "vendor/CMSIS/Device/ST/STM32H7xx/Include", "vendor/FreeRTOS/include",
        "vendor/FreeRTOS/portable/RVDS/ARM_CM4F")]


def build(compiler):
    """Compile and link reproducibly; nonzero tool status aborts immediately."""
    out = FW / "build"
    out.mkdir(exist_ok=True)
    common = ["--cpu=Cortex-M7", "--fpu=FPv5_D16", "--apcs=interwork", "-g"]
    objects = []
    for source in sources():
        obj = out / (source.stem + ".o")
        command = [str(compiler / "armcc.exe"), *common, "--c99", "-O2",
                   "--library_type=microlib", "--split_sections", "-DSTM32H743xx", "-D__MICROLIB",
                   *["-I" + str(p) for p in includes()], "-c", str(source), "-o", str(obj)]
        subprocess.run(command, check=True)
        objects.append(obj)
    startup = out / "startup_stm32h743xx.o"
    subprocess.run([str(compiler / "armasm.exe"), *common, "--pd", "__MICROLIB SETA 1",
                    str(FW / "startup_stm32h743xx.s"), "-o", str(startup)], check=True)
    axf = out / "robot_h743.axf"
    subprocess.run([str(compiler / "armlink.exe"), *map(str, objects), str(startup),
        "--scatter", str(FW / "memory.sct"), "--library_type=microlib",
        "--map", "--symbols", "--info=sizes,totals,unused", "--list", str(out / "robot_h743.map"),
        "-o", str(axf)], check=True)
    for flag, name in [("--i32combined", "robot_h743.hex"), ("--bin", "robot_h743.bin")]:
        subprocess.run([str(compiler / "fromelf.exe"), flag, "--output", str(out / name), str(axf)], check=True)
    print("Build OK:", axf)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", type=Path, default=Path("D:/keil/ARM/ARMCC/bin"))
    args = parser.parse_args()
    if not (args.compiler / "armcc.exe").exists():
        sys.exit("ARM Compiler 5 not found; specify --compiler <ARMCC/bin>.")
    build(args.compiler)
