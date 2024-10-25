import math
from enum import Enum

import plotly.graph_objects as go


class Map(Enum):
    RUST_APP = 1
    ROM_BOOTLOADER = 2
    IDF_BOOTLOADER = 3


MB = 1024 * 1024
KB = 1024
VALUE_SCALING_POWER = 0.15
MAP = Map.IDF_BOOTLOADER


class Memory:
    memories = []

    def __init__(self, name, address, size, depth, dummy=False, dummy_parent=False):
        self.name = name
        self.address = address
        self.size = size
        self.address_end = address + size - 1
        self.depth = depth
        self.dummy = dummy
        self.parent = self.get_parent(name, address, size, depth)
        if self.parent is not None:
            self.parent.children.append(self)
        self.children = []
        self.value = self.get_value(depth, size, dummy, self.parent)

        if dummy:
            self.text = None
        else:
            self.text = f"{name}<br>0x{address:09_X} - 0x{self.address_end:09_X}"
        if dummy:
            self.hovertext = None
        elif size < 0x1_0000:
            self.hovertext = f"size: 0x{size:04X} ({size // KB}kB)"
        elif size < MB:
            self.hovertext = f"size: 0x{size:09_X} ({size // KB}kB)"
        else:
            self.hovertext = f"size: 0x{size:09_X} ({size // MB}MB)"
        if depth == 0:
            self.color = 'lightgrey'
        elif dummy and not dummy_parent:
            self.color = 'white'
        else:
            self.color = None

        Memory.memories.append(self)

    @staticmethod
    def get_parent(name, address, size, depth):
        if depth == 0:
            return None
        parent_depth = depth - 1
        address_end = address + size - 1
        parent = None
        for mem in Memory.memories:
            if mem.depth == parent_depth and mem.address <= address <= mem.address_end:
                if address_end > mem.address_end:
                    print(f"Memory {name} is too large for parent {mem.name}")
                parent = mem
            if mem.depth == depth:
                if mem.address <= address <= mem.address_end:
                    print(f"Memory {name} overlaps with {mem.name}")
                if mem.address <= address_end <= mem.address_end:
                    print(f"Memory {name} overlaps with {mem.name}")

        if parent is not None:
            return parent
        # create dummy inbetween parent
        return Memory(f" {name} parent", address, size, parent_depth, dummy=True, dummy_parent=True)

    @staticmethod
    def get_value(depth, size, dummy, parent):
        if depth == 0:
            return 0
        elif depth == 1:
            # modify the height of the first set of memories
            # so that very different sized regions don't have drastically different heights
            if dummy:
                return math.pow(size, VALUE_SCALING_POWER / 2)
            return math.pow(size, VALUE_SCALING_POWER)
        else:
            return parent.value * size / parent.size

    @classmethod
    def fixup_root_value(cls):
        # set the root value to the sum of all the children
        root = cls.memories[0]
        root.value = sum(memory.value for memory in cls.memories if memory.parent is root)

    @classmethod
    def add_dummy_memories(cls):
        # add dummy memories for gaps
        for memory in cls.memories:
            count = 0
            empty_range_start = memory.address
            filled_range_end = memory.address
            if memory.dummy or len(memory.children) == 0:
                continue
            for child in memory.children:
                if child.address > empty_range_start:
                    # found gap
                    dummy_size = child.address - empty_range_start
                    Memory(f"{memory.name} dummy {count}", empty_range_start, dummy_size, memory.depth + 1,
                           dummy=True)
                    count += 1

                empty_range_start = max(empty_range_start, child.address_end + 1)
                filled_range_end = max(filled_range_end, child.address_end)
            if filled_range_end < memory.address_end:
                # gap at the end
                dummy_size = memory.address_end - filled_range_end
                Memory(f"{memory.name} dummy {count}", filled_range_end + 1, dummy_size, memory.depth + 1,
                       dummy=True)

    @classmethod
    def sort_memories(cls):
        cls.memories.sort(key=lambda memory: (memory.depth, memory.address))

    def __repr__(self):
        return self.text


def kb(size: int) -> int:
    return size * 1024


def mb(size: int) -> int:
    return size * 1024 * 1024


# whole address space
memory_parent = Memory("Memory Map", 0, 2 ** 32, depth=0)

# memory regions
Memory("External memory (Data bus)", 0x3C00_0000, mb(32), depth=1)
Memory("Internal SRAM (Data bus)", 0x3FC8_8000, kb(480), depth=1)
Memory("Internal ROM (Data bus)", 0x3FF0_0000, kb(128), depth=1)
Memory("Internal ROM (Instruction bus)", 0x4000_0000, kb(384), depth=1)
Memory("Internal SRAM (Instruction bus)", 0x4037_0000, kb(448), depth=1)
Memory("External memory (Instruction bus)", 0x4200_0000, mb(32), depth=1)
Memory("Internal RTC SLOW Memory", 0x5000_0000, kb(8), depth=1)
Memory("Peripherals", 0x6000_0000, kb(836), depth=1)
Memory("Internal RTC FAST Memory", 0x600F_E000, kb(8), depth=1)

# subregions
Memory("Internal ROM 1 (Data bus)", 0x3FF0_0000, kb(128), depth=2)
Memory("Internal SRAM 1 (Data bus)", 0x3FC8_8000, kb(416), depth=2)
Memory("Internal SRAM 2 (Data bus)", 0x3FCF_0000, kb(64), depth=2)
Memory("Internal ROM 0 (Instruction bus)", 0x4000_0000, kb(256), depth=2)
Memory("Internal ROM 1 (Instruction bus)", 0x4004_0000, kb(128), depth=2)
Memory("Internal SRAM 0 (Instruction bus)", 0x4037_0000, kb(32), depth=2)
Memory("Internal SRAM 1 (Instruction bus)", 0x4037_8000, kb(416), depth=2)

Memory("UART Controller 0", 0x6000_0000, kb(4), depth=2)
Memory("SPI Controller 1", 0x6000_2000, kb(4), depth=2)
Memory("SPI Controller 0", 0x6000_3000, kb(4), depth=2)
Memory("GPIO", 0x6000_4000, kb(4), depth=2)
Memory("eFuse Controller", 0x6000_7000, kb(4), depth=2)
Memory("Low-Power Management", 0x6000_8000, kb(4), depth=2)
Memory("IO MUX", 0x6000_9000, kb(4), depth=2)
Memory("I2S Controller 0", 0x6000_F000, kb(4), depth=2)
Memory("UART Controller 1", 0x6001_0000, kb(4), depth=2)
Memory("I2C Controller 0", 0x6001_3000, kb(4), depth=2)
Memory("UHCI0", 0x6001_4000, kb(4), depth=2)
Memory("Remote Control Peripheral", 0x6001_6000, kb(4), depth=2)
Memory("Pulse Count Controller", 0x6001_7000, kb(4), depth=2)
Memory("LED PWM Controller", 0x6001_9000, kb(4), depth=2)
Memory("Motor Control PWM 0", 0x6001_E000, kb(4), depth=2)
Memory("Timer Group 0", 0x6001_F000, kb(4), depth=2)
Memory("Timer Group 1", 0x6002_0000, kb(4), depth=2)
Memory("RTC SLOW Memory", 0x6002_1000, kb(8), depth=2)
Memory("System Timer", 0x6002_3000, kb(4), depth=2)
Memory("SPI Controller 2", 0x6002_4000, kb(4), depth=2)
Memory("SPI Controller 3", 0x6002_5000, kb(4), depth=2)
Memory("SYSCON", 0x6002_6000, kb(4), depth=2)
Memory("I2C Controller 1", 0x6002_7000, kb(4), depth=2)
Memory("SD/MMC Host Controller", 0x6002_8000, kb(4), depth=2)
Memory("Two-wire Automotive Interface", 0x6002_B000, kb(4), depth=2)
Memory("Motor Control PWM 1", 0x6002_C000, kb(4), depth=2)
Memory("I2S Controller 1", 0x6002_D000, kb(4), depth=2)
Memory("UART controller 2", 0x6002_E000, kb(4), depth=2)
Memory("USB Serial/JTAG Controller", 0x6003_8000, kb(4), depth=2)
Memory("USB External Control registers", 0x6003_9000, kb(4), depth=2)
Memory("AES Accelerator", 0x6003_A000, kb(4), depth=2)
Memory("SHA Accelerator", 0x6003_B000, kb(4), depth=2)
Memory("RSA Accelerator", 0x6003_C000, kb(4), depth=2)
Memory("Digital Signature", 0x6003_D000, kb(4), depth=2)
Memory("HMAC Accelerator", 0x6003_E000, kb(4), depth=2)
Memory("GDMA Controller", 0x6003_F000, kb(4), depth=2)
Memory("ADC Controller", 0x6004_0000, kb(4), depth=2)
Memory("Camera-LCD Controller", 0x6004_1000, kb(4), depth=2)
Memory("USB core registers", 0x6008_0000, kb(256), depth=2)
Memory("System Registers", 0x600C_0000, kb(4), depth=2)
Memory("PMS Registers", 0x600C_1000, kb(4), depth=2)
Memory("Interrupt Matrix", 0x600C_2000, kb(4), depth=2)
Memory("External Memory Encryption and Decryption", 0x600C_C000, kb(4), depth=2)
Memory("World Controller", 0x600D_0000, kb(4), depth=2)

if MAP == Map.RUST_APP:
    # rust application
    RESERVE_ICACHE = 0x8000
    VECTORS_SIZE = 0x400
    vectors_seg_start = 0x4037_0000 + RESERVE_ICACHE
    iram_seg_start = vectors_seg_start + VECTORS_SIZE

    Memory("vectors_seg", vectors_seg_start, VECTORS_SIZE, depth=3)
    Memory("RWTEXT (iram_seg)", iram_seg_start, kb(328) - VECTORS_SIZE - RESERVE_ICACHE, depth=3)
    dram_seg = Memory("RWDATA (dram_seg)", 0x3FC8_8000, 345856, depth=3)
    Memory("ROTEXT (irom_seg)", 0x4200_0020, mb(4) - 0x20, depth=3)
    Memory("RODATA (drom_seg)", 0x3C00_0020, mb(4) - 0x20, depth=3)
    Memory("rtc_slow_seg", 0x5000_0000, kb(8), depth=3)
    Memory("RTC_FAST_RWTEXT/RTC_FAST_RWDATA (rtc_fast_seg)", 0x600F_E000, kb(8), depth=3)
    Memory("dram2_seg", dram_seg.address + dram_seg.size, 0x3fced710 - (dram_seg.address + dram_seg.size), depth=3)

if MAP == Map.ROM_BOOTLOADER:
    # ROM bootloader
    # Shared buffers, used in UART/USB/SPI download mode only
    Memory("boot_shared_buffers", 0x3fcd7e00, 0x3fce9704 - 0x3fcd7e00, depth=3)
    # PRO CPU stack, can be reclaimed as heap after RTOS startup
    Memory("boot_pro_cpu_stack", 0x3fce9710, 0x3fceb710 - 0x3fce9710, depth=3)
    # APP CPU stack, can be reclaimed as heap after RTOS startup
    Memory("boot_app_cpu_stack", 0x3fceb710, 0x3fced710 - 0x3fceb710, depth=3)
    # ROM .bss and .data (not easily reclaimable)
    Memory("boot_rom_bss_data", 0x3fced710, 0x3fcf0000 - 0x3fced710, depth=3)

if MAP == Map.IDF_BOOTLOADER:
    # idf bootloader
    iram_dram_offset = 0x6f0000
    bootloader_usable_dram_end = 0x3fce9700
    bootloader_stack_overhead = 0x2000
    bootloader_dram_seg_len = 0x5000
    bootloader_iram_loader_seg_len = 0x7000
    bootloader_iram_seg_len = 0x3000
    bootloader_dram_seg_end = bootloader_usable_dram_end - bootloader_stack_overhead
    bootloader_dram_seg_start = bootloader_dram_seg_end - bootloader_dram_seg_len
    bootloader_iram_loader_seg_start = bootloader_dram_seg_start - bootloader_iram_loader_seg_len + iram_dram_offset
    bootloader_iram_seg_start = bootloader_iram_loader_seg_start - bootloader_iram_seg_len

    Memory("boot_dram_seg", bootloader_dram_seg_start, bootloader_dram_seg_len, depth=3)
    Memory("boot_iram_seg", bootloader_iram_seg_start, bootloader_iram_seg_len, depth=3)
    Memory("boot_iram_loader_seg", bootloader_iram_loader_seg_start, bootloader_iram_loader_seg_len, depth=3)

Memory.add_dummy_memories()
Memory.sort_memories()
Memory.fixup_root_value()

labels = [memory.text for memory in Memory.memories]
ids = [memory.name for memory in Memory.memories]
values = [memory.value for memory in Memory.memories]
parents = [memory.parent.name if memory.parent is not None else "" for memory in Memory.memories]
hovertext = [memory.hovertext for memory in Memory.memories]
colors = [memory.color for memory in Memory.memories]

icicle = go.Icicle(
    labels=labels,
    ids=ids,
    values=values,
    parents=parents,
    hovertext=hovertext,
    branchvalues="total",
    hoverinfo="label+text",
    sort=False,
    marker={'colors': colors}
)
fig = go.Figure(icicle)
fig.update_layout(margin=dict(t=0, l=0, r=0, b=0))
fig.show()
fig.write_html("memory_map.html")
