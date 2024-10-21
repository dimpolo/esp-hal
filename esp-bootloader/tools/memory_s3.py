import math

import plotly.graph_objects as go

# TODO insert hidden gap memories

MB = 1024 * 1024
KB = 1024
VALUE_SCALING_POWER = 0.25


class Memory:
    memories = []
    memories_dict = {}

    def __init__(self, name, address, size, scale_value=False):
        self.name = name
        self.address = address
        self.address_end = address + size - 1
        self.size = size
        self.parent = self.get_parent()
        if scale_value:
            # modify the height of the first set of memories
            # so that very different sized regions don't have drastically different heights
            self.value = math.pow(size, VALUE_SCALING_POWER)
        else:
            parent = Memory.memories_dict[self.parent]
            self.value = parent.value * size / parent.size

        self.text = f"{name}<br>0x{address:09_X} - 0x{self.address_end:09_X}"
        if size < 0x1_0000:
            self.hovertext = f"size: 0x{size:04X} ({size // KB}kB)"
        elif size < MB:
            self.hovertext = f"size: 0x{size:09_X} ({size // KB}kB)"
        else:
            self.hovertext = f"size: 0x{size:09_X} ({size // MB}MB)"

        Memory.memories.append(self)
        Memory.memories_dict[name] = self

    def get_parent(self):
        for memory in reversed(Memory.memories):
            if memory.address <= self.address <= memory.address_end:
                return memory.name
        return ""

    @classmethod
    def fixup_root_value(cls):
        # set the root value to the sum of all the children
        root = cls.memories[0]
        root.value = sum(memory.value for memory in cls.memories if memory.parent == root.name)


def kb(size: int) -> int:
    return size * 1024


def mb(size: int) -> int:
    return size * 1024 * 1024


# whole address space
memory_parent = Memory("Memory Map", 0, 2 ** 32, scale_value=True)

# memory regions
Memory("External memory (Data bus)", 0x3C00_0000, mb(32), scale_value=True)
Memory("Internal SRAM (Data bus)", 0x3FC8_8000, kb(480), scale_value=True)
Memory("Internal ROM (Data bus)", 0x3FF0_0000, kb(128), scale_value=True)
Memory("Internal ROM (Instruction bus)", 0x4000_0000, kb(384), scale_value=True)
Memory("Internal SRAM (Instruction bus)", 0x4037_0000, kb(448), scale_value=True)
Memory("External memory (Instruction bus)", 0x4200_0000, mb(32), scale_value=True)
Memory("Internal RTC SLOW Memory", 0x5000_0000, kb(8), scale_value=True)
Memory("Peripherals", 0x6000_0000, kb(836), scale_value=True)
Memory("Internal RTC FAST Memory", 0x600F_E000, kb(8), scale_value=True)

# subregions
Memory("Internal ROM 1 (Data bus)", 0x3FF0_0000, kb(128))
Memory("Internal SRAM 1 (Data bus)", 0x3FC8_8000, kb(416))
Memory("Internal SRAM 2 (Data bus)", 0x3FCF_0000, kb(64))
Memory("Internal ROM 0 (Instruction bus)", 0x4000_0000, kb(256))
Memory("Internal ROM 1 (Instruction bus)", 0x4004_0000, kb(128))
Memory("Internal SRAM 0 (Instruction bus)", 0x4037_0000, kb(32))
Memory("Internal SRAM 1 (Instruction bus)", 0x4037_8000, kb(416))

# placeholder subregions TODO remove
Memory("placeholder External memory (Data bus)", 0x3C00_0000, mb(32))
Memory("placeholder External memory (Instruction bus)", 0x4200_0000, mb(32))
Memory("placeholder Internal RTC SLOW Memory", 0x5000_0000, kb(8))
Memory("placeholder Internal RTC FAST Memory", 0x600F_E000, kb(8))

# rust application
RESERVE_ICACHE = 0x8000
VECTORS_SIZE = 0x400
Memory("vectors_seg", 0x4037_0000 + RESERVE_ICACHE, VECTORS_SIZE)
Memory("iram_seg", 0x4037_0000 + RESERVE_ICACHE + VECTORS_SIZE, kb(328) - VECTORS_SIZE - RESERVE_ICACHE)
dram_seg = Memory("dram_seg", 0x3FC8_8000, 345856)
Memory("irom_seg", 0x4200_0020, mb(4) - 0x20)
Memory("drom_seg", 0x3C00_0020, mb(4) - 0x20)
Memory("rtc_slow_seg", 0x5000_0000, kb(8))
Memory("rtc_fast_seg", 0x600F_E000, kb(8))
Memory("dram2_seg", dram_seg.address + dram_seg.size, 0x3fced710 - (dram_seg.address + dram_seg.size))

# idf bootloader
Memory("boot_dram_seg", 0x3FCE_2700, 0x00005000)
Memory("boot_iram_seg", 0x403C_8700, 0x00003000)
Memory("boot_iram_loader_seg", 0x403C_B700, 0x00007000)

Memory.fixup_root_value()

labels = [memory.text for memory in Memory.memories]
ids = [memory.name for memory in Memory.memories]
values = [memory.value for memory in Memory.memories]
parents = [memory.parent for memory in Memory.memories]
hovertext = [memory.hovertext for memory in Memory.memories]

icicle = go.Icicle(
    labels=labels,
    ids=ids,
    values=values,
    parents=parents,
    hovertext=hovertext,
    branchvalues="total",
    hoverinfo="label+text",
    sort=False
)
fig = go.Figure(icicle)

fig.show()
