import math

import plotly.graph_objects as go

MB = 1024 * 1024
KB = 1024
VALUE_SCALING_POWER = 0.15


class Memory:
    memories = []

    def __init__(self, name, address, size, depth, dummy=False, dummy_parent=False):
        self.name = name
        self.address = address
        self.size = size
        self.address_end = address + size - 1
        self.depth = depth
        self.dummy = dummy
        self.parent = self.get_parent(name, address, size, depth - 1)
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
    def get_parent(name, address, size, parent_depth):
        if parent_depth < 0:
            return None
        for memory in Memory.memories:
            if parent_depth == memory.depth and memory.address <= address <= memory.address_end:
                return memory
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

# rust application
RESERVE_ICACHE = 0x8000
VECTORS_SIZE = 0x400
Memory("vectors_seg", 0x4037_0000 + RESERVE_ICACHE, VECTORS_SIZE, depth=3)
Memory("iram_seg", 0x4037_0000 + RESERVE_ICACHE + VECTORS_SIZE, kb(328) - VECTORS_SIZE - RESERVE_ICACHE, depth=3)
dram_seg = Memory("dram_seg", 0x3FC8_8000, 345856, depth=3)
Memory("irom_seg", 0x4200_0020, mb(4) - 0x20, depth=3)
Memory("drom_seg", 0x3C00_0020, mb(4) - 0x20, depth=3)
Memory("rtc_slow_seg", 0x5000_0000, kb(8), depth=3)
Memory("rtc_fast_seg", 0x600F_E000, kb(8), depth=3)
Memory("dram2_seg", dram_seg.address + dram_seg.size, 0x3fced710 - (dram_seg.address + dram_seg.size), depth=3)

# idf bootloader
Memory("boot_dram_seg", 0x3FCE_2700, 0x00005000, depth=4)
Memory("boot_iram_seg", 0x403C_8700, 0x00003000, depth=4)
Memory("boot_iram_loader_seg", 0x403C_B700, 0x00007000, depth=4)

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
