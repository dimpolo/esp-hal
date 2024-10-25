/* this is the RAM0 region and excludes RAM0 and RAM2 regions meant for ICache and DCache */
dram_start = 0x3FC88000;
iram_start = 0x40378000;
ram_size = 416k;

MEMORY
{
  iram_seg (RWX) : ORIGIN = iram_start, LENGTH = ram_size
  dram_seg (RW) : ORIGIN = dram_start, LENGTH = ram_size
}
