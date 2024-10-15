# esp-bootloader

## Understanding Memory:

The chip has the following internal memory types:

| Memory Type | Description                                                                          |
|-------------|--------------------------------------------------------------------------------------|
| ROM         | Read-Only Memory, contains first stage bootloader and some low level system software |
| SRAM        | Static RAM                                                                           |
| RTC Memory  | Memory retained during sleep modes                                                   |
| eFuse       | Parameters, once an eFuse bit is programmed to 1, it can never be reverted to 0      |

Part of the SRAM is used as a cache for external memory access and cannot be used as general purpose memory.

The chip doesn't have builtin flash memory, code and data are stored in external flash memory.
The chip and the external flash memory are connected via SPI.

The external flash memory is mapped to virtual memory addresses in the chip's address space.
Accessing this virtual memory will access the DCache (Data Cache) or ICache (Instruction Cache).
When a cache miss occurs, the cache controller will initiate a request to the external memory.
 
## Boot process:
* first stage bootloader in ROM runs
  * determine boot mode (assuming normal boot mode in next steps)
  * enable RTC watchdog
  * configure SPI flash based on eFuse values, and attempt to load the code from flash at offset
    * `0x0000` (esp32s3, esp32c2, esp32c3, esp32c6, esp32h2)
    * `0x1000` (esp32, esp32s2) or 
    * `0x2000` (esp32c5, esp32p4)
* second stage bootloader
  * find where application is stored in flash (e.g. by reading a partition table)
  * for the selected partition, read the binary image from flash one segment at a time
    * for segments with load addresses in internal RAM, the contents are copied from flash to the load address
    * for segments which have load addresses in flash, the flash MMU is configured to provide the correct mapping from the flash to the load address.
  * jump to the entry point of the application

The binary image includes a header with information like entry point of the application and external flash configuration.

## TODO 
SRAM -> Cache: is this done by default, the rom bootloader or the user bootloader?
What external flash chips are supported?
Does the flash config in the second stage bootloader header effect the first stage bootloader? 
