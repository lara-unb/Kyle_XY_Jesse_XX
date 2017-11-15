Outline:
EMUL_DRV.SYS is COM port I/O ADDRESS REMAP driver which used for SB16C105xPCI semiconductor in MS-DOS.
If you want to use all function of SB16C105xPCI semiconductor properly, use COM_DRV.SYS driver. Then you can create 
maximum 8 COM ports together with default COM port.
Above case have disadvantage which cause you to correct Application program in order to fit interface to the 
interface of COM_DRV.SYS. However the functions are indeed very powerful and continent.

EMUL_DRV.SYS driver is useful when using the program which can't change I/O Address of default COM ports
In other word, EMUL_DRV.SYS driver provides forced change in I/O ADDRESS which have automatically interfaced 
to PCI/PCIe bus ROM BIOS.



How to Install:
Type as follow in CONFIG.SYS file.

DEVICE=EMUL_DRV.SYS /P? /C?

? means the number

P? means the number of COM ports inside of installed communication board at PCI slot in PC
In other word, when the port number of RS232 is 1ea, type P1
               when the port number of RS232 is 2ea, type P2
               when the port number of RS232 is 4ea, type P4

C? is used to determine I/O ADDRESS of COM port.
When P1,  Address of the C1 Com port is installed to address of Com1, 3F8h
          Address of the C2 Com port is installed to address of Com2, 2F8h
          Address of the C3 Com port is installed to address of Com3, 3E8h
          Address of the C4 Com port is installed to address of Com4, 2E8h

When P2, Address of the C1 in first Com port is installed to address, 3F0h and Address of second Com port is installed to address, 3F8h 
         Address of the C2 in first Com port is installed to address, 2F0h and Address of second Com port is installed to address, 2F8h
         Address of the C3 in first Com port is installed to address, 3E0h and Address of second Com port is installed to address, 3E8h
         Address of the C4 in first Com port is installed to address, 2E0h and Address of second Com port is installed to address, 2E8h

When P4, Address of the C1 in first Com port is installed to address, 3E0h and Address of second Com port is installed to address, 3E8h
         and third Com port is installed to address, 3F0h and Fourth Com port is installed to address,3F8h
         Address of the C2 in first Com port is installed to address, 2E0h and Address of second Com port is installed to address, 2E8h
         and third Com port is installed to address, 2F0h and Fourth Com port is installed to address,2F8h
         Address of the C3 in first Com port is installed to address, 3E0h and Address of second Com port is installed to address, 3E8h
         and third Com port is installed to address, 3F0h and Fourth Com port is installed to address,3F8h
         Address of the C4 in first Com port is installed to address, 2E0h and Address of second Com port is installed to address, 2E8h
         and third Com port is installed to address, 2F0h and Fourth Com port is installed to address,2F8h



Bellow chart is summary of above content.

+----+----+---------------+---------------+---------------+---------------+
| P? | C? | 1'st COM PORT | 2'nd COM PORT | 3'rd COM PORT | 4'th COM PORT |
+----+----+---------------+---------------+---------------+---------------+
|    | C1 |     3F8h      |       x       |       x       |       x       |
|    +----+---------------+---------------+---------------+---------------+
|    | C2 |     2F8h      |       x       |       x       |       x       |
| P1 +----+---------------+---------------+---------------+---------------+
|    | C3 |     3E8h      |       x       |       x       |       x       |
|    +----+---------------+---------------+---------------+---------------+
|    | C4 |     2E8h      |       x       |       x       |       x       |
+----+----+---------------+---------------+---------------+---------------+
|    | C1 |     3F0h      |     3F8h      |       x       |       x       |
|    +----+---------------+---------------+---------------+---------------+
|    | C2 |     2F0h      |     2F8h      |       x       |       x       |
| P2 +----+---------------+---------------+---------------+---------------+
|    | C3 |     3E0h      |     3E8h      |       x       |       x       |
|    +----+---------------+---------------+---------------+---------------+
|    | C4 |     2E0h      |     2E8h      |       x       |       x       |
+----+----+---------------+---------------+---------------+---------------+
|    | C1 |     3E0h      |     3E8h      |     3F0h      |     3F8h      |
|    +----+---------------+---------------+---------------+---------------+
|    | C2 |     2E0h      |     2E8h      |     3E0h      |     3E8h      |
| P4 +----+---------------+---------------+---------------+---------------+
|    | C3 |     3E0h      |     3E8h      |     3F0h      |     3F8h      |
|    +----+---------------+---------------+---------------+---------------+
|    | C4 |     2E0h      |     2E8h      |     3E0h      |     3E8h      |
+----+----+---------------+---------------+---------------+---------------+

Interrupts set attained automatically by system.

If progress of configuration finished properly, the content appears in monitor, which enables you to 
confirm the result directly.

Baud Rate setting is determined by Oscillator or Crystal in the board.
However, there is some problem because all preexisted board uses the standard 1.8432MHz.

If board uses 1.8432MHz Oscillator or Crystal, the board acts rightly in 1.8432MHz as preexisted program did.
However, If board uses 14.7456MHz Oscillator or Crystal, the board acts eight times faster than 
preexisted program setting.
So, In order to acting in 9600bps, you must set 1200bps in Application program.

EMUL_DRV.SYS is driver which used for SB16C105xPCI semiconductor.
If board is manufactured identically with the circuit of Systembase, the RS422,RS485 board provides 
automatic H/W Auto toggling. 
This function will be used in industrial spot conveniently and provides a reliable communication condition
