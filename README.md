NJEmu 2.3.5 is based on NJEmu 2.3.1.


Modified for building on Linux 4.18.0-25-generic #26~18.04.1-Ubuntu SMP for PSP FAT(PSP1004)


``` Using built-in specs.
COLLECT_GCC=psp-gcc
COLLECT_LTO_WRAPPER=/home/hylo/local/pspdev/libexec/gcc/psp/9.3.0/lto-wrapper
Target: psp
Configured with: ../configure --prefix=/home/hylo/local/pspdev --target=psp --enable-languages=c,c++ --enable-lto --with-newlib --enable-cxx-flags=-G0
Thread model: single 
``` 


gcc version 9.3.0 (GCC)


devkit_version=0x06060010


----------------------------------------------

how to use:


O: OK

X: Cancel

Help: select

HOME(PS): emulator menu

On PSV or PPSSPP,HOME(PS) button can't use,you can delete "SystemButtons.prx"

SELECT+START: emulator menu

----------------------------------------------


CPS1PSP

Capcom System 1 Emulator



CPS2PSP

Capcom System 2 Emulator



MVSPSP

Neo-Geo MVS/AES Emulator



NCDZPSP

neogeo Emulator


---------------------------------------------

NJEmu 2.3.5 change log


romset use MAME0.152

font use simhei(CHARSET:GBK)

Japanese command list must use GBK charset,you can use notepad++ convert

fix png format bug

change help to select

change bios menu to R trigger

support multi language

added command hotkey

gamelist to 512

support cheat

added some romset(hack/bootleg)

---------------------------------------------

CPS1PSP

fix dipswitch

fix Mercs can't use player3

added some hack roms button3

fix Warriors of Fate (bootleg)

fix Huo Feng Huang(Chinese bootleg of Sangokushi II) sound

---------------------------------------------

CPS2PSP

---------------------------------------------

MVSPSP

fix dip menu

fix Jockey Grand Prix

fix King of Gladiator (KOF'97 bootleg)

support 128MB crom cache

support unibios 1.0-3.0 and neogit bios

support m1 decrypt

fix 000-lo.lo length

---------------------------------------------

NCDZPSP

fix 000-lo.lo length(fix sleep mode)

---------------------------------------------

Thanks to

cheat code: davex

hbl/multi language support: 173210

mame team

and NJ's emulator
