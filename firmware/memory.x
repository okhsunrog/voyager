MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  /* MBR sits at 0x0000-0x1000 (4KB) */
  /* Bootloader sits at 0xF4000-0xFF000 (44KB) */
  /* Application gets the space in between */
  FLASH : ORIGIN = 0x00001000, LENGTH = 0xF3000  /* 956KB */
  RAM : ORIGIN = 0x20000000, LENGTH = 256K
}
