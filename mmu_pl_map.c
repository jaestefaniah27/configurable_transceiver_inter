/* mmu_pl_map_debug.c */
#include <bsp/aarch64-mmu.h>
#include <rtems.h>
#include <stdio.h>
#include <stdint.h>

#ifndef AARCH64_MMU_DEVICE_nGnRE
  #define AARCH64_MMU_DEVICE_nGnRE AARCH64_MMU_DEVICE
#endif

void mmu_map_pl_axi_early(void)
{
  aarch64_mmu_config_entry e;
  e.begin = 0xA0000000u;
  e.end   = 0xA0000000u + 0x01000000u - 1u; /* 16 MiB */
  e.flags = AARCH64_MMU_DEVICE_nGnRE;

  printf("MMU_MAP: mmu_map_pl_axi_early() called -> begin=0x%08x end=0x%08x\n",
         (unsigned)e.begin, (unsigned)e.end);
  fflush(stdout);

  int rc = aarch64_mmu_set_translation_table_entries(&aarch64_mmu_instance, &e);
  printf("MMU_MAP: aarch64_mmu_set_translation_table_entries rc=%d\n", rc);
  fflush(stdout);

  __asm__ volatile("dsb sy; isb" ::: "memory");
  rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(1));
  printf("MMU_MAP: return from mmu_map_pl_axi_early()\n");
  fflush(stdout);
}
