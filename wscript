from __future__ import print_function
import os
from pathlib import Path

rtems_version = "7"
try:
    import rtems_waf.rtems as rtems
except Exception:
    import sys
    print('error: no rtems_waf git submodule')
    sys.exit(1)

# Nombre de la app tomado automáticamente del directorio donde está este fichero
APP_NAME = Path(__name__).resolve().parent.name

def init(ctx):
    rtems.init(ctx, version = rtems_version, long_commands = True)

def bsp_configure(conf, arch_bsp):
    pass

def options(opt):
    rtems.options(opt)

def configure(conf):
    rtems.configure(conf, bsp_configure = bsp_configure)

def build(bld):
    rtems.build(bld)
    # target usa el nombre de la carpeta como APP_NAME
    bld(features='c cprogram',
        target=f'{APP_NAME}.exe',
        cflags='-g -O2',
        source=bld.path.ant_glob('*.c'))
