libfprint_sources = [
    'fp-context.c',
    'fp-device.c',
    'fp-image.c',
    'fp-print.c',
    'fp-image-device.c',
]

libfprint_private_sources = [
    'fpi-assembling.c',
    'fpi-byte-reader.c',
    'fpi-byte-writer.c',
    'fpi-device.c',
    'fpi-image-device.c',
    'fpi-image.c',
    'fpi-print.c',
    'fpi-ssm.c',
    'fpi-usb-transfer.c',
]

libfprint_public_headers = [
    'fp-context.h',
    'fp-device.h',
    'fp-image-device.h',
    'fp-image.h',
    'fp-print.h',
]

libfprint_private_headers = [
    'fpi-assembling.h',
    'fpi-byte-reader.h',
    'fpi-byte-utils.h',
    'fpi-byte-writer.h',
    'fpi-compat.h',
    'fpi-context.h',
    'fpi-device.h',
    'fpi-image-device.h',
    'fpi-image.h',
    'fpi-log.h',
    'fpi-minutiae.h',
    'fpi-print.h',
    'fpi-usb-transfer.h',
    'fpi-ssm.h',
]

nbis_sources = [
    'nbis/bozorth3/bozorth3.c',
    'nbis/bozorth3/bz_alloc.c',
    'nbis/bozorth3/bz_drvrs.c',
    'nbis/bozorth3/bz_gbls.c',
    'nbis/bozorth3/bz_io.c',
    'nbis/bozorth3/bz_sort.c',
    'nbis/mindtct/binar.c',
    'nbis/mindtct/block.c',
    'nbis/mindtct/chaincod.c',
    'nbis/mindtct/contour.c',
    'nbis/mindtct/detect.c',
    'nbis/mindtct/dft.c',
    'nbis/mindtct/free.c',
    'nbis/mindtct/getmin.c',
    'nbis/mindtct/globals.c',
    'nbis/mindtct/imgutil.c',
    'nbis/mindtct/init.c',
    'nbis/mindtct/line.c',
    'nbis/mindtct/link.c',
    'nbis/mindtct/log.c',
    'nbis/mindtct/loop.c',
    'nbis/mindtct/maps.c',
    'nbis/mindtct/matchpat.c',
    'nbis/mindtct/minutia.c',
    'nbis/mindtct/morph.c',
    'nbis/mindtct/quality.c',
    'nbis/mindtct/remove.c',
    'nbis/mindtct/ridges.c',
    'nbis/mindtct/shape.c',
    'nbis/mindtct/sort.c',
    'nbis/mindtct/util.c',
    'nbis/mindtct/xytreps.c',
]

aeslib = false
aesx660 = false
aes3k = false
drivers_sources = []
drivers_cflags = []
foreach driver: drivers
    if driver == 'upekts'
        drivers_sources += [ 'drivers/upekts.c', 'drivers/upek_proto.c' ]
    endif
    if driver == 'upektc'
        drivers_sources += [ 'drivers/upektc.c' ]
    endif
    if driver == 'upeksonly'
        drivers_sources += [ 'drivers/upeksonly.c' ]
    endif
    if driver == 'uru4000'
        drivers_sources += [ 'drivers/uru4000.c' ]
    endif
    if driver == 'aes1610'
        drivers_sources += [ 'drivers/aes1610.c' ]
        aeslib = true
    endif
    if driver == 'aes1660'
        drivers_sources += [ 'drivers/aes1660.c' ]
        aeslib = true
        aesx660 = true
    endif
    if driver == 'aes2501'
        drivers_sources += [ 'drivers/aes2501.c' ]
        aeslib = true
    endif
    if driver == 'aes2550'
        drivers_sources += [ 'drivers/aes2550.c' ]
        aeslib = true
    endif
    if driver == 'aes2660'
        drivers_sources += [ 'drivers/aes2660.c' ]
        aeslib = true
        aesx660 = true
    endif
    if driver == 'aes3500'
        drivers_sources += [ 'drivers/aes3500.c' ]
        aeslib = true
        aes3k = true
    endif
    if driver == 'aes4000'
        drivers_sources += [ 'drivers/aes4000.c' ]
        aeslib = true
        aes3k = true
    endif
    if driver == 'vcom5s'
        drivers_sources += [ 'drivers/vcom5s.c' ]
    endif
    if driver == 'vfs101'
        drivers_sources += [ 'drivers/vfs101.c' ]
    endif
    if driver == 'vfs301'
        drivers_sources += [ 'drivers/vfs301.c', 'drivers/vfs301_proto.c' ]
    endif
    if driver == 'vfs5011'
        drivers_sources += [ 'drivers/vfs5011.c' ]
    endif
    if driver == 'vfs7552'
        drivers_sources += [ 'drivers/vfs7552.c' ]
    endif
    if driver == 'upektc_img'
        drivers_sources += [ 'drivers/upektc_img.c',  'drivers/upek_proto.c' ]
    endif
    if driver == 'etes603'
        drivers_sources += [ 'drivers/etes603.c' ]
    endif
    if driver == 'vfs0050'
        drivers_sources += [ 'drivers/vfs0050.c' ]
    endif
    if driver == 'elan'
        drivers_sources += [ 'drivers/elan.c' ]
    endif
    if driver == 'virtual_image'
        drivers_sources += [ 'drivers/virtual-image.c' ]
    endif
    if driver == 'synaptics'
    drivers_sources += [
        'drivers/synaptics/synaptics.c',
        'drivers/synaptics/bmkt_message.c',
    ]
    endif
    if driver == 'goodixmoc'
    drivers_sources += [
        'drivers/goodixmoc/goodix.c',
        'drivers/goodixmoc/goodix_proto.c',
    ]
    endif
endforeach

if aeslib
    drivers_sources += [ 'drivers/aeslib.c' ]
endif
if aesx660
    drivers_sources += ['drivers/aesx660.c' ]
endif
if aes3k
    drivers_sources += ['drivers/aes3k.c' ]
endif

other_sources = []

fp_enums = gnome.mkenums_simple('fp-enums',
    sources: libfprint_public_headers,
    install_header: true,
    install_dir: get_option('includedir') / versioned_libname,
)
fp_enums_h = fp_enums[1]

fpi_enums = gnome.mkenums_simple('fpi-enums',
    sources: libfprint_private_headers,
    install_header: false,
)
fpi_enums_h = fpi_enums[1]

enums_dep = declare_dependency(
    sources: [ fp_enums_h, fpi_enums_h ]
)

drivers_sources += configure_file(input: 'empty_file',
    output: 'fpi-drivers.c',
    capture: true,
    command: [
        'echo',
        '\n'.join(drivers_type_list + [] + drivers_type_func)
    ])

deps = [
    enums_dep,
    gio_dep,
    glib_dep,
    gobject_dep,
    gusb_dep,
    imaging_dep,
    mathlib_dep,
    nss_dep,
]

# These are empty and only exist so that the include directories are created
# in the build tree. This silences a build time warning.
subdir('nbis/include')
subdir('nbis/libfprint-include')
deps += declare_dependency(include_directories: [
    root_inc,
    include_directories('nbis/include'),
    include_directories('nbis/libfprint-include'),
])

libnbis = static_library('nbis',
    nbis_sources,
    dependencies: deps,
    c_args: cc.get_supported_arguments([
        '-Wno-error=redundant-decls',
        '-Wno-redundant-decls',
        '-Wno-discarded-qualifiers',
    ]),
    install: false)

libfprint_private = static_library('fprint-private',
    sources: [
        fpi_enums,
        libfprint_private_sources,
    ],
    dependencies: deps,
    link_with: libnbis,
    install: false)

libfprint_drivers = static_library('fprint-drivers',
    sources: drivers_sources,
    c_args: drivers_cflags,
    dependencies: deps,
    link_with: libfprint_private,
    install: false)

mapfile = files('libfprint.ver')
vflag = '-Wl,--version-script,@0@/@1@'.format(meson.source_root(), mapfile[0])

libfprint = library(versioned_libname.split('lib')[1],
    sources: [
        fp_enums,
        libfprint_sources,
        other_sources,
    ],
    soversion: soversion,
    version: libversion,
    link_args : vflag,
    link_depends : mapfile,
    link_with: [libfprint_drivers, libfprint_private],
    dependencies: deps,
    install: true)

libfprint_dep = declare_dependency(link_with: libfprint,
    include_directories: root_inc,
    dependencies: [
        enums_dep,
        gio_dep,
        glib_dep,
        gobject_dep,
        gusb_dep,
    ])

install_headers(['fprint.h'] + libfprint_public_headers,
    subdir: versioned_libname
)

libfprint_private_dep = declare_dependency(
    include_directories: include_directories('.'),
    link_with: libfprint_private,
    dependencies: [
        deps,
        libfprint_dep,
    ]
)

udev_rules = executable('fprint-list-udev-rules',
    'fprint-list-udev-rules.c',
    dependencies: libfprint_private_dep,
    link_with: libfprint_drivers,
    install: false)

if get_option('udev_rules')
    custom_target('udev-rules',
        output: '60-@0@-autosuspend.rules'.format(versioned_libname),
        capture: true,
        command: [ udev_rules ],
        install: true,
        install_dir: udev_rules_dir)
endif

supported_devices = executable('fprint-list-supported-devices',
    'fprint-list-supported-devices.c',
    dependencies: libfprint_private_dep,
    link_with: libfprint_drivers,
    install: false)


if get_option('introspection')
    # We do *not* include the private header here
    libfprint_girtarget = gnome.generate_gir(libfprint,
        sources : fp_enums + [
            libfprint_public_headers,
            libfprint_sources,
        ],
        nsversion : '@0@.0'.format(soversion),
        namespace : 'FPrint',
        symbol_prefix : 'fp_',
        identifier_prefix : 'Fp',
        export_packages : 'fprint',
        extra_args : [
            '--c-include=fprint.h',
        ],
        link_with : libfprint,
        dependencies : [
            gio_dep,
            gobject_dep,
            gusb_dep,
        ],
        includes : [
            'Gio-2.0',
            'GObject-2.0',
            'GUsb-1.0',
        ],
        install : true)
    libfprint_gir = libfprint_girtarget[0]
    libfprint_typelib = libfprint_girtarget[1]
endif
