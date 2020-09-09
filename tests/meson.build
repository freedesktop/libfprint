envs = environment()
# Enable debug messages and abort on warnings
envs.set('G_DEBUG', 'fatal-warnings')
envs.set('G_MESSAGES_DEBUG', 'all')

# Setup paths
envs.set('MESON_SOURCE_ROOT', meson.source_root())
envs.prepend('LD_LIBRARY_PATH', join_paths(meson.build_root(), 'libfprint'))

# Set FP_DEVICE_EMULATION so that drivers can adapt (e.g. to use fixed
# random numbers rather than proper ones)
envs.set('FP_DEVICE_EMULATION', '1')

# Set a colon-separated list of native drivers we enable in tests
envs.set('FP_DRIVERS_WHITELIST', 'virtual_image')

envs.set('NO_AT_BRIDGE', '1')

drivers_tests = [
    'elan',
    'synaptics',
    'vfs0050',
    'vfs5011',
    'goodixmoc',
]

if get_option('introspection')
    envs.prepend('GI_TYPELIB_PATH', join_paths(meson.build_root(), 'libfprint'))

    if 'virtual_image' in drivers
        python3 = find_program('python3')
        unittest_inspector = find_program('unittest_inspector.py')
        base_args = files('virtual-image.py')
        suite = []

        r = run_command(unittest_inspector, files('virtual-image.py'))
        unit_tests = r.stdout().strip().split('\n')

        if r.returncode() == 0 and unit_tests.length() > 0
            suite += 'virtual-image'
        else
            unit_tests = ['virtual-image']
        endif

        foreach ut: unit_tests
            ut_suite = suite
            ut_args = base_args
            if unit_tests.length() > 1
                ut_args += ut
                ut_suite += ut.split('.')[0]
            endif
            test(ut,
                python3,
                args: ut_args,
                suite: ut_suite,
                depends: libfprint_typelib,
                env: envs,
            )
        endforeach
    else
        test('virtual-image',
            find_program('sh'),
            args: ['-c', 'exit 77']
        )
    endif

    foreach driver_test: drivers_tests
        driver_envs = envs
        driver_envs.set('FP_DRIVERS_WHITELIST', driver_test)

        if driver_test in drivers and gusb_dep.version().version_compare('>= 0.3.0')
            test(driver_test,
                find_program('umockdev-test.py'),
                args: join_paths(meson.current_source_dir(), driver_test),
                env: driver_envs,
                suite: ['drivers'],
                timeout: 10,
                depends: libfprint_typelib,
            )
        else
            test(driver_test,
                find_program('sh'),
                args: ['-c', 'exit 77']
            )
        endif
    endforeach
else
    warning('Skipping all driver tests as introspection bindings are missing')
    test('virtual-image',
        find_program('sh'),
        args: ['-c', 'exit 77']
    )

    foreach driver_test: drivers_tests
        test(driver_test,
            find_program('sh'),
            args: ['-c', 'exit 77']
        )
    endforeach
endif

test_utils = static_library('fprint-test-utils',
    sources: [
        'test-utils.c',
        'test-device-fake.c',
    ],
    dependencies: libfprint_private_dep,
    install: false)

unit_tests = [
    'fpi-device',
    'fpi-ssm',
    'fpi-assembling',
]

if 'virtual_image' in drivers
    unit_tests += [
        'fp-context',
        'fp-device',
    ]
endif

unit_tests_deps = { 'fpi-assembling' : [cairo_dep] }

test_config = configuration_data()
test_config.set_quoted('SOURCE_ROOT', meson.source_root())
test_config_h = configure_file(output: 'test-config.h', configuration: test_config)

foreach test_name: unit_tests
    if unit_tests_deps.has_key(test_name)
        missing_deps = false
        foreach dep: unit_tests_deps[test_name]
            if not dep.found()
                missing_deps = true
                break
            endif
        endforeach

        if missing_deps
            # Create a dummy test that always skips instead
            warning('Test @0@ cannot be compiled due to missing dependencies'.format(test_name))
            test(test_name,
                find_program('sh'),
                suite: ['unit-tests'],
                args: ['-c', 'exit 77'],
            )
            continue
        endif
        extra_deps = unit_tests_deps[test_name]
    else
        extra_deps = []
    endif

    basename = 'test-' + test_name
    test_exe = executable(basename,
        sources: [basename + '.c', test_config_h],
        dependencies: [ libfprint_private_dep ] + extra_deps,
        c_args: common_cflags,
        link_with: test_utils,
    )
    test(test_name,
        find_program('test-runner.sh'),
        suite: ['unit-tests'],
        args: [test_exe],
        env: envs,
    )
endforeach

gdb = find_program('gdb', required: false)
if gdb.found()
    add_test_setup('gdb',
        timeout_multiplier: 1000,
        env: [
            'LIBFPRINT_TEST_WRAPPER=@0@ --args'.format(
                gdb.path())
        ])
endif

valgrind = find_program('valgrind', required: false)
if valgrind.found()
    glib_share = glib_dep.get_pkgconfig_variable('prefix') / 'share' / glib_dep.name()
    glib_suppressions = glib_share + '/valgrind/glib.supp'
    python_suppressions = '@0@/@1@'.format(meson.source_root(),
        files('valgrind-python.supp')[0])
    add_test_setup('valgrind',
        timeout_multiplier: 10,
        env: [
            'G_SLICE=always-malloc',
            ('LIBFPRINT_TEST_WRAPPER=@0@ --tool=memcheck --leak-check=full ' +
             '--suppressions=@1@ --suppressions=@2@').format(
                valgrind.path(), glib_suppressions, python_suppressions)
        ])
endif
