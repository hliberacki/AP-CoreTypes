subdir('test_runner')

srcs = [
    'error_domain_test.cpp',
    'core_error_domain_test.cpp',
    'array_test.cpp',
    'map_test.cpp',
    'utility_test.cpp',
    'vector_test.cpp',
    'variant_test.cpp',
    'byte_test.cpp',
    'result_test.cpp'
]

# Add `include` to include directories
incdir = include_directories('../include')

tests_exec = executable(
    'tests',
    srcs,
    dependencies: [
        test_runner_dep,
    ],
    include_directories : incdir,
    link_with: ap_coretypes_lib
)

test('tests', tests_exec)
