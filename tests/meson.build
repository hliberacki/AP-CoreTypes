subdir('test_runner')

srcs = [
    'error_domain_test.cpp',
    'array_test.cpp'
]

# Add `include` to include directories
incdir = include_directories('../include')

tests_exec = executable(
    'tests',
    srcs,
    dependencies: [
        test_runner_dep,
    ],
    include_directories : incdir
)

test('tests', tests_exec)
