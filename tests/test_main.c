/*---------------------------------------------------------------------------------
 * test_main.c -- host test binary entry point.
 *
 * Deliberately tiny: all the actual checks live in test_physics.c. This
 * file's only job is to call the runner and turn its failure count into a
 * process exit code, so `tools`/CI/a human can tell pass from fail from the
 * exit code alone (0 == every check passed), per this project's build
 * verification discipline -- "prove it, don't reason about it."
 *---------------------------------------------------------------------------------*/
#include <stdio.h>

int run_physics_tests(void);

int main(void) {
    int failures = run_physics_tests();
    if (failures != 0) {
        fprintf(stderr, "test_main: %d check(s) failed\n", failures);
        return 1;
    }
    fprintf(stdout, "test_main: all checks passed\n");
    return 0;
}
