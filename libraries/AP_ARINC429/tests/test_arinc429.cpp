#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>

/*
 * The SITL gtest repeats the host suite. The codec sources are compiled
 * into this test. Vehicle firmware includes them only when
 * AP_ARINC429_ENABLED is 1, which SITL leaves at 0, so the symbols
 * are not duplicated in the usual SITL link.
 */
extern "C" {
#include "../src/arinc429_catalog.c"
#include "../src/arinc429_word.c"
#include "../src/arinc429_sched.c"
#include "../src/arinc429_link.c"
#include "../src/arinc429_transport.c"
}

#define A429_NO_MAIN 1
#include "../host/test_arinc429.c"

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

TEST(ARINC429, IcdAndSchedule)
{
    EXPECT_EQ(0, a429_host_tests());
}

AP_GTEST_PANIC()
AP_GTEST_MAIN()
