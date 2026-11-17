#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>

#include "../AP_ARINC429_config.h"

/*
 * The SITL gtest repeats the host suite. The codec sources are compiled
 * into this test when the vehicle leaves AP_ARINC429_ENABLED at 0.
 * A SITL build that turns the feature on already links those objects,
 * so including them again would duplicate the symbols.
 */
#if !AP_ARINC429_ENABLED
extern "C" {
#include "../src/arinc429_catalog.c"
#include "../src/arinc429_word.c"
#include "../src/arinc429_sched.c"
#include "../src/arinc429_link.c"
#include "../src/arinc429_frame.c"
#include "../src/arinc429_tcp.c"
#include "../src/arinc429_transport.c"
#include "../src/arinc429_mode.c"
}
#endif
/* The host suite compares exact engineering values. */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
#include "../host/test_nt1m.c"

#define A429_NO_MAIN 1
#include "../host/test_arinc429.c"
#pragma GCC diagnostic pop

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

TEST(ARINC429, IcdAndSchedule)
{
    EXPECT_EQ(0, a429_host_tests());
    EXPECT_EQ(0, a429_nt1m_tests());
}

AP_GTEST_PANIC()
AP_GTEST_MAIN()
