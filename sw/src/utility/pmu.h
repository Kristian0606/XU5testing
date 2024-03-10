//#include "xpm_counter.h"
//#include "xtmrctr.h" --wrong counter i think
#include "xtime_l.h"
#include "xparameters.h"
#define F_CPU               XPAR_PSU_CORTEXA53_0_CPU_CLK_FREQ_HZ


static inline u32 pmu_get_counter() {
    u64 counter;
    XTime_GetTime(&counter);

    return (u32) counter;
}

static inline u8 pmu_get_utilization() {
    void XTime_StartTimer(void);
    u32 active_cycles = pmu_get_counter();

    return (u8)(100 * (active_cycles >> 16) / (F_CPU >> 16));
}
