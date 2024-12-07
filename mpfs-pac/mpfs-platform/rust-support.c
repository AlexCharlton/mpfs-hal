void DefaultInitOnce(void) {}

void DefaultHart1Entry(void)
{
    while (1)
    {
        __asm volatile("wfi" ::: "memory");
    }
}

void DefaultHart2Entry(void)
{
    while (1)
    {
        __asm volatile("wfi" ::: "memory");
    }
}

void DefaultHart3Entry(void)
{
    while (1)
    {
        __asm volatile("wfi" ::: "memory");
    }
}

void DefaultHart4Entry(void)
{
    while (1)
    {
        __asm volatile("wfi" ::: "memory");
    }
}
