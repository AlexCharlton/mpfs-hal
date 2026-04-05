#include "verilated.h"
#include "Vdisplay_tb.h"

double sc_time_stamp() { return 0; }

int main(int argc, char **argv)
{
    const std::unique_ptr<VerilatedContext> contextp{new VerilatedContext};
    contextp->traceEverOn(true);
    contextp->commandArgs(argc, argv);
    const std::unique_ptr<Vdisplay_tb> tb{new Vdisplay_tb{contextp.get(), "display_tb"}};

    // Initialize simulation inputs
    tb->clk = 0;
    tb->reset = 0;
    tb->buffer0_ready = 0;
    tb->buffer1_ready = 0;
    tb->pclk = 0;

    while (!contextp->gotFinish())
    {
        contextp->timeInc(1);

        // Toggle clocks
        tb->clk = !tb->clk;

        // Reset sequence
        if (contextp->time() == 10)
            tb->reset = 1;
        if (contextp->time() == 4000000)
            tb->buffer0_ready = 1;
        if (contextp->time() == 6000000)
            tb->buffer0_ready = 0;
        if (contextp->time() == 12000000)
            tb->buffer1_ready = 1;

        // Evaluate model
        tb->eval();
    }

    // Cleanup
    tb->final();
    contextp->statsPrintSummary();

    return 0;
}