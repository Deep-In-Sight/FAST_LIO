#include <colormap.hpp>
#include <laserMapping.hpp>
#include <libunwind.h>

std::atomic<bool> stack_trace_printed{false}; // Prevent re-entry
void print_stack_trace()
{
    unw_cursor_t cursor;
    unw_context_t uc;

    unw_getcontext(&uc);
    unw_init_local(&cursor, &uc);

    while (unw_step(&cursor) > 0)
    {
        unw_word_t ip, sp, offset;
        char funcName[256];

        unw_get_reg(&cursor, UNW_REG_IP, &ip);
        unw_get_reg(&cursor, UNW_REG_SP, &sp);
        unw_get_proc_name(&cursor, funcName, sizeof(funcName), &offset);

        printf("ip = %lx, sp = %lx, function = %s + 0x%lx\n", (long)ip, (long)sp, funcName, (long)offset);
    }
}

void SigHandle(int sig)
{
    if (stack_trace_printed.exchange(true))
    {
        // Avoid handling the signal again
        return;
    }
    printf("Caught signal %d\n", sig);
    if (sig == SIGSEGV || sig == SIGABRT)
    {
        print_stack_trace();
    }
    rclcpp::shutdown();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    signal(SIGINT, SigHandle);
    signal(SIGSEGV, SigHandle);
    signal(SIGABRT, SigHandle);

    auto mappingNode = std::make_shared<LaserMappingNode>();
    auto colorizeNode = ColormapNode::getInstance();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(mappingNode);
    executor.add_node(colorizeNode);

    executor.spin();

    if (rclcpp::ok())
        rclcpp::shutdown();

    saveEverything();
    return 0;
}
