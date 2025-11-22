#include "altera_avalon_sierra_ker.h"
#include <altera_avalon_sierra_io.h>
#include <altera_avalon_sierra_regs.h>
#include <altera_avalon_sierra_name.h>
#include <altera_avalon_timer_regs.h>
#include <altera_avalon_pio_regs.h>
#include "system.h"
#include "stdio.h"
#include "io.h"
#include <DE10_Lite_VGA_Driver.h>
#include <DE10_Lite_Arduino_Driver.h>
#include <alt_types.h>
#include <stdbool.h>

/*
 * ---------------------------------------------------------------------
 *  RTK MULTITASKING PROJECT
 * ---------------------------------------------------------------------
 *  This system runs multiple periodic tasks using the Sierra RTK kernel:
 *   - Idle task (background)
 *   - Timer task (1Hz counter)
 *   - Accelerometer sampling task
 *   - Accelerometer filtering (average of last 10 samples)
 *   - Plotting task for graphing Z-axis acceleration
 *
 *  Each task runs with its own stack and priority.
 *  The accelerometer values are shared using a RTK semaphore.
 *  All graphical output is rendered on the DE10-Lite VGA framebuffer.
 * ---------------------------------------------------------------------
 */

#define STACK_SIZE 800          // Stack size for each task
#define SEM_SHARED_DATA 1       // Semaphore ID used for shared accelerometer data

// Task identifiers
#define IDLE            0
#define TASK_ACC        2
#define TASK_ACC_FILTER 3
#define TASK_TIMER      1
#define TASK_PLOT       4

// Structure for accelerometer samples
typedef struct position {
    int16_t x;
    int16_t y;
    int16_t z;
} position_t;

// Global shared accelerometer data (protected by semaphore)
position_t global_acc_data;

// Task stacks
char idle_stack[STACK_SIZE];
char acc_stack[STACK_SIZE];
char acc_filter_stack[STACK_SIZE];
char timer_stack[STACK_SIZE];
char plot_stack[STACK_SIZE];

// Forward declaration
void clear_screen_range(size_t start_x, size_t start_y, size_t end_x, size_t end_y);

/* ================================================================
 *                         IDLE TASK
 * ================================================================ */
void idle_code(void)
{
    int i = 0;
    printf("Idle task started\n");

    while (1)
    {
        for (i = 0; i < 500000; i++);  // Cheap delay
        printf(".\n");
    }
}

/* ================================================================
 *                         TIMER TASK
 *  Counts seconds and displays the running time on the screen.
 * ================================================================ */
void timer_task_code(void)
{
    task_periodic_start_union deadline;
    init_period_time(50);  // 1 second (50 ticks @ 20ms each)
    const char task_name[] = "Timer";

    unsigned int time = 0;

    while (1)
    {
        deadline = wait_for_next_period();

        if (deadline.periodic_start_integer & 0x1)
            printf("Deadline miss: TIMER task\n");

        time++; // Count seconds

        // Display on screen
        tty_print(70, 140, task_name, Col_White, Col_Black);
        int_print(70, 165, time, 5, Col_White, Col_Black);
    }
}

/* ================================================================
 *                    ACCELEROMETER READING TASK
 *  Reads new accelerometer samples every second and displays them.
 * ================================================================ */
void task_acc_code()
{
    task_periodic_start_union deadline;
    init_period_time(50); // 1 second

    position_t local_acc_data;

    while (1)
    {
        deadline = wait_for_next_period();

        if (deadline.periodic_start_integer & 0x1)
            printf("Deadline miss: ACC task\n");

        // Read hardware accelerometer
        accelerometer_receive(&local_acc_data.x,
                              &local_acc_data.y,
                              &local_acc_data.z);

        // Update global shared data
        sem_take(SEM_SHARED_DATA);
        global_acc_data = local_acc_data;
        sem_release(SEM_SHARED_DATA);

        // Display values
        tty_print(60, 25, "task_Acc", Col_White, Col_Black);

        tty_print(60, 40, "X", Col_White, Col_Black);
        int_print(70, 40, local_acc_data.x, 3, Col_White, Col_Black);

        tty_print(60, 50, "Y", Col_White, Col_Black);
        int_print(70, 50, local_acc_data.y, 3, Col_White, Col_Black);

        tty_print(60, 60, "Z", Col_White, Col_Black);
        int_print(70, 60, local_acc_data.z, 3, Col_White, Col_Black);
    }
}

/* ================================================================
 *                ACCELEROMETER FILTER TASK
 *  Computes the average of the last 10 accelerometer samples.
 * ================================================================ */
void task_acc_filter_code()
{
    task_periodic_start_union deadline;
    init_period_time(50);  // 1 second

    position_t acc_array[10];
    int counter = 0;
    bool sampled_ten_times = false;

    int avg_x = 0;
    int avg_y = 0;
    int avg_z = 0;

    while (1)
    {
        deadline = wait_for_next_period();

        if (deadline.periodic_start_integer & 0x1)
            printf("Deadline miss: ACC FILTER task\n");

        // Read shared data
        sem_take(SEM_SHARED_DATA);
        acc_array[counter] = global_acc_data;
        sem_release(SEM_SHARED_DATA);

        tty_print(200, 35, "task_acc_filter", Col_White, Col_Black);

        if (sampled_ten_times)
        {
            // Reset averages
            avg_x = avg_y = avg_z = 0;

            for (size_t i = 0; i < 10; i++)
            {
                avg_x += acc_array[i].x;
                avg_y += acc_array[i].y;
                avg_z += acc_array[i].z;
            }

            avg_x /= 10;
            avg_y /= 10;
            avg_z /= 10;

            // Display filtered output
            tty_print(220, 50, "X", Col_White, Col_Black);
            int_print(230, 50, avg_x, 3, Col_White, Col_Black);

            tty_print(220, 60, "Y", Col_White, Col_Black);
            int_print(230, 60, avg_y, 3, Col_White, Col_Black);

            tty_print(220, 70, "Z", Col_White, Col_Black);
            int_print(230, 70, avg_z, 3, Col_White, Col_Black);
        }
        else if (counter == 9)
        {
            sampled_ten_times = true;
            clear_screen_range(170, 0, 319, 110);
        }
        else
        {
            tty_print(200, 65, "sampling...", Col_White, Col_Black);
        }

        counter = (counter + 1) % 10;
    }
}

/* ================================================================
 *                       PLOTTING TASK
 *  Plots Z-axis acceleration as a scrolling graph.
 * ================================================================ */
void task_plot_code()
{
    task_periodic_start_union deadline;
    init_period_time(50);  // 1 second

    position_t local_pos;
    int counter = 0;

    while (1)
    {
        deadline = wait_for_next_period();

        if (deadline.periodic_start_integer & 0x1)
            printf("Deadline miss: PLOT task\n");

        // Clear part of graph area when starting a new row
        if (counter == 0)
            clear_screen_range(170, 125, 319, 239);

        tty_print(200, 130, "task_acc_filter", Col_White, Col_Black);

        // Get accelerometer sample
        sem_take(SEM_SHARED_DATA);
        local_pos = global_acc_data;
        sem_release(SEM_SHARED_DATA);

        // Draw X-axis baseline
        tty_print(190, 180, "0", Col_White, Col_Black);
        draw_hline(200, 180, 60, Col_Cyan);

        // Plot Z value
        draw_filled_circle(
            205 + (5 * counter),
            180 + ((local_pos.z / 8) * (-1)),
            1,
            Col_Green);

        counter = (counter + 1) % 5;
    }
}

/* ================================================================
 *                           MAIN
 * ================================================================ */
int main()
{
    // Initial welcome screen
    clear_screen(Col_Black);
    tty_print(150, 20, "Menyar Hees", Col_Magenta, Col_Black);
    tty_print(140, 120, "press any button", Col_Red, Col_Black);

    int button = 3;

    // Wait for pushbutton input
    while (button == 3)
    {
        button = 0x3 & IORD_ALTERA_AVALON_PIO_DATA(PIO_BUTTONS_IN_BASE);
    }

    clear_screen(Col_Black);

    // Sierra initialization
    Sierra_Initiation_HW_and_SW();

    printf("Sierra HW version = %d\n", sierra_HW_version());
    printf("Sierra SW driver version = %d\n", sierra_SW_driver_version());

    // Set RTK time base: 20ms tick (50Hz)
    set_timebase(1000);

    // Ensure accelerometer is ready
    while (!accelerometer_open_dev())
        printf("Unable to open accelerometer device!\n");

    while (!accelerometer_init())
        printf("Unable to initialize accelerometer!\n");

    // Draw graph axes
    draw_hline(0, 120, CANVAS_WIDTH - 1, Col_White);
    draw_vline(160, 0, CANVAS_HEIGHT - 1, Col_White);

    // Create RTK tasks
    task_create(IDLE, 0, READY_TASK_STATE, idle_code, idle_stack, STACK_SIZE);
    task_create(TASK_TIMER, 1, READY_TASK_STATE, timer_task_code, timer_stack, STACK_SIZE);
    task_create(TASK_ACC, 1, READY_TASK_STATE, task_acc_code, acc_stack, STACK_SIZE);
    task_create(TASK_ACC_FILTER, 1, READY_TASK_STATE, task_acc_filter_code, acc_filter_stack, STACK_SIZE);
    task_create(TASK_PLOT, 1, READY_TASK_STATE, task_plot_code, plot_stack, STACK_SIZE);

    // Start multitasking
    tsw_on();

    while (1)
        printf("Something went wrong!\n");

    return 0;
}

/* ================================================================
 *      CLEAR A RECTANGLE AREA OF THE SCREEN (FILL BLACK)
 * ================================================================ */
void clear_screen_range(size_t start_x, size_t start_y, size_t end_x, size_t end_y)
{
    for (size_t x = start_x; x <= end_x; x++)
    {
        for (size_t y = start_y; y <= end_y; y++)
        {
            write_pixel(x, y, Col_Black);
        }
    }
}
