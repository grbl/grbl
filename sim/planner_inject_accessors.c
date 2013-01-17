#include "../planner.h"

static block_t block_buffer[BLOCK_BUFFER_SIZE];  // A ring buffer for motion instructions
block_t *get_block_buffer() { return block_buffer; }

static volatile uint8_t block_buffer_head;       // Index of the next block to be pushed
uint8_t get_block_buffer_head() { return block_buffer_head; }

static volatile uint8_t block_buffer_tail;       // Index of the next block to be pushed
uint8_t get_block_buffer_tail() { return block_buffer_tail; }
