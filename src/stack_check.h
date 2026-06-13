/**
 * Compile-time guard against a stack-resident object overflowing its task
 * stack. The overflow is otherwise silent (no FreeRTOS stack-overflow check)
 * and bricks USB before it enumerates.
 */

#pragma once

#ifdef ARDUINO
#include <FreeRTOS.h>
#include <cstddef>

namespace plrs {

// A task's largest stack-resident object may use at most half the stack,
// leaving the rest for call frames. FreeRTOS stack sizes are in words.
constexpr std::size_t CALL_FRAME_FACTOR = 2;

template <typename T>
constexpr bool fits_on_task_stack(std::size_t stack_words) {
  return sizeof(T) * CALL_FRAME_FACTOR <= stack_words * sizeof(StackType_t);
}

} // namespace plrs

#endif
