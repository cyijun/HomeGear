#pragma once
#include "FreeRTOS.h"
#include <cstddef>
#include <cstring>
#include <deque>
#include <vector>

struct MockQueue {
  size_t capacity, itemSize;
  std::deque<std::vector<unsigned char>> items;
};
using QueueHandle_t = MockQueue *;
inline QueueHandle_t xQueueCreate(size_t capacity, size_t itemSize) { return new MockQueue{capacity, itemSize, {}}; }
inline BaseType_t xQueueSend(QueueHandle_t queue, const void *item, TickType_t) {
  if (!queue || queue->items.size() == queue->capacity) return pdFALSE;
  const auto *bytes = static_cast<const unsigned char *>(item);
  queue->items.emplace_back(bytes, bytes + queue->itemSize);
  return pdTRUE;
}
inline BaseType_t xQueueReceive(QueueHandle_t queue, void *item, TickType_t) {
  if (!queue || queue->items.empty()) return pdFALSE;
  std::memcpy(item, queue->items.front().data(), queue->itemSize);
  queue->items.pop_front();
  return pdTRUE;
}
inline void vQueueDelete(QueueHandle_t queue) { delete queue; }
