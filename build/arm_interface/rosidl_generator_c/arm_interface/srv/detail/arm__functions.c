// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from arm_interface:srv/Arm.idl
// generated code does not contain a copyright notice
#include "arm_interface/srv/detail/arm__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `obj_class`
#include "rosidl_runtime_c/string_functions.h"

bool
arm_interface__srv__Arm_Request__init(arm_interface__srv__Arm_Request * msg)
{
  if (!msg) {
    return false;
  }
  // xy
  // obj_class
  if (!rosidl_runtime_c__String__init(&msg->obj_class)) {
    arm_interface__srv__Arm_Request__fini(msg);
    return false;
  }
  return true;
}

void
arm_interface__srv__Arm_Request__fini(arm_interface__srv__Arm_Request * msg)
{
  if (!msg) {
    return;
  }
  // xy
  // obj_class
  rosidl_runtime_c__String__fini(&msg->obj_class);
}

bool
arm_interface__srv__Arm_Request__are_equal(const arm_interface__srv__Arm_Request * lhs, const arm_interface__srv__Arm_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // xy
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->xy[i] != rhs->xy[i]) {
      return false;
    }
  }
  // obj_class
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->obj_class), &(rhs->obj_class)))
  {
    return false;
  }
  return true;
}

bool
arm_interface__srv__Arm_Request__copy(
  const arm_interface__srv__Arm_Request * input,
  arm_interface__srv__Arm_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // xy
  for (size_t i = 0; i < 2; ++i) {
    output->xy[i] = input->xy[i];
  }
  // obj_class
  if (!rosidl_runtime_c__String__copy(
      &(input->obj_class), &(output->obj_class)))
  {
    return false;
  }
  return true;
}

arm_interface__srv__Arm_Request *
arm_interface__srv__Arm_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_interface__srv__Arm_Request * msg = (arm_interface__srv__Arm_Request *)allocator.allocate(sizeof(arm_interface__srv__Arm_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arm_interface__srv__Arm_Request));
  bool success = arm_interface__srv__Arm_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arm_interface__srv__Arm_Request__destroy(arm_interface__srv__Arm_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arm_interface__srv__Arm_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arm_interface__srv__Arm_Request__Sequence__init(arm_interface__srv__Arm_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_interface__srv__Arm_Request * data = NULL;

  if (size) {
    data = (arm_interface__srv__Arm_Request *)allocator.zero_allocate(size, sizeof(arm_interface__srv__Arm_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arm_interface__srv__Arm_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arm_interface__srv__Arm_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
arm_interface__srv__Arm_Request__Sequence__fini(arm_interface__srv__Arm_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      arm_interface__srv__Arm_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

arm_interface__srv__Arm_Request__Sequence *
arm_interface__srv__Arm_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_interface__srv__Arm_Request__Sequence * array = (arm_interface__srv__Arm_Request__Sequence *)allocator.allocate(sizeof(arm_interface__srv__Arm_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arm_interface__srv__Arm_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arm_interface__srv__Arm_Request__Sequence__destroy(arm_interface__srv__Arm_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arm_interface__srv__Arm_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arm_interface__srv__Arm_Request__Sequence__are_equal(const arm_interface__srv__Arm_Request__Sequence * lhs, const arm_interface__srv__Arm_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arm_interface__srv__Arm_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arm_interface__srv__Arm_Request__Sequence__copy(
  const arm_interface__srv__Arm_Request__Sequence * input,
  arm_interface__srv__Arm_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arm_interface__srv__Arm_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arm_interface__srv__Arm_Request * data =
      (arm_interface__srv__Arm_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arm_interface__srv__Arm_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arm_interface__srv__Arm_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arm_interface__srv__Arm_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
arm_interface__srv__Arm_Response__init(arm_interface__srv__Arm_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
arm_interface__srv__Arm_Response__fini(arm_interface__srv__Arm_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
arm_interface__srv__Arm_Response__are_equal(const arm_interface__srv__Arm_Response * lhs, const arm_interface__srv__Arm_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
arm_interface__srv__Arm_Response__copy(
  const arm_interface__srv__Arm_Response * input,
  arm_interface__srv__Arm_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

arm_interface__srv__Arm_Response *
arm_interface__srv__Arm_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_interface__srv__Arm_Response * msg = (arm_interface__srv__Arm_Response *)allocator.allocate(sizeof(arm_interface__srv__Arm_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arm_interface__srv__Arm_Response));
  bool success = arm_interface__srv__Arm_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arm_interface__srv__Arm_Response__destroy(arm_interface__srv__Arm_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arm_interface__srv__Arm_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arm_interface__srv__Arm_Response__Sequence__init(arm_interface__srv__Arm_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_interface__srv__Arm_Response * data = NULL;

  if (size) {
    data = (arm_interface__srv__Arm_Response *)allocator.zero_allocate(size, sizeof(arm_interface__srv__Arm_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arm_interface__srv__Arm_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arm_interface__srv__Arm_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
arm_interface__srv__Arm_Response__Sequence__fini(arm_interface__srv__Arm_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      arm_interface__srv__Arm_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

arm_interface__srv__Arm_Response__Sequence *
arm_interface__srv__Arm_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_interface__srv__Arm_Response__Sequence * array = (arm_interface__srv__Arm_Response__Sequence *)allocator.allocate(sizeof(arm_interface__srv__Arm_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arm_interface__srv__Arm_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arm_interface__srv__Arm_Response__Sequence__destroy(arm_interface__srv__Arm_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arm_interface__srv__Arm_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arm_interface__srv__Arm_Response__Sequence__are_equal(const arm_interface__srv__Arm_Response__Sequence * lhs, const arm_interface__srv__Arm_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arm_interface__srv__Arm_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arm_interface__srv__Arm_Response__Sequence__copy(
  const arm_interface__srv__Arm_Response__Sequence * input,
  arm_interface__srv__Arm_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arm_interface__srv__Arm_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arm_interface__srv__Arm_Response * data =
      (arm_interface__srv__Arm_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arm_interface__srv__Arm_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arm_interface__srv__Arm_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arm_interface__srv__Arm_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "arm_interface/srv/detail/arm__functions.h"

bool
arm_interface__srv__Arm_Event__init(arm_interface__srv__Arm_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    arm_interface__srv__Arm_Event__fini(msg);
    return false;
  }
  // request
  if (!arm_interface__srv__Arm_Request__Sequence__init(&msg->request, 0)) {
    arm_interface__srv__Arm_Event__fini(msg);
    return false;
  }
  // response
  if (!arm_interface__srv__Arm_Response__Sequence__init(&msg->response, 0)) {
    arm_interface__srv__Arm_Event__fini(msg);
    return false;
  }
  return true;
}

void
arm_interface__srv__Arm_Event__fini(arm_interface__srv__Arm_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  arm_interface__srv__Arm_Request__Sequence__fini(&msg->request);
  // response
  arm_interface__srv__Arm_Response__Sequence__fini(&msg->response);
}

bool
arm_interface__srv__Arm_Event__are_equal(const arm_interface__srv__Arm_Event * lhs, const arm_interface__srv__Arm_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!arm_interface__srv__Arm_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!arm_interface__srv__Arm_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
arm_interface__srv__Arm_Event__copy(
  const arm_interface__srv__Arm_Event * input,
  arm_interface__srv__Arm_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!arm_interface__srv__Arm_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!arm_interface__srv__Arm_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

arm_interface__srv__Arm_Event *
arm_interface__srv__Arm_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_interface__srv__Arm_Event * msg = (arm_interface__srv__Arm_Event *)allocator.allocate(sizeof(arm_interface__srv__Arm_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arm_interface__srv__Arm_Event));
  bool success = arm_interface__srv__Arm_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arm_interface__srv__Arm_Event__destroy(arm_interface__srv__Arm_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arm_interface__srv__Arm_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arm_interface__srv__Arm_Event__Sequence__init(arm_interface__srv__Arm_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_interface__srv__Arm_Event * data = NULL;

  if (size) {
    data = (arm_interface__srv__Arm_Event *)allocator.zero_allocate(size, sizeof(arm_interface__srv__Arm_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arm_interface__srv__Arm_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arm_interface__srv__Arm_Event__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
arm_interface__srv__Arm_Event__Sequence__fini(arm_interface__srv__Arm_Event__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      arm_interface__srv__Arm_Event__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

arm_interface__srv__Arm_Event__Sequence *
arm_interface__srv__Arm_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_interface__srv__Arm_Event__Sequence * array = (arm_interface__srv__Arm_Event__Sequence *)allocator.allocate(sizeof(arm_interface__srv__Arm_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arm_interface__srv__Arm_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arm_interface__srv__Arm_Event__Sequence__destroy(arm_interface__srv__Arm_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arm_interface__srv__Arm_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arm_interface__srv__Arm_Event__Sequence__are_equal(const arm_interface__srv__Arm_Event__Sequence * lhs, const arm_interface__srv__Arm_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arm_interface__srv__Arm_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arm_interface__srv__Arm_Event__Sequence__copy(
  const arm_interface__srv__Arm_Event__Sequence * input,
  arm_interface__srv__Arm_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arm_interface__srv__Arm_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arm_interface__srv__Arm_Event * data =
      (arm_interface__srv__Arm_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arm_interface__srv__Arm_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arm_interface__srv__Arm_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arm_interface__srv__Arm_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
