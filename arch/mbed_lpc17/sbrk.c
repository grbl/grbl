#include <reent.h>
#include <errno.h>
#include <stdlib.h>

/* This relies on linker magic to get stack and heap pointers */
extern unsigned int _pvHeapStart;

extern unsigned int __stacks_min__; 

extern void *heap_end;

#define __heap_start__	_pvHeapStart

#include "FreeRTOS.h"
#include "task.h"
#include "mpu_wrappers.h"

/* Low-level bulk RAM allocator -- used by Newlib's Malloc */
void *heap_end = NULL;
PRIVILEGED_FUNCTION void *_sbrk_r(struct _reent *ptr, ptrdiff_t incr)
{
	void *prev_heap_end, *next_heap_end, *ret;

	taskENTER_CRITICAL();
	{
		/* Initialize on first call */
		if (heap_end == NULL)
		{
			heap_end = (void *)&__heap_start__;
		}

		prev_heap_end = heap_end;

		/* Align to always be on 8-byte boundaries */
		next_heap_end = (void *)((((unsigned int)heap_end + incr) + 7) & ~7);

		/* Check if this allocation would collide with the heap */
	/*	if (next_heap_end > (void *)&__stacks_min__) FIXME
		{
			ptr->_errno = ENOMEM;
			ret = NULL;
		}
*/
		//else
		//{
			heap_end = next_heap_end;
			ret = (void *)prev_heap_end;
		//}
	}
	taskEXIT_CRITICAL();
	return ret;
}

