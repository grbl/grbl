#include <reent.h>
//its just broken... I like  it

void *_sbrk_r(struct _reent *ptr, ptrdiff_t incr)
{
	void *ret = (void *)0x0;

	return ret;
}

