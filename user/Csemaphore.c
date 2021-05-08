#include "Csemaphore.h"
#include "kernel/types.h"
#include "user/user.h"
#include "kernel/fcntl.h"

int
csem_alloc(struct counting_semaphore* sem, int initial_value)
{
  sem->binarySemaphore1 = bsem_alloc();
  sem->binarySemaphore2 = bsem_alloc();
  
  if ((sem->binarySemaphore1 == -1) | (sem->binarySemaphore2 == -1))
  {
    return -1;
  }

  sem->value = initial_value;

  if (!initial_value)
  {
    bsem_down(sem->binarySemaphore2);
  }

  return 0;  
}

void
csem_free(struct counting_semaphore* sem)
{
  bsem_free(sem->binarySemaphore1);
  bsem_free(sem->binarySemaphore2);
}

void
csem_down(struct counting_semaphore* sem)
{
	bsem_down(sem->binarySemaphore2);
	bsem_down(sem->binarySemaphore1);
  sem->value--;
	if (sem->value > 0)
		bsem_up(sem->binarySemaphore2);
	bsem_up(sem->binarySemaphore1);
}

void
csem_up(struct counting_semaphore* sem)
{
    bsem_down(sem->binarySemaphore1);
    sem->value++;
    if (sem->value == 1)
        bsem_up(sem->binarySemaphore2);
    bsem_up(sem->binarySemaphore1);
}