struct counting_semaphore {
	int binarySemaphore1;
	int binarySemaphore2;
	int value;
};

int csem_alloc(struct counting_semaphore*, int);
void csem_free(struct counting_semaphore*);
void csem_down(struct counting_semaphore*);
void csem_up (struct counting_semaphore*);