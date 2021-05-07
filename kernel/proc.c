#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "riscv.h"
#include "spinlock.h"
#include "proc.h"
#include "defs.h"
#include "fs.h"

struct cpu cpus[NCPU];

struct proc proc[NPROC];

struct proc *initproc;

int nextpid = 1;
int nexttid = 1;

struct spinlock pid_lock;
struct spinlock tid_lock;


extern void forkret(void);
static void freeproc(struct proc *p);

extern void *start_sigret_injection(void);
extern void *end_sigret_injection(void);

extern char trampoline[]; // trampoline.S

// helps ensure that wakeups of wait()ing
// parents are not lost. helps obey the
// memory model when using p->parent.
// must be acquired before any p->lock.
struct spinlock wait_lock;

// Allocate a page for each process's kernel stack.
// Map it high in memory, followed by an invalid
// guard page.
void
proc_mapstacks(pagetable_t kpgtbl) {
  struct proc *p;
  struct thread *t;
  
  for(p = proc; p < &proc[NPROC]; p++) {
    for(t = p->threads; t < &p->threads[NTHREAD]; t++){
      char *pa = kalloc();
      if(pa == 0)
        panic("kalloc");
      // create a flat 2d map of p_i and its t_1 ... t_8 threads
      // example of the memory space: p1_t1 | pg | ... | p1_t8 | pg | p2_t1 | ...
      uint64 va = KSTACK(NTHREAD * (int) (p - proc) + (int) (t - p->threads));
      kvmmap(kpgtbl, va, (uint64)pa, PGSIZE, PTE_R | PTE_W);
    }
  }
}

// initialize the proc table at boot time.
void
procinit(void)
{
  struct proc *p;
  struct thread *t;
  
  initlock(&pid_lock, "nextpid");
  initlock(&tid_lock, "nexttid");
  initlock(&wait_lock, "wait_lock");
  for(p = proc; p < &proc[NPROC]; p++) {
      initlock(&p->lock, "proc");
      for(t = p->threads; t < &p->threads[NTHREAD]; t++){
        t->kstack = KSTACK(NTHREAD * (int) (p - proc) + (int) (t - p->threads));
        initlock(&t->lock, "thread");
      }
  }
}

// Must be called with interrupts disabled,
// to prevent race with process being moved
// to a different CPU.
int
cpuid()
{
  int id = r_tp();
  return id;
}

// Return this CPU's cpu struct.
// Interrupts must be disabled.
struct cpu*
mycpu(void) {
  int id = cpuid();
  struct cpu *c = &cpus[id];
  return c;
}

// Return the current struct proc *, or zero if none.
struct proc*
myproc(void) {
  push_off();
  struct cpu *c = mycpu();
  struct proc *p = c->proc;
  pop_off();
  return p;
}

struct thread*
mythread(void) {
  push_off();
  struct cpu *c = mycpu();
  struct thread *t = c->thread;
  pop_off();
  return t;
}

int
allocpid() {
  int pid;
  printf("acquire - pid_lock in: 110\n");
  acquire(&pid_lock);
  pid = nextpid;
  nextpid = nextpid + 1;
  printf("releas - pid_lock in: 110\n");
  release(&pid_lock);

  return pid;
}

int
alloctid() {
  int tid;
  printf("acquire - tid_lock in: 123\n");
  acquire(&tid_lock);
  tid = nexttid;
  nexttid = nexttid + 1;
  printf("release - tid_lock in: 123\n");
  release(&tid_lock);

  return tid;
}

// return 1 on failure 0 on succsess
static int
allocThread(struct thread *t, struct proc *p, int threadNum){
  printf("alloc thread\n");

  t->tid = alloctid();
  t->proc = p;

  if((t->backupTrapframe = (struct trapframe *)kalloc()) == 0){
    return 1;
  }
  
  // Set up new context to start executing at forkret,
  // which returns to user space.
  memset(&t->context, 0, sizeof(t->context));
  t->context.ra = (uint64)forkret;
  t->context.sp = t->kstack + PGSIZE;
  
  return 0;
}


// Look in the process table for an UNUSED proc.
// If found, initialize state required to run in the kernel,
// and return with p->lock held.
// If there are no free procs, or a memory allocation fails, return 0.
static struct proc*
allocproc(void)
{
  printf("alloc proc has ben started\n");
  struct proc *p;

  for(p = proc; p < &proc[NPROC]; p++) {
    printf("acquire p->lock in: 169 on proc: %s\n", p->name);
    acquire(&p->lock);
    if(p->state == UNUSED) {
      goto found;
    } else {
      printf("release p->lock in: 169\n");
      release(&p->lock);
    }
  }
  return 0;

found:
  p->pid = allocpid();
  p->state = USED;


  // Allocate a trapframe page.
  if((p->trapframes = kalloc()) == 0){
    freeproc(p);
    printf("release p->lock in: 146\n");
    release(&p->lock);
    return 0;
  }

    // init thread list 
  for(int i = 0; i < NTHREAD; i++){
    p->threads[i].state = T_UNUSED;
    p->threads[i].signalMaskBackup = 0;
    p->threads[i].threadIndexInProc = i;
    p->threads[i].chan = 0;
    p->threads[i].killed = 0;
    p->threads[i].trapframe = (struct trapframe *)p->trapframes + i;
    // init all threads locks
  }

  // An empty user page table.
  p->pagetable = proc_pagetable(p);
  if(p->pagetable == 0){
    freeproc(p);
    printf("release p->lock in: 192");
    release(&p->lock);
    return 0;
  }

  // allocate Thread memory 
  acquire(&p->threads[0].lock);
  if(allocThread(&p->threads[0], p, 0))
    return 0;

  // set defult masks and handlers
  for(int i = 0; i < 32; i++){
    p->signalHandlers[i] = (void*) SIG_DFL;
    p->signalHandlersMasks[i] = 0;
  }
  // set other masks
  p->pendingSignal = 0;
  p->signalMask = 0;

  printf("release p->lock in: 209\n");
  printf("proc %d has been allocated\n",p->pid);
  return p;
}

static void
freeThread(struct thread *t){
  t->chan = 0;
  t->killed = 0;
  t->xstate = 0;
  t->tid = 0;
  t->state = T_UNUSED;
}

static void
freeThreads(struct proc *p){
  for(int i=0; i < NTHREAD; i++){
    acquire(&p->threads[i].lock);
    if(p->threads[i].state != T_UNUSED){
      freeThread(&p->threads[i]);
    }
    release(&p->threads[i].lock);
  }
}

// free a proc structure and the data hanging from it,
// including user pages.
// p->lock must be held.
static void
freeproc(struct proc *p)
{

  if(p->pagetable)
    proc_freepagetable(p->pagetable, p->sz);

  p->pagetable = 0;
  p->sz = 0;
  p->pid = 0;
  p->parent = 0;
  p->name[0] = 0;
  p->killed = 0;
  p->xstate = 0;
  p->state = UNUSED;
  freeThreads(p);

  if(p->trapframes)
    kfree((void*) p->trapframes);
}

// Create a user page table for a given process,
// with no user memory, but with trampoline pages.
pagetable_t
proc_pagetable(struct proc *p)
{
  pagetable_t pagetable;

  // An empty page table.
  pagetable = uvmcreate();
  if(pagetable == 0)
    return 0;

  // map the trampoline code (for system call return)
  // at the highest user virtual address.
  // only the supervisor uses it, on the way
  // to/from user space, so not PTE_U.
  if(mappages(pagetable, TRAMPOLINE, PGSIZE,
              (uint64)trampoline, PTE_R | PTE_X) < 0){
    uvmfree(pagetable, 0);
    return 0;
  }

  // map the trapframe just below TRAMPOLINE, for trampoline.S.
  if(mappages(pagetable, TRAPFRAME, PGSIZE,
              (uint64)(p->trapframes), PTE_R | PTE_W) < 0){
    uvmunmap(pagetable, TRAMPOLINE, 1, 0);
    uvmfree(pagetable, 0);
    return 0;
  }

  return pagetable;
}

// Free a process's page table, and free the
// physical memory it refers to.
void
proc_freepagetable(pagetable_t pagetable, uint64 sz)
{
  uvmunmap(pagetable, TRAMPOLINE, 1, 0);
  uvmunmap(pagetable, TRAPFRAME, 1, 0);
  uvmfree(pagetable, sz);
}

// a user program that calls exec("/init")
// od -t xC initcode
uchar initcode[] = {
  0x17, 0x05, 0x00, 0x00, 0x13, 0x05, 0x45, 0x02,
  0x97, 0x05, 0x00, 0x00, 0x93, 0x85, 0x35, 0x02,
  0x93, 0x08, 0x70, 0x00, 0x73, 0x00, 0x00, 0x00,
  0x93, 0x08, 0x20, 0x00, 0x73, 0x00, 0x00, 0x00,
  0xef, 0xf0, 0x9f, 0xff, 0x2f, 0x69, 0x6e, 0x69,
  0x74, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00
};

// Set up first user process.
void
userinit(void)
{
  struct proc *p;

  p = allocproc();
  initproc = p;

  // allocate one user page and copy init's instructions
  // and data into it.
  uvminit(p->pagetable, initcode, sizeof(initcode));
  p->sz = PGSIZE;

  // prepare for the very first "return" from kernel to user.
  p->threads[0].trapframe->epc = 0;      // user program counter
  p->threads[0].trapframe->sp = PGSIZE;  // user stack pointer

  safestrcpy(p->name, "initcode", sizeof(p->name));
  p->cwd = namei("/");

  printf("wait_lock acquire in: 331\n");
  // set proccess to be used (not runnable)
  p->state = ALIVE;
  //set thread to be runnable
  p->threads[0].state = RUNNABLE;
  printf("wait_lock release in: 336\n");
  release(&p->lock);
  release(&p->threads[0].lock);
}

// Grow or shrink user memory by n bytes.
// Return 0 on success, -1 on failure.
int
growproc(int n)
{
  uint sz;
  struct proc *p = myproc();
  printf("acquire p->lock in: 351");
  // TODO sync
  // acquire(&p->lock);
  sz = p->sz;
  if(n > 0){
    if((sz = uvmalloc(p->pagetable, sz, sz + n)) == 0) {
      return -1;
    }
  } else if(n < 0){
    sz = uvmdealloc(p->pagetable, sz, sz + n);
  }
  p->sz = sz;
  printf("release p->lock in: 358\n");
  // release(&p->lock);
  return 0;
  
}

// Create a new process, copying the parent.
// Sets up child kernel stack to return as if from fork() system call.
int
fork(void)
{
  int i, pid;
  struct proc *np;
  struct proc *p = myproc();
  struct thread *t = mythread();

  // Allocate process.
  if((np = allocproc()) == 0){
    return -1;
  }
  printf("##fork## after alloc proc\n");

  // Copy user memory from parent to child.
  if(uvmcopy(p->pagetable, np->pagetable, p->sz) < 0){
    freeproc(np);
    printf("release p->lock in: 386\n");
    release(&np->lock);
    return -1;
  }
  np->sz = p->sz;

  // copy saved user registers.
  acquire(&wait_lock);
  *np->threads[0].trapframe = *t->trapframe;
  release(&wait_lock);
  // *(np->trapframe) = *(p->trapframe);

  // Cause fork to return 0 in the child.
  acquire(&wait_lock);
  np->threads[0].trapframe->a0 = 0;
  release(&wait_lock);
  // np->trapframe->a0 = 0;
  

  // set thread to part of np
  acquire(&wait_lock);
  np->threads[0].proc = np;
  release(&wait_lock);


  // increment reference counts on open file descriptors.
  for(i = 0; i < NOFILE; i++)
    if(p->ofile[i])
      np->ofile[i] = filedup(p->ofile[i]);
  np->cwd = idup(p->cwd);

  safestrcpy(np->name, p->name, sizeof(p->name));

  pid = np->pid;
  release(&np->lock);

  acquire(&wait_lock);
  np->parent = p;
  release(&wait_lock);

  printf("release - wait_lock in: 419\n");
  printf("acquire np->lock in: 422\n");
  acquire(&np->lock);
  np->state = ALIVE;
  np->threads[0].state = RUNNABLE;

  // the child points also on the father signal handler
  for(int i = 0; i < 32; i++){
    np->signalHandlers[i] = p->signalHandlers[i];
    np->signalHandlersMasks[i] = p->signalHandlersMasks[i];
  }

  np->signalMask= p->signalMask;
  printf("release np->lock in: 430\n");
    
  release(&np->lock);
  release(&np->threads[0].lock);

  return pid;
}

// Pass p's abandoned children to init.
// Caller must hold wait_lock.
void
reparent(struct proc *p)
{
  struct proc *pp;

  for(pp = proc; pp < &proc[NPROC]; pp++){
    if(pp->parent == p){
      pp->parent = initproc;
      printf("initproc wakeup\n");
      wakeup(initproc);
    }
  }
}

// when a proc exits we have to terminate all
// threads that running on it
void
killAllChildrens(struct proc *p, struct thread *t){
  struct thread *th;
  for (th = p->threads; th < &p->threads[NTHREAD]; th++){
    if(t != th){
      acquire(&th->lock);
      if(th->state == SLEEPING)
        th->state = RUNNABLE;
      if(th->state != T_UNUSED)
        th->killed = 1;
      release(&th->lock);
    }
  }
}
// before existing, we have to make sure
// all childrens are killed.
void 
waitToOthers(struct proc *p, struct thread *t){
  struct thread *th;
  int ready = 0;
  while(1){
    ready = 1;
    for (th = p->threads; th < &p->threads[NTHREAD]; th++){
      if(t != th){
        if(t->state !=T_UNUSED && t->state != T_ZOMBIE)
          ready = 0;
      }
    }
    if(ready)
      break;
  }
}

// Exit the current process.  Does not return.
// An exited process remains in the zombie state
// until its parent calls wait().
void
exit(int status)
{
  struct proc *p = myproc();

  if(p == initproc)
    panic("init exiting");

  // Close all open files.
  for(int fd = 0; fd < NOFILE; fd++){
    if(p->ofile[fd]){
      struct file *f = p->ofile[fd];
      fileclose(f);
      p->ofile[fd] = 0;
    }
  }
  acquire(&p->lock);
  killAllChildrens(p, mythread());
  release(&p->lock);

  begin_op();
  iput(p->cwd);
  end_op();
  p->cwd = 0;

  waitToOthers(p, mythread());
  
  printf("acquire wait_lock in: 479\n");
  acquire(&wait_lock);

  // Give any children to init.
  reparent(p);

  // Parent might be sleeping in wait().
  printf("wakeup wakeup\n");
  wakeup(p->parent);
  
  acquire(&p->lock);
  // update thread 
  mythread()->state = T_ZOMBIE;
  mythread()->xstate = status;
  p->state = ZOMBIE;
  p->xstate = status;

  release(&p->lock);
  release(&wait_lock);

  // Jump into the scheduler, never to return.
  acquire(&mythread()->lock);
  sched();
  panic("zombie exit");
}

// Wait for a child process to exit and return its pid.
// Return -1 if this process has no children.
int
wait(uint64 addr)
{
  struct proc *np;
  int havekids, pid;
  struct proc *p = myproc();
  printf("acquire wait_lock in: 508\n");
  acquire(&wait_lock);

  for(;;){
    // Scan through table looking for exited children.
    havekids = 0;
    for(np = proc; np < &proc[NPROC]; np++){
      if(np->parent == p){
        // make sure the child isn't still in exit() or swtch().
        printf("acquire np->lock in: 517\n");
        acquire(&np->lock);

        havekids = 1;
        if(np->state == ZOMBIE){
          // Found one.
          pid = np->pid;
          if(addr != 0 && copyout(p->pagetable, addr, (char *)&np->xstate,
                                  sizeof(np->xstate)) < 0) {
            printf("release - np->lock in: 524\n");          
            release(&np->lock);
            printf("release - wait_lock in: 526\n");
            release(&wait_lock);
            return -1;
          }
          freeproc(np);
          printf("release - np->lock in: 531\n");          
          release(&np->lock);
          printf("release - wait_lock in: 533\n");
          release(&wait_lock);
          return pid;
        }
        printf("release - np->lock in: 537\n");
        release(&np->lock);
      }
    }

    // No point waiting if we don't have any children.
    if(!havekids || p->killed){
      printf("release - wait_lock in: 544\n");
      release(&wait_lock);
      return -1;
    }
    
    // Wait for a child to exit.
    sleep(p, &wait_lock);  //DOC: wait-sleep
  }
}

// Per-CPU process scheduler.
// Each CPU calls scheduler() after setting itself up.
// Scheduler never returns.  It loops, doing:
//  - choose a process to run.
//  - swtch to start running that process.
//  - eventually that process transfers control
//    via swtch back to the scheduler.
void
scheduler(void)
{
  struct proc *p;
  struct thread *t;
  struct cpu *c = mycpu();
  printf("started scheduler\n");
  c->proc = 0;
  for(;;){
    // Avoid deadlock by ensuring that devices can interrupt.
    intr_on();
    for(p = proc; p < &proc[NPROC]; p++) {
      // if the proc is terminated jump to the next one
      // TODO - check if it is OK ?
      if(p->state == UNUSED || p->state == ZOMBIE || p->state == UNUSED){
        continue;
      }
        
      for(t = p->threads; t < &p->threads[NTHREAD]; t++){
        // printf("%d is about to acquire its lock\n", t->tid);
        acquire(&t->lock);
        if(t->state == RUNNABLE) {
          // Switch to chosen process.  It is the process's job
          // to release its lock and then reacquire it
          // before jumping back to us.
          // p->state = USED;
          c->proc = p;
          printf("thread: %d is RUNNING unde proc: %s\n", t->tid, proc->name);
          t->state = RUNNING;
          c->thread = t;
          swtch(&c->context, &t->context);
          // Process is done running for now.
          // It should have changed its p->state before coming back.
          c->proc = 0;
          c->thread = 0;
        }
        printf("%d is about to release its lock in schduler\n", t->tid);
        release(&t->lock);
      }
    }
    
  }
}

// Switch to scheduler.  Must hold only p->lock
// and have changed proc->state. Saves and restores
// intena because intena is a property of this
// kernel thread, not this CPU. It should
// be proc->intena and proc->noff, but that would
// break in the few places where a lock is held but
// there's no process.
void
sched(void)
{
  int intena;
  // struct proc *p = myproc();
  struct thread *t = mythread();
  printf("noff: %d\n",mycpu()->noff);

  if(!holding(&t->lock))
    panic("sched t->lock");
  if(mycpu()->noff != 1)
    panic("sched locks");
  if(t->state == RUNNING)
    panic("sched running");
  if(intr_get())
    panic("sched interruptible");

  intena = mycpu()->intena;
  swtch(&t->context, &mycpu()->context);
  mycpu()->intena = intena;
}

// Give up the CPU for one scheduling round.
void
yield(void)
{
  struct thread *t = mythread();
  printf("acquire p->lock in: 623\n");
  acquire(&t->lock);
  t->state = RUNNABLE;
  sched();
  printf("releas p->lock in: 623\n");
  release(&t->lock);
}

// A fork child's very first scheduling by scheduler()
// will swtch to forkret.
void
forkret(void)
{
  static int first = 1;

  // Still holding t->lock from scheduler.
  printf("release - mythread() in: 650 by thread:%d and proc: %s\n", mythread()->tid, mythread()->name);
  release(&mythread()->lock);

  if (first) {
    // File system initialization must be run in the context of a
    // regular process (e.g., because it calls sleep), and thus cannot
    // be run from main().
    first = 0;
    printf("before fsinit\n");
    fsinit(ROOTDEV);
  }
  printf("before usertrap\n");

  usertrapret();
}

// Atomically release lock and sleep on chan.
// Reacquires lock when awakened.
void
sleep(void *chan, struct spinlock *lk)
{
  // struct proc *p = myproc();
  struct thread *t = mythread();
  
  // Must acquire p->lock in order to
  // change p->state and then call sched.
  // Once we hold p->lock, we can be
  // guaranteed that we won't miss any wakeup
  // (wakeup locks p->lock),
  // so it's okay to release lk.

  acquire(&t->lock);  //DOC: sleeplock1
  release(lk);

  // Go to sleep.
  t->chan = chan;
  t->state = SLEEPING;

  printf("sleep sched with tid:%d and chan: %d\n", t->tid, chan);
  sched();

  // Tidy up.
  t->chan = 0;

  // Reacquire original lock.
  release(&t->lock);
  acquire(lk);
}

// Wake up all processes sleeping on chan.
// Must be called without any p->lock.
void
wakeup(void *chan)
{
  struct proc *p;
  struct thread *t;

  for(p = proc; p < &proc[NPROC]; p++) {
    if(p->state != ALIVE)
      continue;
    acquire(&p->lock);
    for(t = p->threads; t < &p->threads[NTHREAD]; t++){
      if(t != mythread()){
        acquire(&t->lock);
        if(t->state == SLEEPING && t->chan == chan) {
          t->state = RUNNABLE;
        }
        release(&t->lock);
      }
    }
    acquire(&p->lock);
  }
}

// Kill the process with the given pid.
// The victim won't exit until it tries to return
// to user space (see usertrap() in trap.c).
int
kill(int pid, int signum)
{
  struct proc *p;
  struct thread *t;

  for(p = proc; p < &proc[NPROC]; p++){
    printf("acquire p-lock: 732\n");
    acquire(&p->lock);
    if(p->pid == pid){
      
      // if kill made on a unvalid proccess
      if(p->state == ZOMBIE || p->state == UNUSED || signum < 0 || signum > 31 || pid < 0) 
      {
        release(&p->lock);
        return -1;
      }

      // update the pending signles on the proc
      uint newSignal = p->pendingSignal | (1 << signum);
      p->pendingSignal = newSignal;

      for(t = p->threads; t < &p->threads[NTHREAD]; t++){
        if(mythread() != t){

          acquire(&t->lock);

          if (t->state == SLEEPING)
            t->state = RUNNABLE;

          release(&t->lock);
        }
      }

      release(&p->lock);
      return 0;
    }
    printf("release p->lock in: 728");
    release(&p->lock);
  }
  return -1;
}

// Copy to either a user address, or kernel address,
// depending on usr_dst.
// Returns 0 on success, -1 on error.
int
either_copyout(int user_dst, uint64 dst, void *src, uint64 len)
{
  struct proc *p = myproc();
  if(user_dst){
    return copyout(p->pagetable, dst, src, len);
  } else {
    memmove((char *)dst, src, len);
    return 0;
  }
}

// Copy from either a user address, or kernel address,
// depending on usr_src.
// Returns 0 on success, -1 on error.
int
either_copyin(void *dst, int user_src, uint64 src, uint64 len)
{
  struct proc *p = myproc();
  if(user_src){
    return copyin(p->pagetable, dst, src, len);
  } else {
    memmove(dst, (char*)src, len);
    return 0;
}
  }

// Print a process listing to console.  For debugging.
// Runs when user types ^P on console.
// No lock to avoid wedging a stuck machine further.
// TODO add thread list printing ?????

void
procdump(void)
{
  static char *states[] = {
  
  [UNUSED]    "unused",
  [SLEEPING]  "sleep ",
  [RUNNABLE]  "runble",
  [RUNNING]   "run   ",
  [ZOMBIE]    "zombie"
  };
  struct proc *p;
  char *state;
  acquire(&wait_lock);
  printf("\n");
  for(p = proc; p < &proc[NPROC]; p++){
    if(p->state == UNUSED)
      continue;
    if(p->state >= 0 && p->state < NELEM(states) && states[p->state])
      state = states[p->state];
    else
      state = "???";
    printf("%d %s %s", p->pid, state, p->name);
    printf("\n");
  }
  release(&wait_lock);
}

int 
sigaction (int signum, const struct sigaction *act, struct sigaction *oldact)
{
  if (signum < 0 || signum > 31 || signum == SIGKILL || signum == SIGSTOP || act ==0)
  {
    return -1;
  }

  struct proc *p = myproc();
  
  if (oldact != 0)
  {
    copyout(p->pagetable, (uint64)&oldact->sa_handler, (char *)&p->signalHandlers[signum], sizeof(p->signalHandlers[signum]));
    copyout(p->pagetable, (uint64)&oldact->sigmask, (char *)&p->signalHandlersMasks[signum], sizeof(p->signalMask));
  }

  copyin(p->pagetable, (char *)&p->signalHandlers[signum], (uint64)&act->sa_handler, sizeof(act->sa_handler));
  copyin(p->pagetable, (char *)&p->signalHandlersMasks[signum], (uint64)&act->sigmask, sizeof(act->sigmask));
  
  // printf("end of sigaction %p\n", p->signalHandlers[signum]);

  return 0;
}

uint
sigprocmask(uint sigmask)
{
  struct proc *p = myproc();
  uint ret = p->signalMask;
  p->signalMask = sigmask;
  return ret;
}

void
sigret(void)
{
  struct proc *p = myproc();
  if (!p)
  {
    return;
  }
  //1.
  copyin(p->pagetable,(char *)p->trapframe, (uint64)p->trapframeBackup,sizeof(struct trapframe));
  p->trapframe->sp += sizeof(struct trapframe);
  //2.
  p->signalMask = p->signalMaskBackup;
  //3.
  p->handling = 0;
}

void
checkCont(struct proc* p){
  for(int i = 0; i < 32; i++){
    if (p->signalHandlers[i] == (void *)SIGCONT)
    {
      p->sigcont = 1;
    }
  }
  if (p->sigcont == 0 && (p->signalHandlers[SIGCONT] == (void *)SIG_DFL && (p->pendingSignal & (1 << SIGCONT))))
  {
      p->sigcont = 1;
  }
}

void
sigstopHandler()
{
  struct proc *p = myproc();
  while(!p->sigcont )
  {
    checkCont(p);
    yield();
  }
  p->sigcont = 0;
}

void
sigkillHandler()
{
  struct proc *p = myproc();
  p->killed = 1;
}

void
signalHandler()
{
  struct proc* p = myproc();
  if (!p || p->handling)
  {
    return;
  }
  
  for (int i = 0; i < 32; i++)
  {
    uint signumbit = (1 << i);
    // check if the signal is pending and not masked
    if ((p->pendingSignal & signumbit) && !(p->signalMask & signumbit)  && (p->signalHandlers[i] != (void *)SIG_IGN))
    {
      if (p->signalHandlers[i] == (void *)SIGCONT)
      {
        p->sigcont = 1;
      }
      else if (p->signalHandlers[i] == (void *)SIG_DFL)
      {
        if (i == SIGSTOP)
        {
          sigstopHandler();
        }
        else if (i == SIGCONT)
        {
          p->sigcont = 1;
        }
        else
        {
          sigkillHandler();
          
        }
        p->pendingSignal ^= signumbit;        
      }
      else
      {
        //1.
        //2.
        p->signalMaskBackup = p->signalMask;
        p->signalMask = p->signalHandlersMasks[i];
        //3.
        p->handling = 1;
        //4.
        p->trapframe->sp -= sizeof(struct trapframe);
        p->trapframeBackup = (struct trapframe *)(p->trapframe->sp); //not sure that will make a backup, I think that need to make deep copy of it
        //5.
        copyout(p->pagetable, (uint64)p->trapframeBackup, (char *)p->trapframe, sizeof(struct trapframe));
        //6.
        p->trapframe->epc = (uint64)p->signalHandlers[i];
        //7.
        uint64 sizeOfSigret=(uint64)&end_sigret_injection - (uint64)&start_sigret_injection; 
        p->trapframe->sp -= sizeOfSigret;
        //8.
        copyout(p->pagetable, (uint64)p->trapframe->sp, (char *)&start_sigret_injection, sizeOfSigret);
        //9.
        p->trapframe->a0 = i;
        p->trapframe->ra = p->trapframe->sp;
        p->pendingSignal ^= signumbit;
      }
    }
  }
}
int
kthread_create(void(*start_func)(), void *stack)
{
  struct proc *p = myproc();
  struct thread *t;
  int index = 0;
  for(t = p->threads; t < &proc->threads[NTHREAD]; t++) {
    acquire(&t->lock); 
    if(t->state == T_UNUSED) {
      goto found;
    }
    index++;
  }
  printf("t->lock in: 876");      
  release(&t->lock);
  return 0;

found:
  t->tid = allocpid();
  t->state = USED;

  if(allocThread(&p->threads[index], p, index)){
    freeThread(t);
    t->state = UNUSED;
    p->state = UNUSED;
    release(&t->lock);
    return 0;
  }
  printf("release t->lock in: 887\n");
  
  return t->tid;
}


int
kthread_id(){
  int tid =  mythread()->tid;
  if(tid == 0)
    return -1;
  return tid;
}

void
kthread_exit(int status){
  // update thread status before exisiting to be terminated
  mythread()->xstate = status;
  mythread()->state = ZOMBIE;

  struct proc *p = myproc();
  struct thread *t;

  // wake up all thread that sleep on this thread
  // before it terminates forgood and creates deadlock
  printf("exitthread wakeup\n");
  wakeup(&mythread()->tid);
  acquire(&p->lock);
  // check its the last thread
  int onlyThread = 1;
  for(t = p->threads; t < &p->threads[NTHREAD]; t++){
    if(t != mythread()){
      if(t != mythread() && t->state != T_ZOMBIE &&  t->state != T_UNUSED){
        onlyThread = 0;
        break;
      }
    }
  }

  // didn't found other runnable threads
  if(onlyThread)
    exit(status);

  release(&p->lock);

  acquire(&t->lock);
  sched();
  release(&t->lock);
};

int
kthread_join(int thread_id, int* status){

  struct proc *p = myproc();
  struct thread *t;

  // wake up all thread that sleep on this thread
  // before it terminates forgood and creates deadlock
  wakeup(&mythread()->tid);
  acquire(&wait_lock);
  // check its the last thread
  for(t = p->threads; t < &p->threads[NTHREAD]; t++){
    if(t->tid == thread_id){
      if(t->state == T_ZOMBIE){
        // wrtie in status threads exist status
        copyout(p->pagetable, (uint64)&status, (char *)&t->xstate, sizeof(int));
        release(&wait_lock);
        return 0;
      }
      else if(t->state != T_UNUSED){
        sleep(&thread_id, &wait_lock);
        copyout(p->pagetable, (uint64)&status, (char *)&t->xstate, sizeof(int));
        release(&wait_lock);
        return 0;
      }
    }
  }
  release(&wait_lock);
  return -1;
}
