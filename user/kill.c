#include "kernel/types.h"
#include "kernel/stat.h"
#include "user/user.h"

int
main(int argc, char **argv)
{
  int i;

  if(argc < 2){
    fprintf(2, "usage: kill pid...\n");
    exit(1);
  }

  if(argc % 2 == 0){
    fprintf(2, "usage: must be signal\n");
    exit(1);
  }
  for(i=1; (i+1)<argc; i=i+2)
    kill(atoi(argv[i]), atoi(argv[i+1]));
  exit(0);
}
