
/**
 * Simple TCP server to simulate a medipix Labview system.
 * Arguments:
 *   port number - port number to listen for connections
 * 
 * Matthew Pearson
 * Oct 2011
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <netinet/in.h>
#include <string.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>

#define MAXLINE 256

/*Function prototypes.*/
void sig_chld(int signo);
int echo_request(int socket_fd);


int main(int argc, char *argv[])
{
  int fd, fd2; 
  socklen_t client_size;
  pid_t child_pid = 0;
  struct sockaddr_in server_addr, client_addr;
  char *buff = NULL;

  printf("Started Medipix simulation server...\n");

  if (argc != 2) {
    printf("  ERROR: Use: %s {port number to use for listen socket}\n", argv[0]);
    exit(EXIT_FAILURE);
  }
  
  /* Create a TCP socket.*/
  fd = socket(AF_INET, SOCK_STREAM, 0);

  /* Create and initialise a socket address structure.*/
  memset(&server_addr, 0, sizeof(struct sockaddr_in));
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  server_addr.sin_port = htons(atoi(argv[1]));

  /* Bind the socket address to the socket buffer.*/
  bind(fd, (struct sockaddr *) &server_addr, sizeof(server_addr));

  /* Listen for incoming connections. */
  listen(fd, 10);

  /* Install signal handler to clean up children.*/
  signal(SIGCHLD, sig_chld);

  /*Loop forever.*/
  while(1) {
    client_size = sizeof(client_addr);
    if ((fd2 = accept(fd, (struct sockaddr *) &client_addr, &client_size)) < 0) {
      if (errno == EINTR) {
	continue; /*Deal with interupted system call, since this blocks.*/
      } else {
	perror(argv[0]);
	exit(EXIT_FAILURE);
      }
    }

    if ((child_pid = fork()) == 0) {
      /*Child process.*/
      printf("In child...\n");
      close(fd);
      while(1) { /*Keep reading until connection is closed or there is an error.*/
	if (echo_request(fd2) != EXIT_SUCCESS) {
	  perror(argv[0]);
	  printf("  Client failed to handle protocol, or connection closed.\n");
	  exit(EXIT_FAILURE);
	}
      }
      exit(EXIT_SUCCESS);
    }

    /*Parent process.*/
    close(fd2);
  }

  /*Should never get here.*/
  printf("Finishing Medipix server.\n");
  return EXIT_SUCCESS;

}


/**
 * Signal handler for SIGCHLD.
 *
 */ 
void sig_chld(int signo)
{
  pid_t pid = 0;
  int status = 1;

  while ((pid = waitpid(-1, &status, WNOHANG)) > 0) {
    printf("Child %d has terminated. Status = %d\n", pid, status);
  }
}


/**
 * Read from socket and send back response. 
 * Read a maximum of MAXLINE, or until a newline, then echo the command back.
 */
int echo_request(int socket_fd) 
{
  int nleft = MAXLINE-1;
  int nread = 0;
  char buffer[MAXLINE] = {'\0'};
  char tempbuf[MAXLINE];
  char response[MAXLINE] = {'\0'};
  char *bptr = NULL;
  int i = 0;
  int command = 0;
  char *tok = NULL;

  bptr = &buffer;

  if (socket_fd == NULL) {
    return EXIT_FAILURE;
  }


  ///Read until nothing left.
  while (nleft > 0) {
    if ((nread = read(socket_fd, bptr, nleft)) < 0) {
      if (errno == EINTR) {
	nread = 0;
	printf("EINTR");
      } else {
	printf("Unknown read error.");
	return EXIT_FAILURE;
      }
    } else if (nread == 0) {
      printf("Read nothing. Socket closed.\n");
      return EXIT_FAILURE; //Done. Client has probably disconnected.
    }

    nleft = nleft - nread;

    //Read until '\r\n', then print the string. Then reset the buffer pointer.
    for (i=0; i<nread; i++) {
      if ((*bptr=='\r')&&(*(bptr+1)=='\n')) {
	//printf("From client: %s\n", buffer);
	bptr = &buffer;
	command = 1;
      }
      bptr++;
    }

    /*Default response is an error (for a SET command)*/
    strncpy(response, "MPX,1,\r\n", MAXLINE);


    //Handle command
    if (command == 1) {
      bptr = &buffer;
      strncpy(tempbuf, buffer, MAXLINE);
      
      tok = strtok(tempbuf, ",");
      if (!strncmp(tok,"MPX",3)) {
	tok = strtok(NULL, ",");
	if (!strncmp(tok,"SET",3)) {
	  tok = strtok(NULL, ",");
	  strncpy(response,"MPX,0,\r\n",MAXLINE);
	}
	else if (!strncmp(tok,"GET",3)) {
	  tok = strtok(NULL, ",");
	  if (tok != NULL) {
	    sprintf(response, "MPX,0,%s,5,\r\n", tok);
	  } else {
	    if (write(socket_fd, "MPX,1,\r\n", 8) <= 0) {
	      printf("Error writing back to client.\n");
	      return EXIT_FAILURE;
	    }
	  }
	}
	
	if (write(socket_fd, response, MAXLINE-nleft+1) <= 0) {
	  printf("Error writing back to client.\n");
	  return EXIT_FAILURE;
	}
	
      } else {
	if (write(socket_fd, "MPX,1,\r\n", 8) <= 0) {
	  printf("Error writing back to client.\n");
	  return EXIT_FAILURE;
	}
      }
      
      nleft = MAXLINE;
     
      //Clear buffer for next command.
      memset(&buffer[0], '\0', sizeof(buffer));
      memset(&tempbuf[0], '\0', sizeof(tempbuf));
      
    }
    
  }

  printf("Buffer full. From client: %s\n", buffer);

  return(EXIT_SUCCESS);
}
