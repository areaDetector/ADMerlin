
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
#include <pthread.h>

#define MAXLINE 256

/*Function prototypes.*/
void sig_chld(int signo);
int echo_request(int socket_fd);
void *commandThread(void* command_fd);


int main(int argc, char *argv[])
{
  int fd, fd2, fd_data, fd2_data; 
  socklen_t client_size;
  pid_t child_pid = 0;
  struct sockaddr_in server_addr, client_addr, server_addr_data, client_addr_data;
  char *buff = NULL;

  pthread_t tid;

  printf("Started Medipix simulation server...\n");

  if (argc != 3) {
    printf("  ERROR: Use: %s {command socket} {data socket}\n", argv[0]);
    exit(EXIT_FAILURE);
  }
  
  /* Create a TCP socket.*/
  fd = socket(AF_INET, SOCK_STREAM, 0);
  //fd_data = socket(AF_INET, SOCK_STREAM, 0);

  /* Create and initialise a socket address structure.*/
  memset(&server_addr, 0, sizeof(struct sockaddr_in));
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  server_addr.sin_port = htons(atoi(argv[1]));

  //memset(&server_addr_data, 0, sizeof(struct sockaddr_in));
  //server_addr.sin_family = AF_INET;
  //server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  //server_addr.sin_port = htons(atoi(argv[2]));

  /* Bind the socket address to the socket buffer.*/
  bind(fd, (struct sockaddr *) &server_addr, sizeof(server_addr));
  //bind(fd_data, (struct sockaddr *) &server_addr_data, sizeof(server_addr_data));

  /* Listen for incoming connections. */
  listen(fd, 10);
  //listen(fd_data, 10);

  /* Install signal handler to clean up children.*/
  signal(SIGCHLD, sig_chld);

  /*Loop forever.*/
  while(1) {

    /*Command socket*/
    printf("Connecting command socket...\n");
    client_size = sizeof(client_addr);
    if ((fd2 = accept(fd, (struct sockaddr *) &client_addr, &client_size)) < 0) {
      if (errno == EINTR) {
	continue; /*Deal with interupted system call, since this blocks.*/
      } else {
	perror(argv[0]);
	exit(EXIT_FAILURE);
      }
    }

    /*Data socket*/
    //printf("Connecting data socket...\n");
    //client_size = sizeof(client_addr_data);
    //if ((fd2_data = accept(fd_data, (struct sockaddr *) &client_addr_data, &client_size)) < 0) {
    //  if (errno == EINTR) {
    //continue; /*Deal with interupted system call, since this blocks.*/
    // } else {
    //perror(argv[0]);
    //exit(EXIT_FAILURE);
    //}
    //}

    pthread_create(&tid, NULL, &commandThread, (void *) fd2);


    /*Deal with commands in a new thread.*/
    //if ((child_pid = fork()) == 0) {
      /*Child process.*/
      //printf("In child...\n");
      //close(fd);
      //while(1) { /*Keep reading until connection is closed or there is an error.*/
      //if (echo_request(fd2) != EXIT_SUCCESS) {
    //	  perror(argv[0]);
    //  printf("  Client failed to handle protocol, or connection closed.\n");
    //  exit(EXIT_FAILURE);
    //}
    //}
    //exit(EXIT_SUCCESS);
    //}

    /*Parent process.*/
    //close(fd2);
    //close(fd2_data);
  }

  /*Should never get here.*/
  printf("Finishing Medipix server.\n");
  return EXIT_SUCCESS;

}


void * commandThread(void *command_fd)
{
  /*No need to check status of this thread.*/
  pthread_detach(pthread_self());
  printf("Started commandThread.\n");
  while (1) {
    if (echo_request((int)command_fd) != EXIT_SUCCESS) {
      printf("  Client failed to handle protocol, or connection closed.\n");
      /*close connected socket*/
      close((int) command_fd);
      return EXIT_FAILURE;
    }
  }
  
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
    printf("NULL sokcet fd in echo_request.\n");
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
    
    printf("nread: %d\n", nread);
    printf("We got: %s\n", bptr);

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
    strncpy(response, "MPX,1\r\n", MAXLINE);


    //Handle command
    if (command == 1) {
      bptr = &buffer;
      strncpy(tempbuf, buffer, MAXLINE);
      
      /*Remove the terminator.*/
      tok = strtok(tempbuf, "\r\n");
      //printf("tok: %s\n", tok);
      tok = strtok(tok, ",");
      //printf("tok: %s\n", tok);
      if (!strncmp(tok,"MPX",3)) {
	tok = strtok(NULL, ",");
	//printf("tok: %s\n", tok);
	if (!strncmp(tok,"SET",3)) {
	  //printf("tok: %s\n", tok);
	  tok = strtok(NULL, ",");
	  strncpy(response,"MPX,0\r\n",MAXLINE);
	}
	else if (!strncmp(tok,"GET",3)) {
	  //printf("tok: %s\n", tok);
	  tok = strtok(NULL, ",");
	  //printf("tok: %s\n", tok);
	  if (tok != NULL) {
	    sprintf(response, "MPX,0,%s,5\r\n", tok);
	  } else {
	    if (write(socket_fd, "MPX,1\r\n", 8) <= 0) {
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
	if (write(socket_fd, "MPX,1\r\n", 8) <= 0) {
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
