/**
 * Simple TCP server to simulate a medipix Labview system.
 * Arguments:
 *   port number - port number to listen for connections
 * 
 * Matthew Pearson
 * Oct 2011
 *
 * Giles Knap
 * Jan 2012
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
#define MAXDATA 256*256*2 // 256 X by 256 Y by 2 bytes per pixel
#define DATAHEADERLEN 256
#define CMDLEN 4
#define HEADER_LEN 15 // this includes 2 commas + the header and length fields

/*Function prototypes.*/
void sig_chld(int signo);
int echo_request(int socket_fd);
int produce_data(int socket_fd);
void *commandThread(void* command_fd);
void *dataThread(void* data_fd);
int produce_data(int data_fd);

int data_exit = 0;
int do_data = 0;
pthread_mutex_t do_data_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t do_data_cond = PTHREAD_COND_INITIALIZER;

int main(int argc, char *argv[])
{
	int fd, fd2, fd_data, fd2_data;
	socklen_t client_size;
	struct sockaddr_in server_addr, client_addr, server_addr_data,
			client_addr_data;

	pthread_t tid, tid_data;

	printf("Started Medipix simulation server...\n");

	if (argc != 3)
	{
		printf("  ERROR: Use: %s {command socket} {data socket}\n", argv[0]);
		exit(EXIT_FAILURE);
	}

	/* Create a TCP socket.*/
	fd = socket(AF_INET, SOCK_STREAM, 0);
	fd_data = socket(AF_INET, SOCK_STREAM, 0);

	/* Create and initialise a socket address structure.*/
	memset(&server_addr, 0, sizeof(struct sockaddr_in));
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	server_addr.sin_port = htons(atoi(argv[1]));

	memset(&server_addr_data, 0, sizeof(struct sockaddr_in));
	server_addr_data.sin_family = AF_INET;
	server_addr_data.sin_addr.s_addr = htonl(INADDR_ANY);
	server_addr_data.sin_port = htons(atoi(argv[2]));

	/* Bind the socket address to the socket buffer.*/
	bind(fd, (struct sockaddr *) &server_addr, sizeof(server_addr));
	bind(fd_data, (struct sockaddr *) &server_addr_data,
			sizeof(server_addr_data));

	/* data socket timeout	 */
	struct timeval timeout;
	    timeout.tv_sec = 2;
	    timeout.tv_usec = 0;

	setsockopt (fd_data, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout, sizeof(timeout));

	/* Listen for incoming connections. */
	listen(fd, 10);
	listen(fd_data, 10);

	/* Install signal handler to clean up children.*/
	//  signal(SIGCHLD, sig_chld);
	/*Loop forever.*/
	while (1)
	{

		/*Command socket*/
		printf("Waiting for command socket...\n");
		client_size = sizeof(client_addr);
		if ((fd2 = accept(fd, (struct sockaddr *) &client_addr, &client_size))
				< 0)
		{
			if (errno == EINTR)
			{
				continue; /*Deal with interupted system call, since this blocks.*/
			}
			else
			{
				perror(argv[0]);
				exit(EXIT_FAILURE);
			}
		}

		if (fd2 > 0)
		{

			/*Data socket*/
			printf("Waiting for data socket...\n");
			client_size = sizeof(client_addr_data);
			if ((fd2_data = accept(fd_data,
					(struct sockaddr *) &client_addr_data, &client_size)) < 0)
			{
				if (errno == EINTR)
				{
					continue; /*Deal with interupted system call, since this blocks.*/
				}
				else
				{
					perror(argv[0]);
					exit(EXIT_FAILURE);
				}
			}

			if (fd2_data > 0)
			{
				pthread_create(&tid, NULL, &commandThread, (void *) fd2);
				pthread_create(&tid_data, NULL, &dataThread, (void *) fd2_data);

				/*Block here waiting for the threads to finish. This only allows a single client to
				 connect, to keep it simple.*/
				pthread_join(tid, NULL);
				printf("thread 1 finished.\n");
				pthread_join(tid_data, NULL);
				printf("thread 2 finished.\n");
			}
			else
			{
				close(fd2);
			}
		}

	}

	/*Should never get here.*/
	printf("Finishing Medipix server.\n");
	return EXIT_SUCCESS;

}

void * commandThread(void *command_fd)
{
	printf("Started commandThread.\n");

	if (echo_request((int) command_fd) != EXIT_SUCCESS)
	{
		printf("  Client failed to handle protocol, or connection closed.\n");
		/*close connected socket*/
		close((int) command_fd);
		/*signal data thread to exit.*/
		printf("***signalling data thread to exit.\n");
		pthread_mutex_lock(&do_data_mutex);
		do_data = 1;
		data_exit = 1;
		pthread_cond_signal(&do_data_cond);
		pthread_mutex_unlock(&do_data_mutex);
		return (void *) EXIT_FAILURE;
	}
	return (void *) EXIT_SUCCESS;
}

void * dataThread(void *data_fd)
{
	printf("Started dataThread.\n");

	if (produce_data((int) data_fd) != EXIT_SUCCESS)
	{
		printf(
				"  Data client failed to handle protocol, or connection closed.\n");
		/*close connected socket*/
		close((int) data_fd);
		return (void *) EXIT_FAILURE;
	}
	return (void *) EXIT_SUCCESS;
}

/**
 * Signal handler for SIGCHLD. NOT NEEDED NOW WE ARE USING PTHREADS.
 *
 */
void sig_chld(int signo)
{
	pid_t pid = 0;
	int status = 1;

	while ((pid = waitpid(-1, &status, WNOHANG)) > 0)
	{
		printf("Child %d has terminated. Status = %d\n", pid, status);
	}
}

/**
 * Read from socket and send back response. 
 * Read a maximum of MAXLINE, or until a newline, then echo the command back.
 */
int echo_request(int socket_fd)
{
	int nleft = HEADER_LEN;
	int nread = 0;
	char buffer[MAXLINE + 1] =
	{	'\0'};char
	response[MAXLINE] =
	{ '\0' };
	char *bptr = NULL;
	int inheader = 1;
	char *tok = NULL;
	char *cmdType, *cmdName;
	int bodylen = 0;

	bptr = buffer;

	if (socket_fd == 0)
	{
		printf("NULL socket fd in echo_request.\n");
		return EXIT_FAILURE;
	}

	// keep reading and responding until an error occurs
	while (1)
	{
		// clear response
		response[0] = (char) NULL;
		// first time around following loop we read header_len bytes
		nleft = HEADER_LEN;
		// first pass of the while loop is reading header
		inheader = 1;

		/// Read until end of current block (header or body)
		while (nleft > 0)
		{
			printf("reading socket...\n");
			if ((nread = read(socket_fd, bptr, nleft)) < 0)
			{
				if (errno == EINTR)
				{
					nread = 0;
					printf("EINTR");
				}
				else
				{
					printf("Unknown read error.\n");
					return EXIT_FAILURE;
				}
			}
			else if (nread == 0)
			{
				printf("Read nothing. Socket closed.\n");
				return EXIT_FAILURE; //Done. Client has probably disconnected.
			}

			nleft = nleft - nread;
			if (inheader)
			{
				if (nleft == 0)
				{
					inheader = 0;
					bptr[HEADER_LEN] = (char) NULL;

					printf("received command header: %s\n", buffer);

					nleft = MAXLINE - HEADER_LEN;
					tok = strtok(bptr, ",");
					tok = strtok(NULL, ",");
					if (tok == NULL)
					{
						printf("MPX Command header is missing length field.\n");
						continue;
					}
					else
					{
						// subtract 1 from bodylen since we already read the 1st comma
						nleft = bodylen = atoi(tok) - 1;
					}
				}
			}
			else
			{
			}

		}

		bptr[bodylen] = (char) NULL;
		printf("received command body: %s\n", buffer);

		//Handle command
		bptr = buffer;

		cmdType = strtok(bptr, ",");
		cmdName = strtok(NULL, ",");
		if (cmdType == NULL || cmdName == NULL)
		{
			printf("badly formed MPX command\n");
			sprintf(response, "MPX,0000000008,ERROR,1");
			continue;
		}

		if (!strncmp(cmdType, "SET", 3))
		{
			bodylen = strlen(cmdName) + 7;
			sprintf(response, "MPX,%010u,SET,%s,0", bodylen, cmdName);
		}
		else if (!strncmp(cmdType, "GET", 3))
		{
			if(!strncmp(cmdName,"DETECTORSTATUS",MAXLINE))
			{
				bodylen = strlen(cmdName) + 9;
				sprintf(response, "MPX,%010u,GET,%s,0,0", bodylen, cmdName);
			}
			if(!strncmp(cmdName,"GETSOFTWAREVERSION",MAXLINE))
			{
				bodylen = strlen(cmdName) + 11;
				sprintf(response, "MPX,%010u,GET,%s,0.1,0", bodylen, cmdName);
			}
			else
			{
				// default response
				bodylen = strlen(cmdName) + 11;
				sprintf(response, "MPX,%010u,GET,%s,9.0,0", bodylen, cmdName);
			}
		}
		else if (!strncmp(cmdType, "CMD", 3))
		{
			if (!strncmp(cmdName, "STARTACQUISITION", 16))
			{
				strncpy(response, "MPX,0000000023,CMD,STARTACQUISITION,0", MAXLINE);
				/*signal data thread to send some data back.*/
				printf("***signalling data thread.\n");
				pthread_mutex_lock(&do_data_mutex);
				do_data = 1;
				pthread_cond_signal(&do_data_cond);
				pthread_mutex_unlock(&do_data_mutex);
				printf("***data thread singnalled.\n");
			}
		}
		else
		{
			printf("Unknown MPX command: '%s'\n", cmdName);
			sprintf(response, "MPX,0000000008,ERROR,1");
		}


		// deliberately send junk to test re-synch capability
		printf("sending garbage..\n");
		if(write(socket_fd, "garbage garbage", 15) <=0)
			printf("garbage 1 failed\n");
		if(write(socket_fd, "MPX,0000000023,XXX,STARTACQUISITION,0",37 ) <=0)
			printf("garbage 2 failed\n");

		printf("sending response: %s \n", response);
		if (write(socket_fd, response, strlen(response)) <= 0)
		{
			printf("Error writing back to client.\n");
			return EXIT_FAILURE;
		}

		//Clear buffer for next command.
		memset(&buffer[0], '\0', sizeof(buffer));
	}

	printf("Buffer full. From client: %s\n", buffer);

	return EXIT_SUCCESS;
}

/**
 * Produce simulation data when we start acquisition. 
 */
int produce_data(int data_fd)
{
	//char *data = "Here is some data.\r\n";
	//unsigned int trailer = 0xDA; /* CR LF */

	// frame = header (including comma) + data frame type (3 chars) +
	//		comma + data frame header + image data
	int frameSize = HEADER_LEN + CMDLEN + 1 + DATAHEADERLEN + MAXDATA;
    char data[frameSize];
	unsigned int i;
	int headersLength = HEADER_LEN + CMDLEN + DATAHEADERLEN;

	snprintf(data, HEADER_LEN,"MPX,%010u,", frameSize - HEADER_LEN + 1);
	sprintf((data + HEADER_LEN), "12B,%-256s",
			"1,1,2012-02-01 11:26:00.000,.05,6.0,8.0,0,0,0,0,0,0,0,0");

	// create dummy data
	for (i = headersLength; i < MAXDATA+headersLength; i++)
	{
		data[i] = (i % 255) & 0xFF;
		//printf("data[%d]: %x\n", i, data[i] & 0xFF);
	}

	while (1)
	{
		printf("***taking mutex in data thread.\n");
		/*Wait for signal to produce some data.*/
		pthread_mutex_lock(&do_data_mutex);
		while (do_data == 0)
		{
			printf("***waiting in data thread.\n");
			pthread_cond_wait(&do_data_cond, &do_data_mutex);
		}

		printf("***got signal in data thread.\n");

		//printf("data_fd: %d\n", data_fd);
		//printf("data_exit: %d\n", data_exit);
		//printf("do_data: %d\n", do_data);

		//printf("MAXDATA: %d\n", MAXDATA);

		if ((do_data == 1) && (data_exit == 0))
		{

			// send a silly acquisition header
			if (write(data_fd, "MPX,0000000030,HDR,dummy acquisition header.", 30 + HEADER_LEN -1) <= 0)
			{
				printf("Error writing acquisition header to client.\n");
				do_data = 0;
				pthread_mutex_unlock(&do_data_mutex);
				//return EXIT_FAILURE;
			}
			printf("*** wrote data - acquisition header\n");

			// write an image
			if (write(data_fd, data, frameSize) <= 0)
			{
				printf("Error writing data frame 1 to client.\n");
				do_data = 0;
				pthread_mutex_unlock(&do_data_mutex);
				//return EXIT_FAILURE;
			}
			printf("*** wrote data - image one \n");

			// write a second image
			if (write(data_fd, data, frameSize) <= 0)
			{
				printf("Error writing data frame 2 to client.\n");
				do_data = 0;
				pthread_mutex_unlock(&do_data_mutex);
				//return EXIT_FAILURE;
			}
			printf("*** wrote data - image two\n");

			do_data = 0;
		}
		else
		{
			if (data_exit)
			{
				printf("Exiting data thread.\n");
				data_exit = 0;
				do_data = 0;
				pthread_mutex_unlock(&do_data_mutex);
				break;
			}
		}

		pthread_mutex_unlock(&do_data_mutex);

	}
	pthread_mutex_unlock(&do_data_mutex);

	return EXIT_SUCCESS;
}
