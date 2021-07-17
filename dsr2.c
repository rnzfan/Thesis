/*
 * Simulation program for Dynamic Source Routing Protocol
 * written in CSIM 
 *
 * @Modified date: 7/5/2005
 * @Author: Fan Tsz Chuen Roy
 * @SID: 02729143
 * 
 * Final Year Project
 * CUHK Information Engineering
 *
 */


/* assume all the time unit = 1ms */
#include <csim.h>

#define LOSTPROB 0.01				 /* lost prob. */
#define MAXREPOLLTIME 3				 /* maximum repolling times */
#define NUMBEROFNODE 17              /* total number of nodes including the master */

#define REQUEST_TIME 2.0					/* request's packet time */
#define REPLY_TIME 4.0						/* reply's packet time */
#define SENSOR_COMPUTATION_TIME 30.0		/* sensor reply computation time */
#define MASTER_COMPUTATION_TIME 10.0		/* master polling computation time */
#define PROPAGATION_DELAY 0.01				/* propagation delay */
#define TIMEOUT 150.0						/* time out value */
#define FLOODING_TIME 30.0                  /* flooding's packet time */
#define WAITTING_TIME 2						/* Waitting time when sending flooding packet */
#define ONE_TIMESLOT 10.0                   /* Time for one time slot */

int table[NUMBEROFNODE][2][NUMBEROFNODE];        /* routing tables */
int topology[NUMBEROFNODE][NUMBEROFNODE];		 /* network topology */

/* for statistics */
int polling_time = 0;
int tout_time = 0;
int begin_findnewroute[NUMBEROFNODE];
int end_findnewroute[NUMBEROFNODE];
int maxtime_findnewroute = 0;
int totaltime_findnewroute = 0;
int totalfreq_findnewroute = 0;
int totalfreq_collision = 0;

/* initialize the function protocol */
void packet(int);
int polling(int, int);
int getnextnode(int, int);
void topologies(int);
int connect(int, int);
void deleteroute(int, int);
int flooding(int, int);
void printonetable(int);
int changetopology(int);

FACILITY n;                    /* Network */

sim() {
	int i, j;
	int out;
	int repoll_time;				  /* repolling times */
	int start_time;
	long change;                      /* changing topology */
	int stime;
	int oldtopology = 0;
	int newtopology;

	reseed(NIL, 34024);               /* reseed a new sequence once starting the program */
	inittable();                      /* initialize the routing tables */
	topologies(oldtopology);	  /* topologies: type 0 */

	create("sim");                    /* make this a process */
	n = facility("network");  

	while (simtime() < 50000.0)       /* simulation time */
	{
		for (i=1; i<NUMBEROFNODE; i++ )		  /* from sensor S1 to sensor S16, total 16 sensors */
		{
			repoll_time = 0;
			out = 0;                  /* out = false, haven't finish polling on particular sensor */
			
			while ((out == 0 || out == -1) && repoll_time < MAXREPOLLTIME )
			{
				start_time = simtime();   /* get the current time */
				
				out = polling(0, i);
				polling_time++;
				printf("polling time: %d\n\n", polling_time);

				if (out == 0)
				{
					printf("~~~~ TIME OUT ~~~~\n");
				}

				if ((out == 0 || out == -1) && (simtime() - start_time < TIMEOUT))
				{
					printf("~~~~ HOLD NOW ~~~~\n");
					hold(TIMEOUT - (simtime() - start_time));  /* hold until reach the timeout value */				
					tout_time++;
				}
				repoll_time++;
			}

			
			/* delete outdated routes in the routing tables when repoll times = 3 */
			if (repoll_time == 3)
			{
				begin_findnewroute[i] = simtime();
				printf("\nBegin reestablish[%d]: %d\n\n", i, begin_findnewroute);
				deleteroute(0, i);
			}
			
			change = random_int(1, 50);
			/* prob. to chang topology on each polling = 1/50 */
			if (change == 1)
			{
				newtopology = changetopology(oldtopology);
				topologies(0);
				topologies(newtopology);  /* change the currnet topology */
				printf("<< New topology: %d >>\n", newtopology);
				oldtopology = newtopology;
			
				stime = simtime();
				printf("simtime: %d\n\n", stime);
				reseed(NIL, stime);
			}

		}
	}
	genreport();                   /* print suitable statistics in the report */
	printf("\n\n");
	report();
	terminate();
}

int polling(int source, int dest) {
	double ramdom_value;
	int nextnode;
	int nownode;

	printf("Polling on [ %d ]\n", dest);
	nownode = source;
	do
	{	
		nextnode = getnextnode(nownode, dest);

		/* if there is no route to dest, use flooding to find route */
		if (nextnode == -1)
		{
			nextnode = flooding(nownode, dest);
			if (nextnode == -1)    /* flooding is not success */
			{
				return -2;
			}
		} 

		/* there is no connection between nownode and nextnode */
		if (connect(nownode, nextnode) == 0)
		{
			printf("\n~~~~ No real connection~~~~\n\n");
			return -1;
		}

		ramdom_value = uniform(0.0, 1.0);
		if (ramdom_value > LOSTPROB)
		{
			printf("Send request: \n");
			packet(source);			/* send request from Master to other */
		} else {
			return 0;
		}
		nownode = nextnode;
		
		hold(SENSOR_COMPUTATION_TIME);  /* wait for the computation time in the sensor */
		printf("Sensor computing ...\n");	
	}
	while (nownode != dest);

	do
	{
		nextnode = getnextnode(nownode, source);

		/* if there is no route to source, use flooding to find route */
		if (nextnode == -1)
		{
			nextnode = flooding(nownode, source);
			if (nextnode == -1)    /* flooding is not success */
			{
				return -2;
			}
		} 

		/* there is no connection between nownode and nextnode */
		if (connect(nownode, nextnode) == 0)
		{
			printf("\n~~~~ No real connection~~~~\n\n");
			return -1;
		}

		ramdom_value = uniform(0.0, 1.0);
		if (ramdom_value > LOSTPROB)
		{
			printf("Reply: \n");
			packet(dest);			/* send reply from other to Master */
		} else {
			return 0;
		}		
		nownode = nextnode;

		if (nownode != source)
		{
			hold(SENSOR_COMPUTATION_TIME);  /* wait for the computation time in the sensor */
			printf("Sensor computing ...\n");
		} else {
			hold(MASTER_COMPUTATION_TIME);  /* wait for the computation time in the master */
			printf("Master computing ...\n");
		}
	}
	while (nownode != source);
	
	return 1;
}

void packet(int source) {             

	if (source == 0)				/* from Master to other */
	{
		reserve(n);								 /* using the network, other need to wait */
		hold(REQUEST_TIME + PROPAGATION_DELAY);  /* wait for request transmit time */
		printf("M --> \n");
		release(n);								 /* release the network */
	}
	else							/* from other to Master */
	{
		reserve(n);								 /* using the network, other need to wait */
		hold(REPLY_TIME + PROPAGATION_DELAY);    /* wait for reply transmit time */
		printf("S ==> \n");
		release(n);								 /* release the network */
	}

}

int getnextnode(int nownode, int dest) {
	int i;
	int nextnode = -1;

	for (i=0 ; i<NUMBEROFNODE ;i++ )
	{
		if (table[nownode][0][i] == dest)
		{
			/* get the nextnode value in suitable row of the suitable table */
			nextnode = table[nownode][1][i];
			printf("nownode: %d ; next node: %d\n", nownode, nextnode);
			break;
		} 
	}
	return nextnode;
}

/* use flooding to find whether the immediate nextnode have a route to destination */
int flooding(int nownode, int target) {
	int i, j;
	int temp;
	int temp_time;
	int temphop1 = -1;
	int temphop2 = -1;
	long temphop1_wait = -1;
	long temphop2_wait = -1;
	int temphopOK = -1;
	
	printf("\nFlooding: %d <-> %d\n\n", nownode, target);
	if (connect(nownode, target) == 1) /* if there is a connection between them */
	{
		printf("Flooding OK, next node: %d\n", target);
		table[nownode][1][target] = target;  /* update the routing table of nownode */
		table[target][1][nownode] = nownode; /* update the routing table of target */

		hold((FLOODING_TIME + REQUEST_TIME)*2);  /* wait for the transmission and computing time */

		printonetable(nownode);
		printonetable(target);
		return target;
	} else {
		for (i=0 ; i<NUMBEROFNODE ;i++ )
		{
			temp = table[nownode][1][i];
			if (temp != -1)
			{
				for (j=1 ; j<NUMBEROFNODE ;j++ )
				{
					if (topology[j][temp] == target)
					{
						if (temphop2 == -1)
						{
							temphop1 = temp;
							temphop2 = temp;
							break;
						}
						temphop2 = temp;
						break;

					}
				}
			}
		}
		if (temphop1 != temphop2)
		{
			/* waiting time before send flooding packet (collision occur) */
			temphop1_wait = random_int(0, WAITTING_TIME);  /* waitting time */
			temphop2_wait = random_int(0, WAITTING_TIME);

			printf("Potential colli.: %d(%d)->%d<-%d(%d)\n", temphop1, temphop1_wait, target, temphop2, temphop2_wait);

			/* hop who has min. waiting time will go first */
			if (temphop1_wait > temphop2_wait)
			{
				temphopOK = temphop2;
				hold(ONE_TIMESLOT * temphop2_wait);   /* wait for some timeslots' time */
			} else {
				temphopOK = temphop1;
				hold(ONE_TIMESLOT * temphop1_wait);
			}
		} else {  /* no potential collision */
			if (temphop1 == -1) /* flooding dose not succeed */
			{
				printf("Flooding does not succeed\n");
				return -1;
			}
			temphop1_wait = random_int(0, WAITTING_TIME);  /* waitting time */
			hold(ONE_TIMESLOT * temphop1_wait);
			temphopOK = temphop1;  
		}

		/* collision occur, give up this polling */
		if (temphop1_wait == temphop2_wait)
		{
			totalfreq_collision++;
			printf("Collision occur: %d\n", totalfreq_collision);
			printf("Flooding does not succeed\n");
			return -1;
		}
		
		printf("Flooding OK, next node: %d\n", temphopOK);
		table[nownode][1][target] = temphopOK;  /* update the routing table of nownode */
		table[temphopOK][1][nownode] = nownode; /* update the routing table of temp */
		table[temphopOK][1][target] = target;
		table[target][1][nownode] = temphopOK;  /* update the routing table of target */
		table[target][1][temphopOK] = temphopOK;

		hold((FLOODING_TIME + REQUEST_TIME)*2);  /* wait for the transmission and computing time */
		hold((FLOODING_TIME + REQUEST_TIME)*2);  /* wait for the transmission and computing time */

		printonetable(nownode);
		printonetable(temphopOK);
		printonetable(target);
		
		/* caculate the time to find a new route */
		end_findnewroute[target] = simtime();
		printf("\nEnd reestablish[%d]: %d\n",target, end_findnewroute);
		temp_time = (end_findnewroute[target] - begin_findnewroute[target]);
		/* choose the max re-establish time */
		if (temp_time > maxtime_findnewroute)
		{
			maxtime_findnewroute = temp_time;
		}
		totaltime_findnewroute += temp_time;
		totalfreq_findnewroute++;
		printf("Max reestablish(freq): %d(%d)\n\n", maxtime_findnewroute, totalfreq_findnewroute);

		return temphopOK;
	}
}

int connect(int nownode, int nextnode) {
	int i;

	for (i=1 ;i<NUMBEROFNODE ;i++ )
	{
		/* check if there is a connection between nownode and nextnode */
		if (topology[i][nownode] == nextnode)
		{
			return 1;  /* 1 means there is a connection */
		}
	}
	return 0;
}



/* intialize each routing table */
inittable() {
	int i, j;
	printf("intialize the routing table\n");

	/* intialize the master's routing table */
	for (i=1 ;i<NUMBEROFNODE ;i++)
	{
		table[0][0][i] = i;
		table[0][1][i] = i;
	}

	/* intialize sensors' routing table */
	for (i=1 ;i<NUMBEROFNODE ;i++ )
	{
		for (j=0 ;j<NUMBEROFNODE ;j++ )
		{
			table[i][0][j] = j;
			table[i][1][j] = -1;
		}
		table[i][1][0] = 0;
	}

	/* print table */
	printtable();

	printf("\n");
}

printtable() {
	int i, j;

	/* print table */
	printf("table: Master\n");
	for (j=0 ;j<NUMBEROFNODE ;j++ )
	{
		printf("%d | %d\n", table[0][0][j], table[0][1][j]);
	}
	printf("\n");
	for (i=1 ;i<NUMBEROFNODE ;i++ )
	{
		printf("table: S%d\n", i);
		for (j=0 ;j<NUMBEROFNODE ;j++ )
		{
			printf("%d | %d\n", table[i][0][j], table[i][1][j]);
		}
		printf("\n");
	}
}

/* print a particular table */
void printonetable(int temp) {
	int j;
	printf("table: S%d\n", temp);
	for (j=0 ;j<NUMBEROFNODE ;j++ )
	{
		printf("%d | %d\n", table[temp][0][j], table[temp][1][j]);
	}
	printf("\n");
}

/* different topologies */
void topologies(int type) {
	int i, j;
	switch (type)
	{
		case 0:  
		{
			/* all become minus one, indicate there is no connection*/
			for (i=0 ;i<NUMBEROFNODE ;i++ )
			{
				for (j=0 ;j<NUMBEROFNODE ;j++ )
				{
					topology[i][j] = -1;
				}
			}

			for (i=0 ;i<NUMBEROFNODE ;i++ )
			{
				topology[0][i] = i;  
				topology[i][0] = i;  /* M is connected to all hops */
			}
			for (i=1 ;i<NUMBEROFNODE ;i++ )
			{
				topology[1][i] = 0;  /* all hops is connected to M */ 
			}
			break;
		}

		case 1:
		{
			topology[1][0] = -1;  /* delete S1 in row M */
			topology[1][1] = 2;   /* S2 replaces M in row S1 */
			topology[2][2] = 1;   /* add S1 in row S2 */
			break;
		}
		case 2:
		{	
			topology[1][0] = -1;
			topology[3][0] = -1;
			topology[5][0] = -1;
			topology[1][1] = 2;
			topology[1][3] = 2;
			topology[2][2] = 1;
			topology[3][2] = 3;
			topology[1][5] = 6;
			topology[2][6] = 5;
			break;
		}
		case 3:
		{
			topology[1][0] = -1;
			topology[5][0] = -1;
			topology[1][1] = 2;
			topology[2][2] = 1;
			topology[1][5] = 6;
			topology[2][6] = 5;
			break;
		}
		case 4:
		{
			topology[1][0] = -1;
			topology[5][0] = -1;
			topology[1][1] = 2;
			topology[2][2] = 1;
			topology[1][5] = 6;
			topology[2][6] = 5;
			topology[8][0] = -1;
			topology[1][8] = 7;
			topology[2][8] = 9;
			topology[2][7] = 8;
			topology[2][9] = 8;
			break;
		}
		case 5:
		{
			topology[1][0] = -1;
			topology[4][0] = -1;
			topology[5][0] = -1;
			topology[13][0] = -1;
			topology[1][1] = 2;
			topology[2][2] = 1;
			topology[1][4] = 6;
			topology[1][5] = 6;
			topology[2][6] = 4;
			topology[3][6] = 5;
			topology[1][13] = 12;
			topology[2][13] = 14;
			topology[2][12] = 13;
			topology[2][14] = 13;
			break;
		}
		case 6:
		{
			topology[1][0] = -1;
			topology[4][0] = -1;
			topology[5][0] = -1;
			topology[7][0] = -1;
			topology[13][0] = -1;
			topology[1][1] = 2;
			topology[2][2] = 1;
			topology[1][4] = 3;
			topology[2][4] = 6;
			topology[2][3] = 4;
			topology[2][6] = 4;
			topology[1][5] = 6;
			topology[3][6] = 5;
			topology[1][7] = 6;
			topology[4][6] = 7;
			topology[1][13] = 12;
			topology[2][12] = 13;
			break;
		}
		case 7:
		{
			topology[1][0] = -1;
			topology[4][0] = -1;
			topology[5][0] = -1;
			topology[7][0] = -1;
			topology[13][0] = -1;
			topology[1][1] = 2;
			topology[2][2] = 1;
			topology[1][4] = 3;
			topology[2][4] = 6;
			topology[2][3] = 4;
			topology[2][6] = 4;
			topology[1][5] = 6;
			topology[3][6] = 5;
			topology[1][7] = 6;
			topology[4][6] = 7;
			topology[1][13] = 12;
			topology[2][12] = 13;
			topology[16][0] = -1;
			topology[14][0] = -1;
			topology[1][16] = 2;
			topology[3][2] = 16;
			topology[1][14] = 12;
			topology[3][12] = 14;
			break;
		}
		case 8:
		{
			topology[1][0] = -1;
			topology[4][0] = -1;
			topology[5][0] = -1;
			topology[13][0] = -1;
			topology[1][1] = 2;
			topology[2][1] = 16;
			topology[2][2] = 1;
			topology[2][16] = 1;
			topology[1][4] = 6;
			topology[1][5] = 6;
			topology[2][6] = 4;
			topology[3][6] = 5;
			topology[1][13] = 12;
			topology[2][12] = 13;
			break;
		}
		case 9:
		{
			topology[1][0] = -1;
			topology[5][0] = -1;
			topology[13][0] = -1;
			topology[1][1] = 16;
			topology[2][16] = 1;
			topology[1][5] = 6;
			topology[2][6] = 5;
			topology[1][13] = 12;
			topology[2][12] = 13;
			break;
		}
		case 11: /* use for testing */
		{
			topology[1][0] = -1;
			topology[1][1] = 2;
			topology[2][1] = 3;
			topology[2][2] = 1;
			topology[2][3] = 1;
			break;
		}
	}
}

/* delete the routes between hop a and hop b in a's routing table */
void deleteroute(int a, int b) {
	int j;
	
	for (j=0 ;j<NUMBEROFNODE ;j++ )
	{
		if (table[a][0][j] == b)
		{
			table[a][1][j] = -1;  /* delete the outdated route in routing table of hop a */
		}
	}
	printf("Del entry %d\n in routing table: S%d\n", b, a);
	printonetable(a);

}

/* input the previous topology, output the next topology */
int changetopology(int old) {
	long prob;
	if (old == 2)
	{
		prob = random_int(0, 1);
		if (prob == 0)
		{
			return 3;
		} else {
			return 4;
		}
	}
	if (old == 9)
	{
		prob = random_int(0, 2);
		if (prob == 0)
		{
			return 0;
		} else if (prob == 1)
		{
			return 2;
		} else {
			return 5;
		}
	}
	return (old + 1);
}

genreport() {
	int k;
	printf("The average time-out probability is: %.6f\n", (double)tout_time/polling_time);
	printf("The max time to reestablish: %.2fmsec\n", (double)maxtime_findnewroute);
	printf("The average time to reestablish(freq): %.2fmsec(%d)\n", (double)totaltime_findnewroute/totalfreq_findnewroute,totalfreq_findnewroute);
	printf("Total number of collisions: %d\n", totalfreq_collision);

	/* print all the waitting time slot */
	printf("Waiting time slot: [");
	for (k=0; k <= WAITTING_TIME ; k++ )
	{
		printf(" %d", k);
	}
	printf(" ]\n");
}


