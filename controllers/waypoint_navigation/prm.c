#include<stdio.h>
#include<stdlib.h>
#include<math.h>
double n=500,p,coordinate, Kx[5000],Ky[5000],dist[5000],a=-1,b=1;
int vertexCounter=0;
int edgeCounter=0;

struct vertex
{
	double Vx;
	double Vz;
	int visited;
	struct vertex* previous;
};
struct vertex vertexset[5000];

struct edge
{
	struct vertex v1;
	struct vertex v2; 
};
struct edge edgeset[13000];

struct dist
{
	double dbp; //distance between points
	int drefv; //what vertex we are at while finding distance
};

double uniformRandomNumber(double a, double b)
{
	return (((double)rand())/(double)RAND_MAX)*(b-a)+a;
}
double collisioncheck(double x, double z, int workspace[33][33])  //it checks to see if the point is in the blocked cell
{
	int c1=floor((x+1)/0.07);
	int c2=floor((z+1)/0.07);
  //printf("collision check (%d, %d)\n", c1, c2);
	if(workspace[c1][c2]==1)
		return 1;
	else
		return 0;
}


void sort(struct dist dist[],int vertexcounter)
{
	struct dist t;
	int c,d;
	for(c=0;c<(vertexcounter-1);c++)
	{ 
		for(d=0;d<vertexcounter-c-1;d++)
		{
			if(dist[d].dbp>dist[d+1].dbp)
			{
				t= dist[d];
				dist[d]=dist[d+1];
				dist[d+1]=t;

			}  
		}
	}
}
int contains(struct edge* edgeset,struct vertex q,struct vertex qPrime)
{
  //printf("contains\n");
  int i;
	for(i=0;i<edgeCounter;i++)
	{
		if((edgeset[i].v1.Vx==q.Vx && edgeset[i].v1.Vz==q.Vz) && (edgeset[i].v2.Vx==qPrime.Vx && edgeset[i].v2.Vz==qPrime.Vz))
			return 1;
		else if ((edgeset[i].v1.Vx==qPrime.Vx && edgeset[i].v1.Vz==qPrime.Vz) && (edgeset[i].v2.Vx==q.Vx && edgeset[i].v2.Vz==q.Vz))
			return 1; 
	}
	return 0;
}

int lpobst(struct vertex q,struct vertex qPrime, int workspace[33][33])
{
  //printf("lpobst\n");
  
  int cc1 = collisioncheck(q.Vx,q.Vz,workspace);
  //printf("cc1=%d\n", cc1);
  
  int cc2 = collisioncheck(qPrime.Vx,qPrime.Vz,workspace);
  //printf("cc2=%d\n", cc2);
	
  //printf("q=(%.2f, %.2f)\nq'=(%.2f, %.2f)\n",q.Vx, q.Vz, qPrime.Vx, qPrime.Vz);
  
  double dist = hypot((q.Vx - qPrime.Vx), (q.Vz - qPrime.Vz));
  
  //printf("dist=%.2f\n", dist);
  
  if (cc1 || cc2 )
  {
   // printf("Collision found\n");
		return 1;
  }
	else if(dist<0.01) 
  {
    //printf("Reached smallest point\n");
		return 0;
  }
	else 
	{
    //printf("Recusive check\n");
		struct vertex qDPrime;
		qDPrime.Vx=((q.Vx+qPrime.Vx)/2.0);
		qDPrime.Vz=((q.Vz+qPrime.Vz)/2.0);  
		return lpobst(q,qDPrime,workspace) || lpobst(qDPrime,qPrime,workspace);
	}

}

double pathCost(struct vertex* current)
{
	double cost = 0;
	while(current->previous!=0)
	{
    //printf("Check cost from (%.2f, %.2f)\n", current->Vx, current->Vz);
		cost += hypot((current->Vx - current->previous->Vx), (current->Vz - current->previous->Vz)); 
		current = current->previous;
	}
return cost;
}

//ROADMAP CONSTRUCTION PHASE
void prm_build_roadmap(int workspace[33][33])
{
  int i,j,k;
	for(i=0;i<5000;i++)
	{
		vertexset[i].Vx=0;
		vertexset[i].Vz=0;
	}

	for(i=0;i<5000;i++)
	{
		edgeset[i].v1.Vx=0;
		edgeset[i].v1.Vz=0;
		edgeset[i].v2.Vx=0;
		edgeset[i].v2.Vz=0;
	}
  
  //printf("Start sampling\n");
  
	while(vertexCounter<n)
	{

		double Px= uniformRandomNumber(a,b);
		double Pz= uniformRandomNumber(a,b);
    
   
		if(!collisioncheck(Px,Pz,workspace))
		{
      //printf("Point collision free\n\n");
			vertexset[vertexCounter].Vx=Px;   //adding vertex to vertex array
			vertexset[vertexCounter].Vz=Pz;   //adding vertex to vertex array
			vertexCounter++;
		}
	} 
  
  
	for(i=0; i<vertexCounter; i++)	//from all vertices to all other vertices
	{
    //printf("checking vertex %d\n", i);
    //printf("vertexCounter=%d\n", vertexCounter);
		struct dist dist[5000];
		for(j=0;j<vertexCounter;j++)  
		{
			dist[j].dbp=hypot((vertexset[j].Vx - vertexset[i].Vx), (vertexset[j].Vz - vertexset[i].Vz));  
			dist[j].drefv=j;

		} 
    //printf("checking vertex %d\n", i);
    
		sort(dist,vertexCounter);
		struct vertex q=vertexset[i];

    //printf("checking vertex %d\n", i);
		for(k=1;k<10;k++)
		{ 
      //printf("l1:checking vertex %d\n", i);
			struct vertex qPrime=vertexset[dist[k].drefv];
      //printf("l2:checking vertex %d\n", i);
      
			if(!contains(edgeset,q,qPrime)&& (!lpobst(q,qPrime,workspace)))
			{
      
        //printf("l3:checking vertex %d\n", i);
				edgeset[edgeCounter].v1=q;   //adding vertex(one end of edge) to edge array
				edgeset[edgeCounter].v2=qPrime;   //adding vertex(other end of edge) to edge array
				edgeCounter++;
			}
		} 
    //printf("checking vertex %d\n\n", i);


	}
  
   FILE* fp = fopen("map.m", "w");
     if (fp == NULL)
     {
        printf("Error opening file!\n");
        exit(1);
     }
   for(i=0;i<vertexCounter;i++)
	{
		//scatter(x, y);
		
		fprintf(fp, "scatter(%.2f,%.2f)\n",vertexset[i].Vx,vertexset[i].Vz);
	
		printf("vertex %d = (%.2f, %.2f)\n",i,vertexset[i].Vx,vertexset[i].Vz);
		
		
	}


	for(i=0;i<edgeCounter;i++)
	{
		
		//plot([x1 x2], [y1, y2]);
		fprintf(fp, "plot([%.2f %.2f],[%.2f,%.2f])\n",  edgeset[i].v1.Vx, edgeset[i].v2.Vx,	edgeset[i].v1.Vz,  	edgeset[i].v2.Vz);
		printf("v1=(%.2f, %.2f), v2=(%.2f, %.2f)\n",edgeset[i].v1.Vx, edgeset[i].v1.Vz,	edgeset[i].v2.Vx,  	edgeset[i].v2.Vz);
	
	}
  
  for(i=0;i<33;i++)
  {
    for(j=0;j<33;j++)
    {
      if(workspace[i][j]==1)
      { 
        fprintf(fp, "plot([(((%d)*0.07)-1) (((%d+1)*0.07)-1)],[((%d)*0.07)-1 ((%d)*0.07)-1])\n", i,i,j,j);
        fprintf(fp, "plot([(((%d)*0.07)-1) (((%d+1)*0.07)-1)], [((%d+1)*0.07)-1 ((%d+1)*0.07)-1])\n",i,i,j,j); 
        fprintf(fp, "plot([(((%d+1)*0.07)-1) (((%d+1)*0.07)-1)], [((%d)*0.07)-1 ((%d+1)*0.07)-1])\n",i,i,j,j);
        fprintf(fp, "plot([(((%d)*0.07)-1) (((%d)*0.07)-1)], [((%d)*0.07)-1 ((%d+1)*0.07)-1])\n",i,i,j,j); 
      }
    }  
  }
  
 
	
   fclose(fp);  
}


//QUERY PHASE
struct vertex* prm_query(struct vertex qStart,struct vertex qGoal,int workspace[33][33], int* pathLength)
{
	vertexset[vertexCounter].Vx=qStart.Vx;   //adding vertex to vertex array
	vertexset[vertexCounter].Vz=qStart.Vz;
  vertexset[vertexCounter].previous=0;
  
  int startPointer = vertexCounter;
  
	vertexCounter++;
	vertexset[vertexCounter].Vx=qGoal.Vx;   //adding vertex to vertex array
	vertexset[vertexCounter].Vz=qGoal.Vz;
  
  int goalPointer = vertexCounter;
  
  
	vertexCounter++;
	struct dist dist[5000];

  printf("added start and goal\n");

  int j;
	//code to add start to nearest vertex
	for(j=0;j<vertexCounter;j++)  
	{
		dist[j].dbp=hypot((vertexset[j].Vx - qStart.Vx), (vertexset[j].Vz - qStart.Vz));  
		dist[j].drefv=j;

	} 
	sort(dist,vertexCounter);
	int added=0;

  printf("Found sorted list of start neighbors\n");

  int k;
	for(k=1;k<15 && added==0;k++)
	{ 
		struct vertex qPrime=vertexset[dist[k].drefv];
		if(!contains(edgeset,qStart,qPrime) && (!lpobst(qStart,qPrime,workspace)))
		{
			edgeset[edgeCounter].v1=qStart;   
			edgeset[edgeCounter].v2=qPrime;   
			added++;
			edgeCounter++;
		}
	} 
  
  printf("Attached start\n");
  
	//code to add goal to nearest vertex
	for(j=0;j<vertexCounter;j++)  
	{
		dist[j].dbp=hypot((vertexset[j].Vx - qGoal.Vx), (vertexset[j].Vz - qGoal.Vz));  
		dist[j].drefv=j;

	} 
	sort(dist,vertexCounter);
  
  printf("Found sorted list of goal neighbors\n");
	added=0;

	for(k=1;k<15 && added==0;k++)
	{ 
		struct vertex qPrime=vertexset[dist[k].drefv];
		if(!contains(edgeset,qGoal,qPrime)&& (!lpobst(qGoal,qPrime,workspace)))
		{
			edgeset[edgeCounter].v1=qGoal;   
			edgeset[edgeCounter].v2=qPrime;   
			added++;
			edgeCounter++;
		}
	}  
  printf("Attached goal\n");
  
  printf("Edgecounter = %d\n", edgeCounter);
  
	struct dist* queue= malloc(sizeof(struct dist)*5000);
  
  int i;
  for(i=0; i<5000; i++)
  {
    queue[i].dbp = INFINITY;
    queue[i].drefv = -1;
  }
  
  for(i=0; i<vertexCounter; i++)
  {
    vertexset[i].previous = 0;
  }
  
	int qcounter=0;
	for(i=0;i<edgeCounter;i++)
	{
    printf("Check edge [(%.2f, %.2f) -> (%.2f, %.2f)] for (%.2f, %.2f)\n", edgeset[i].v1.Vx, edgeset[i].v1.Vz, edgeset[i].v2.Vx, edgeset[i].v2.Vz, qStart.Vx, qStart.Vz);
		if((edgeset[i].v1.Vx==qStart.Vx && edgeset[i].v1.Vz==qStart.Vz) || (edgeset[i].v2.Vx==qStart.Vx && edgeset[i].v2.Vz==qStart.Vz))
		{
			struct vertex point;
			if(edgeset[i].v1.Vx == qStart.Vx && edgeset[i].v1.Vz==qStart.Vz )
			{
				point.Vx = edgeset[i].v2.Vx;
				point.Vz = edgeset[i].v2.Vz;
			}
			else
			{
				point.Vx = edgeset[i].v1.Vx;
				point.Vz = edgeset[i].v1.Vz;
			}
			int j;
			for(j=0; j<vertexCounter; j++)
			{
				if((vertexset[j].Vx==point.Vx && vertexset[j].Vz==point.Vz))
				{
					break;
				}
			}	
			vertexset[j].previous=&vertexset[startPointer];
			queue[qcounter].dbp =  hypot((vertexset[j].Vx - qStart.Vx), (vertexset[j].Vz - qStart.Vz));  
			queue[qcounter].drefv = j;  
			qcounter++;
			sort(dist,qcounter);
		}
  }
    printf("qcounter = %d\n", qcounter);
    vertexset[startPointer].visited=1;
    
		while(qcounter>0)
		{
      printf("Queue: \n");
      int q_it;
      for(q_it=0; q_it<5000; q_it++)
      {
        if(queue[q_it].drefv != -1 && queue[q_it].drefv < 5000 && queue[q_it].dbp != INFINITY)
          printf("(%.2f, %.2f) @ %.2f\n", vertexset[queue[q_it].drefv].Vx, vertexset[queue[q_it].drefv].Vz, queue[q_it].dbp);
      }
    
			queue[0].dbp =INFINITY;
			struct vertex* current = &vertexset[queue[0].drefv];
      
      printf("Pop (%.2f, %.2f) off queue\n", current->Vx, current->Vz);
      printf("qcounter = %d\n", qcounter);
			current->visited=1;
			
			sort(queue, qcounter);
      qcounter--;

			for(i=0;i<edgeCounter;i++)
			{
        //printf("try edge %d of %d\n", i, edgeCounter);
				if((edgeset[i].v1.Vx==current->Vx && edgeset[i].v1.Vz==current->Vz) || (edgeset[i].v2.Vx==current->Vx && edgeset[i].v2.Vz==current->Vz))
				{
					struct vertex point;
					if(edgeset[i].v1.Vx == current->Vx && edgeset[i].v1.Vz==current->Vz )
					{
						point.Vx = edgeset[i].v2.Vx;
						point.Vz = edgeset[i].v2.Vz;
					}
					else
					{
						point.Vx = edgeset[i].v1.Vx;
						point.Vz = edgeset[i].v1.Vz;
					}
					int j;
					for(j=0; j<vertexCounter; j++)
					{
            //printf("find vertex in array, %d\n", j);
						if((vertexset[j].Vx==point.Vx && vertexset[j].Vz==point.Vz))
						{
							break;
						}
					}	
					if(vertexset[j].previous==0 && vertexset[j].visited==0)
					{
            printf("Add new edge to path\n");
						vertexset[j].previous=current;
						queue[qcounter].dbp =  hypot((vertexset[j].Vx - current->Vx), (vertexset[j].Vz - current->Vz)) + pathCost(current);  
						queue[qcounter].drefv = j;  
						qcounter++;
					}
					else if(vertexset[j].visited==0)
					{
            printf("Try updating edge\n");
						double tempCost = hypot((vertexset[j].Vx - current->Vx), (vertexset[j].Vz - current->Vz)) + pathCost(current);

						if(tempCost < pathCost(&vertexset[j]))
						{
              printf("Updating edge\n");
							vertexset[j].previous=current;
							queue[qcounter].dbp =  hypot((vertexset[j].Vx - current->Vx), (vertexset[j].Vz - current->Vz)) + pathCost(current);  
							queue[qcounter].drefv = j;  
							//qcounter++;
						}
					}
					//sort(dist,qcounter);
				}

			}

					sort(dist,qcounter);

		}
	
	int tempCount=0;
	struct vertex** tempPath = malloc(sizeof(struct vertex*)*5000);

	struct vertex* current = &vertexset[goalPointer];

  if(current->previous == 0)
  {
    printf("No path found to goal!\n");
    exit(1);
  }

	while(current != 0)
	{
    printf("Add point (%.2f, %.2f) to path\n", current->Vx, current->Vz);
		tempPath[tempCount] = current;
		tempCount++;
		current=current->previous;
	}
  
  
	struct vertex* path = malloc(sizeof(struct vertex)*tempCount);
  *pathLength = tempCount;

  printf("built temp path\n");
	for(i=0; i<tempCount; i++)
	{
    int tempIndex = tempCount-1-i;
    printf("Set path[%d] to tempPath[%d]\n", i, tempIndex);
		path[i].Vx = tempPath[tempIndex]->Vx;
		path[i].Vz = tempPath[tempIndex]->Vz;
    printf("path=(%.2f,%.2f)\n",path[i].Vx,path[i].Vz);
	}

  printf("flipped array:\n");
  //new path printing in matlab
  FILE* fp = fopen("path.m", "w");
     if (fp == NULL)
     {
        printf("Error opening file!\n");
        exit(1);
     }
   for(i=0;i<tempCount-1;i++)
	{
		//scatter(x, y);
		
		fprintf(fp, "scatter(%.2f,%.2f)\n",path[i].Vx,path[i].Vz);
	
		//printf("vertex %d = (%.2f, %.2f)\n",i,vertexset[i].Vx,vertexset[i].Vz);
		
		
	}


	for(i=0;i<tempCount-1;i++)
	{
		
		//plot([x1 x2], [y1, y2]);
		fprintf(fp, "plot([%.2f %.2f],[%.2f,%.2f])\n",  path[i].Vx, path[i+1].Vx,	path[i].Vz,  	path[i+1].Vz);

	
	}
  
  for(i=0;i<33;i++)
  {
    for(j=0;j<33;j++)
    {
      if(workspace[i][j]==1)
      { 
        fprintf(fp, "plot([(((%d)*0.07)-1) (((%d+1)*0.07)-1)],[((%d)*0.07)-1 ((%d)*0.07)-1])\n", i,i,j,j);
        fprintf(fp, "plot([(((%d)*0.07)-1) (((%d+1)*0.07)-1)], [((%d+1)*0.07)-1 ((%d+1)*0.07)-1])\n",i,i,j,j); 
        fprintf(fp, "plot([(((%d+1)*0.07)-1) (((%d+1)*0.07)-1)], [((%d)*0.07)-1 ((%d+1)*0.07)-1])\n",i,i,j,j);
        fprintf(fp, "plot([(((%d)*0.07)-1) (((%d)*0.07)-1)], [((%d)*0.07)-1 ((%d+1)*0.07)-1])\n",i,i,j,j); 
      }
    }  
  }
  
 
	
   fclose(fp);  
  
  //new path printing end
	free(tempPath);
  
  printf("Free-ed temp path\n");

	return path;

}
