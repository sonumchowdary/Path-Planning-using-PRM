struct vertex
{
  double Vx;
  double Vz;
  int visited;
  struct vertex* previous;
};

struct edge
{
  struct vertex v1;
  struct vertex v2; 
};
struct dist
{
   double dbp; 
   int drefv; 
};
 
double uniformRandomNumber(double a, double b);
double collisioncheck(double x, double z, int workspace[33][33]);
void sort(struct dist dist[],int vertexcounter);
int contains(struct edge edgeset,struct vertex q,struct vertex qPrime);
int lpobst(struct vertex q,struct vertex qPrime, int workspace[33][33]);
double pathCost(struct vertex* current);
double prm_build_roadmap();
struct vertex* prm_query(struct vertex qStart,struct vertex qGoal,int workspace[33][33], int* pathLength);

