/////////////////////////////////////////////////////////////////////
// Carlos Hernandez
// All rights reserved
/////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>

#define MAXNODES 4000000
#define MAXNEIGH 45
#define MAX_SOLUTIONS 1000000
#define MAX_RECYCLE   100000

#define LARGE  1000000000
#define BASE   10000000

#define max(x,y) ( (x) > (y) ? (x) : (y) )
#define min(x,y) ( (x) < (y) ? (x) : (y) )
#define MAX_NODES_IN_ROUTE 7 // Inicio + hasta 5 paradas + Meta


//********************************************** Main data structures ******************************************************
struct gnode;
typedef struct gnode gnode;

struct gnode // stores info needed for each graph node
{
  long long int id;
  unsigned h1;
  unsigned h2;
  unsigned long long int key;
  unsigned gmin;
  unsigned long heapindex;
};

struct snode;
typedef struct snode snode;

struct snode // BOA*'s search nodes
{
  int state;
  unsigned g1;
  unsigned g2;
  double key;
  unsigned long heapindex;
  snode *searchtree;
};


gnode* graph_node;
unsigned num_gnodes;
unsigned adjacent_table[MAXNODES][MAXNEIGH];
unsigned pred_adjacent_table[MAXNODES][MAXNEIGH];
unsigned goal, start;
gnode* start_state;
gnode* goal_state;
snode* start_node;

unsigned long long int stat_expansions = 0;
unsigned long long int stat_generated = 0;
unsigned long long int minf_solution = LARGE;

unsigned solutions[MAX_SOLUTIONS][2];
unsigned nsolutions = 0;
unsigned stat_pruned = 0;
unsigned stat_created = 0;


//********************************************** Binary Heap Data Structures ******************************************************

// --------------------------    Binary Heap for Dijkstra  -----------------------------------------
#define HEAPSIZEDIJ 3000000
gnode* heap_dij[HEAPSIZEDIJ];
unsigned long int heapsize_dij = 0;
unsigned long int stat_percolations = 0;

// ---------------------------------------------------------------
void percolatedown_dij(int hole, gnode* tmp) {
  int child;

  if (heapsize_dij != 0) {
    for (; 2 * hole <= heapsize_dij; hole = child) {
      child = 2 * hole;
      if (child != heapsize_dij && heap_dij[child + 1]->key < heap_dij[child]->key)
        ++child;
      if (heap_dij[child]->key < tmp->key) {
        heap_dij[hole] = heap_dij[child];
        heap_dij[hole]->heapindex = hole;
        ++stat_percolations;
      }
      else
        break;
    } // end for
    heap_dij[hole] = tmp;
    heap_dij[hole]->heapindex = hole;
  }
}
/* --------------------------------------------------------------- */
void percolateup_dij(int hole, gnode* tmp) {
  if (heapsize_dij != 0) {
    for (; hole > 1 && tmp->key < heap_dij[hole / 2]->key; hole /= 2) {
      heap_dij[hole] = heap_dij[hole / 2];
      heap_dij[hole]->heapindex = hole;
      ++stat_percolations;
    }
    heap_dij[hole] = tmp;
    heap_dij[hole]->heapindex = hole;
  }
}
/* --------------------------------------------------------------- */
void percolateupordown_dij(int hole, gnode* tmp) {
  if (heapsize_dij != 0) {
    if (hole > 1 && heap_dij[hole / 2]->key > tmp->key)
      percolateup_dij(hole, tmp);
    else
      percolatedown_dij(hole, tmp);
  }
}
/* --------------------------------------------------------------- */
void insertheap_dij(gnode* thiscell) {

  if (thiscell->heapindex == 0)
    percolateup_dij(++heapsize_dij, thiscell);
  else
    percolateupordown_dij(thiscell->heapindex, heap_dij[thiscell->heapindex]);
}
/* --------------------------------------------------------------- */
void deleteheap_dij(gnode* thiscell) {
  if (thiscell->heapindex != 0) {
    percolateupordown_dij(thiscell->heapindex, heap_dij[heapsize_dij--]);
    thiscell->heapindex = 0;
  }
}
/* --------------------------------------------------------------- */
gnode* topheap_dij() {
  if (heapsize_dij == 0)
    return NULL;
  return heap_dij[1];
}
/* --------------------------------------------------------------- */
void emptyheap_dij() {
  int i;

  for (i = 1; i <= heapsize_dij; ++i)
    heap_dij[i]->heapindex = 0;
  heapsize_dij = 0;
}

/* --------------------------------------------------------------- */
gnode* popheap_dij() {
  gnode* thiscell;

  if (heapsize_dij == 0)
    return NULL;
  thiscell = heap_dij[1];
  thiscell->heapindex = 0;
  percolatedown_dij(1, heap_dij[heapsize_dij--]);
  return thiscell;
}

int sizeheap_dij() {
  return heapsize_dij;
}

gnode* posheap_dij(int i) {
  return heap_dij[i];
}

// --------------------------    Binary Heap for BOA*  -----------------------------------------
#define HEAPSIZE 40000000
snode* heap[HEAPSIZE];
unsigned long int heapsize = 0;

// ---------------------------------------------------------------
void percolatedown(int hole, snode* tmp) {
  int child;

  if (heapsize != 0) {
    for (; 2 * hole <= heapsize; hole = child) {
      child = 2 * hole;
      if (child != heapsize && heap[child + 1]->key < heap[child]->key)
        ++child;
      if (heap[child]->key < tmp->key) {
        heap[hole] = heap[child];
        heap[hole]->heapindex = hole;
        ++stat_percolations;
      }
      else
        break;
    } // end for
    heap[hole] = tmp;
    heap[hole]->heapindex = hole;
  }
}
/* --------------------------------------------------------------- */
void percolateup(int hole, snode* tmp) {
  if (heapsize != 0) {
    for (; hole > 1 && tmp->key < heap[hole / 2]->key; hole /= 2) {
      heap[hole] = heap[hole / 2];
      heap[hole]->heapindex = hole;
      ++stat_percolations;
    }
    heap[hole] = tmp;
    heap[hole]->heapindex = hole;
  }
}
/* --------------------------------------------------------------- */
void percolateupordown(int hole, snode* tmp) {
  if (heapsize != 0) {
    if (hole > 1 && heap[hole / 2]->key > tmp->key)
      percolateup(hole, tmp);
    else
      percolatedown(hole, tmp);
  }
}
/* --------------------------------------------------------------- */
void insertheap(snode* thiscell) {
  if (thiscell->heapindex == 0)
    percolateup(++heapsize, thiscell);
  else
    percolateupordown(thiscell->heapindex, heap[thiscell->heapindex]);
}
/* --------------------------------------------------------------- */
void deleteheap(snode* thiscell) {
  if (thiscell->heapindex != 0) {
    percolateupordown(thiscell->heapindex, heap[heapsize--]);
    thiscell->heapindex = 0;
  }
}
/* --------------------------------------------------------------- */
snode* topheap() {
  if (heapsize == 0)
    return NULL;
  return heap[1];
}
/* --------------------------------------------------------------- */
void emptyheap() {
  int i;

  for (i = 1; i <= heapsize; ++i)
    heap[i]->heapindex = 0;
  heapsize = 0;
}

/* --------------------------------------------------------------- */
snode* popheap() {
  snode* thiscell;

  if (heapsize == 0)
    return NULL;
  thiscell = heap[1];
  thiscell->heapindex = 0;
  percolatedown(1, heap[heapsize--]);
  return thiscell;
}

int sizeheap() {
  return heapsize;
}

long int opensize() {
  return heapsize_dij;
}
snode* posheap(int i) {
  return heap[i];
}
// --------------------------    Binary Heap end --------------------------------------------




//********************************************** Reading the file ******************************************************

void read_adjacent_table(const char* filename) {
	FILE* f;
	int i, ori, dest, dist, t;
	f = fopen(filename, "r");
	int num_arcs = 0;
    if (f == NULL) {
        fprintf(stderr, "Error: No se puede abrir el archivo %s.\n", filename);
        exit(EXIT_FAILURE);
    }
	fscanf(f, "%d %d", &num_gnodes, &num_arcs);
	fscanf(f, "\n");
//	printf("%d %d", num_gnodes, num_arcs);
	for (i = 0; i < num_gnodes; i++)
		adjacent_table[i][0] = 0;

	for (i = 0; i < num_arcs; i++) {
		fscanf(f, "%d %d %d %d\n", &ori, &dest, &dist, &t);
	//	printf("%d %d %d %d\n", ori, dest, dist, t);
		adjacent_table[ori - 1][0]++;
		adjacent_table[ori - 1][adjacent_table[ori - 1][0] * 3 - 2] = dest - 1;
		adjacent_table[ori - 1][adjacent_table[ori - 1][0] * 3 - 1] = dist;
		adjacent_table[ori - 1][adjacent_table[ori - 1][0] * 3] = t;

		pred_adjacent_table[dest - 1][0]++;
		pred_adjacent_table[dest - 1][pred_adjacent_table[dest - 1][0] * 3 - 2] = ori - 1;
		pred_adjacent_table[dest - 1][pred_adjacent_table[dest - 1][0] * 3 - 1] = dist;
		pred_adjacent_table[dest - 1][pred_adjacent_table[dest - 1][0] * 3] = t;
	}
	fclose(f);
}

void new_graph() {
	int y;
	if (graph_node == NULL) {
		graph_node = (gnode*) calloc(num_gnodes, sizeof(gnode));
		for (y = 0; y < num_gnodes; ++y) 		{
			graph_node[y].id = y;
			graph_node[y].gmin = LARGE;
			graph_node[y].h1 = LARGE;
			graph_node[y].h2 = LARGE;
		}
	}
}


//********************************************** BOA* ******************************************************

void initialize_parameters() {
    start_state = &graph_node[start];
    goal_state = &graph_node[goal];
    stat_percolations = 0;
}

int backward_dijkstra(int dim) {
    int i;
	for (i = 0; i < num_gnodes; ++i)
        graph_node[i].key = LARGE;
    emptyheap_dij();
    goal_state->key = 0;
    insertheap_dij(goal_state);

    while (topheap_dij() != NULL) {
        gnode* n;
        gnode* pred;
        short d;
        n = popheap_dij();
        if (dim == 1)
            n->h1 = n->key;
        else
            n->h2 = n->key;
        ++stat_expansions;
        for (d = 1; d < pred_adjacent_table[n->id][0] * 3; d += 3) {
            pred = &graph_node[pred_adjacent_table[n->id][d]];
            int new_weight = n->key + pred_adjacent_table[n->id][d + dim];
            if (pred->key > new_weight) {
                pred->key = new_weight;
                insertheap_dij(pred);
            }
        }
    }
    return 1;
}

snode* new_node() {
    snode* state = (snode*)malloc(sizeof(snode));
    state->heapindex = 0;
    return state;
}

void write_solution_to_file(unsigned solution_index, unsigned g1, unsigned g2) {
    char filename[256];
    snprintf(filename, sizeof(filename), "solution_%u.txt", solution_index);

    FILE* solution_file = fopen(filename, "w");
    if (solution_file == NULL) {
        printf("Error al abrir el archivo %s.\n", filename);
        exit(EXIT_FAILURE);
    }

    fprintf(solution_file, "Solución #%u\n", solution_index);
    fprintf(solution_file, "Costo G1: %u\n", g1);
    fprintf(solution_file, "Costo G2: %u\n", g2);

    fclose(solution_file);
}

int boastar() {
    snode* recycled_nodes[MAX_RECYCLE];
    int next_recycled = 0;
    nsolutions = 0;
    stat_pruned = 0;
    emptyheap();

    start_node = new_node();
    ++stat_created;
    start_node->state = start;
    start_node->g1 = 0;
    start_node->g2 = 0;
    start_node->key = 0;
    start_node->searchtree = NULL;
    insertheap(start_node);

    stat_expansions = 0;
    while (topheap() != NULL) {
        snode* n = popheap();
        short d;

        if (n->g2 >= graph_node[n->state].gmin || n->g2 + graph_node[n->state].h2 >= minf_solution) {
            stat_pruned++;
            if (next_recycled < MAX_RECYCLE) {
                recycled_nodes[next_recycled++] = n;
            }
            continue;
        }

        graph_node[n->state].gmin = n->g2;

        if (n->state == goal) {
            printf("GOAL [%d,%d] nsolutions:%d expanded:%llu generated:%llu heapsize:%d pruned:%d\n",
                   n->g1, n->g2, nsolutions, stat_expansions, stat_generated, sizeheap(), stat_pruned);

            solutions[nsolutions][0] = n->g1;
            solutions[nsolutions][1] = n->g2;
            write_solution_to_file(nsolutions, n->g1, n->g2); // Crear archivo .txt
            nsolutions++;

            if (nsolutions > MAX_SOLUTIONS) {
                printf("Maximum number of solutions reached, increase MAX_SOLUTIONS!\n");
                exit(1);
            }
            if (minf_solution > n->g2)
                minf_solution = n->g2;
            continue;
        }

        ++stat_expansions;

        for (d = 1; d < adjacent_table[n->state][0] * 3; d += 3) {
            snode* succ;
            double newk1, newk2, newkey;
            unsigned nsucc = adjacent_table[n->state][d];
            unsigned cost1 = adjacent_table[n->state][d + 1];
            unsigned cost2 = adjacent_table[n->state][d + 2];

            unsigned newg1 = n->g1 + cost1;
            unsigned newg2 = n->g2 + cost2;
            unsigned h1 = graph_node[nsucc].h1;
            unsigned h2 = graph_node[nsucc].h2;

            if (newg2 >= graph_node[nsucc].gmin || newg2 + h2 >= minf_solution)
                continue;

            newk1 = newg1 + h1;
            newk2 = newg2 + h2;

            if (next_recycled > 0) {
                succ = recycled_nodes[--next_recycled];
            } else {
                succ = new_node();
                ++stat_created;
            }

            succ->state = nsucc;
            stat_generated++;

            newkey = newk1 * (double)BASE + newk2;
            succ->searchtree = n;
            succ->g1 = newg1;
            succ->g2 = newg2;
            succ->key = newkey;
            insertheap(succ);
        }
    }

    return nsolutions > 0;
}

/* ------------------------------------------------------------------------------*/
void call_boastar(const char* output_filename) {
    FILE* output_file = fopen(output_filename, "w");
    if (output_file == NULL) {
        printf("Error al abrir el archivo de salida %s.\n", output_filename);
        exit(EXIT_FAILURE);
    }

    float runtime;
    struct timeval tstart, tend;

    initialize_parameters();

    gettimeofday(&tstart, NULL);

    // Calcula h1 y h2 usando Dijkstra inverso
    backward_dijkstra(1);
    backward_dijkstra(2);

    // Llama a BOA*
    boastar();

    gettimeofday(&tend, NULL);
    runtime = 1.0 * (tend.tv_sec - tstart.tv_sec) + 1.0 * (tend.tv_usec - tstart.tv_usec) / 1000000.0;

    // Imprime resultados en el formato solicitado
    fprintf(output_file, "#instancia;%d;nsoluciones;%d;runtime;%f;nodos_expandidos;%llu;nodos_generados;%llu\n",
            1, nsolutions, runtime * 1000, stat_expansions, stat_generated);

    fclose(output_file);
}


/*----------------------------------------------------------------------------------*/
int main(int argc, char* argv[]) {
    if (argc != 3) {
        printf("Uso: %s <NY-road-d.txt> <salida344.csv>\n", argv[0]);
        return 1;
    }

    read_adjacent_table(argv[1]);
    new_graph();

    unsigned start_points[] = {180833, 100000, 150000}; // Ejemplo de múltiples inicios
    unsigned goal_points[] = {83149, 90000, 160000};   // Ejemplo de múltiples metas
    unsigned i, j; // Declarar fuera del bucle

    // Tamaños de los arreglos
    unsigned start_count = sizeof(start_points) / sizeof(start_points[0]);
    unsigned goal_count = sizeof(goal_points) / sizeof(goal_points[0]);

    for (i = 0; i < start_count; i++) {
        printf("Iterando por start_points[%u] = %u\n", i, start_points[i]);
        for (j = 0; j < goal_count; j++) {
            printf("Iterando por goal_points[%u] = %u\n", j, goal_points[j]);

            // Configuración de inicio y meta
            start = start_points[i];
            goal = goal_points[j];

            // Mostrar información de inicio y meta
            printf("Resolviendo de %u a %u\n", start, goal);

            // Llamada al algoritmo
            call_boastar(argv[2]);
        }
    }

    return 0;
<<<<<<< Updated upstream
=======
}*/

void read_routes(const char* filename, unsigned** start_points, unsigned** goal_points, unsigned* num_routes) {
    FILE* file = fopen(filename, "r");
    if (file == NULL) {
        printf("Error al abrir el archivo %s.\n", filename);
        exit(EXIT_FAILURE);
    }

    unsigned buffer[MAX_NODES_IN_ROUTE];
    unsigned route_count = 0;
    unsigned i; // Declarar fuera de los bucles
    *num_routes = 0;

    while (!feof(file)) {
        unsigned num_nodes = 0;

        // Leer nodos de una línea
        for (i = 0; i < MAX_NODES_IN_ROUTE; i++) {
            if (fscanf(file, "%u", &buffer[i]) != 1) {
                break;
            }
            num_nodes++;
        }

        if (num_nodes < 2) {
            continue; // Ignorar rutas no válidas
        }

        // Generar pares start-goal
        for (i = 0; i < num_nodes - 1; i++) {
            start_points[route_count][i] = buffer[i];
            goal_points[route_count][i] = buffer[i + 1];
            route_count++;
        }
    }

    fclose(file);
    *num_routes = route_count;
}

int main(int argc, char* argv[]) {
    if (argc != 3) {
        printf("Uso: %s <NY-queries-1p> <archivo_salida.csv>\n", argv[0]);
        return 1;
    }

    // Leer rutas desde el archivo
    unsigned* start_points[MAXNODES];
    unsigned* goal_points[MAXNODES];
    unsigned num_routes = 0;
    unsigned i; // Declarar fuera del bucle

    for (i = 0; i < MAXNODES; i++) { // Usar 'i' ya declarada
        start_points[i] = malloc(MAX_NODES_IN_ROUTE * sizeof(unsigned));
        goal_points[i] = malloc(MAX_NODES_IN_ROUTE * sizeof(unsigned));
    }

    read_routes(argv[1], start_points, goal_points, &num_routes);

    // Procesar cada ruta con BOA*
    for (i = 0; i < num_routes; i++) { // Reutilizar 'i' declarada anteriormente
        start = start_points[i][0];
        goal = goal_points[i][0];
        printf("Procesando ruta de %u a %u...\n", start, goal);
        call_boastar(argv[2]);
    }

    printf("Todas las rutas han sido procesadas.\n");

    // Liberar memoria
    for (i = 0; i < MAXNODES; i++) { // Reutilizar 'i'
        free(start_points[i]);
        free(goal_points[i]);
    }

    return 0;
>>>>>>> Stashed changes
}



