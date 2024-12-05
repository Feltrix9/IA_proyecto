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

#define MAX_ROUTES 100000 // Máximo número de rutas permitidas
#define MAX_SEGMENTS 3 // Segmentos por ruta (inicio ? parada_1 ? parada_2 ? final)

#define LARGE  1000000000
#define BASE   10000000

#define max(x,y) ( (x) > (y) ? (x) : (y) )
#define min(x,y) ( (x) < (y) ? (x) : (y) )

typedef struct {
    unsigned start;
    unsigned stop1;
    unsigned stop2;
    unsigned goal;
} route;

// Arreglo para almacenar las rutas
route routes[MAX_ROUTES];
unsigned num_routes = 0;


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

snode* new_node() {
    snode* state = (snode*)malloc(sizeof(snode));
    if (state == NULL) {
        printf("Error al asignar memoria para un nuevo nodo.\n");
        exit(EXIT_FAILURE);
    }
    state->heapindex = 0;
    return state;
}

void write_solution_to_file(unsigned solution_index, unsigned g1, unsigned g2) {
    const char* filename = "solutions.txt";

    FILE* solution_file = fopen(filename, "a"); // "a" para agregar contenido al archivo
    if (solution_file == NULL) {
        printf("Error al abrir el archivo %s.\n", filename);
        exit(EXIT_FAILURE);
    }

    fprintf(solution_file, "%u %u %u\n", solution_index, g1, g2);
    fclose(solution_file);
}

void new_graph() {
    graph_node = (gnode*)calloc(num_gnodes, sizeof(gnode));
    unsigned y;
    for (y = 0; y < num_gnodes; ++y) {
        graph_node[y].id = y;
        graph_node[y].gmin = LARGE;
        graph_node[y].h1 = LARGE;
        graph_node[y].h2 = LARGE;
    }
    printf("Nuevo grafo creado con %u nodos.\n", num_gnodes);
}

void read_queries(const char* filename, unsigned* start, unsigned* stop1, unsigned* stop2, unsigned* goal, unsigned* num_routes) {
    FILE* file = fopen(filename, "r");
    if (file == NULL) {
        printf("Error al abrir el archivo %s.\n", filename);
        exit(EXIT_FAILURE);
    }

    unsigned inicio, parada1, parada2, final;
    *num_routes = 0;

    while (fscanf(file, "%u %u %u %u", &inicio, &parada1, &parada2, &final) == 4) {
        start[*num_routes] = inicio;
        stop1[*num_routes] = parada1;
        stop2[*num_routes] = parada2;
        goal[*num_routes] = final;
        (*num_routes)++;

        if (*num_routes >= MAX_ROUTES) {
            printf("Se alcanzó el límite máximo de rutas (%d).\n", MAX_ROUTES);
            break;
        }
    }

    fclose(file);
}

void initialize_parameters() {
    // Validar si los nodos están dentro del rango permitido
    if (start >= num_gnodes || goal >= num_gnodes) {
        printf("Error: Nodos fuera de rango (start=%u, goal=%u, max=%u).\n", start, goal, num_gnodes);
        exit(EXIT_FAILURE);
    }

    // Validar si los nodos tienen conexiones
    if (adjacent_table[start][0] == 0) {
        printf("Error: Nodo de inicio %u no tiene conexiones.\n", start);
        exit(EXIT_FAILURE);
    }
    if (adjacent_table[goal][0] == 0) {
        printf("Error: Nodo meta %u no tiene conexiones.\n", goal);
        exit(EXIT_FAILURE);
    }

    // Verificar que los nodos tengan conexiones
    printf("Nodo de inicio %u tiene %u conexiones\n", start, adjacent_table[start][0]);
    printf("Nodo meta %u tiene %u conexiones\n", goal, adjacent_table[goal][0]);

    // Configurar estados inicial y meta
    start_state = &graph_node[start];
    goal_state = &graph_node[goal];
    stat_percolations = 0;
}

 // Imprimir el nodo de inicio


//********************************************** Reading the file ******************************************************

void print_path(snode* goal_node) {
    snode* current = goal_node;
    printf("Camino desde %u a %u:\n", start, goal);
    while (current != NULL) {
        printf("Nodo %u (g1=%u, g2=%u) -> ", current->state, current->g1, current->g2);
        current = current->searchtree;
    }
    printf("Nodo %u\n", start);  // Imprimir el nodo de inicio
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

        printf("Expandiendo nodo %u con g1=%u, g2=%u\n", n->state, n->g1, n->g2);  // Trazas de depuración

        if (n->g2 >= graph_node[n->state].gmin || n->g2 + graph_node[n->state].h2 >= minf_solution) {
            stat_pruned++;
            if (next_recycled < MAX_RECYCLE) {
                recycled_nodes[next_recycled++] = n;
            }
            continue;
        }

        graph_node[n->state].gmin = n->g2;

        if (n->state == goal) {
            printf("GOAL alcanzado: [%d,%d] nsolutions:%d expanded:%llu generated:%llu heapsize:%d pruned:%d\n",
                   n->g1, n->g2, nsolutions, stat_expansions, stat_generated, sizeheap(), stat_pruned);
            print_path(n);  // Imprimir el camino hasta la meta
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

            // Agregar mensaje para verificar los nodos generados
            printf("Generando nodo sucesor %u con g1=%u, g2=%u, h1=%u, h2=%u\n", nsucc, newg1, newg2, h1, h2);
        }
    }

    return nsolutions;
}



void read_adjacent_table(const char* filename) {
    FILE* f = fopen(filename, "r");
    if (f == NULL) {
        fprintf(stderr, "Error: No se puede abrir el archivo %s.\n", filename);
        exit(EXIT_FAILURE);
    }

    int ori, dest, dist, t;
    int num_arcs = 0;
    fscanf(f, "%d %d", &num_gnodes, &num_arcs);

	int i;
    for (i = 0; i < num_arcs; i++) {
        fscanf(f, "%d %d %d %d", &ori, &dest, &dist, &t);

        // Validación: asegurarse de que no se desbordan índices
        if (ori >= MAXNODES || dest >= MAXNODES) {
            printf("Error: Nodo fuera de rango en arco %d ? %d.\n", ori, dest);
            continue;
        }

        adjacent_table[ori][0]++;
        adjacent_table[ori][adjacent_table[ori][0] * 3 - 2] = dest;
        adjacent_table[ori][adjacent_table[ori][0] * 3 - 1] = dist;
        adjacent_table[ori][adjacent_table[ori][0] * 3] = t;

        pred_adjacent_table[dest][0]++;
        pred_adjacent_table[dest][pred_adjacent_table[dest][0] * 3 - 2] = ori;
        pred_adjacent_table[dest][pred_adjacent_table[dest][0] * 3 - 1] = dist;
        pred_adjacent_table[dest][pred_adjacent_table[dest][0] * 3] = t;

        // Imprimir los arcos leídos
       // printf("Arco leído: %d -> %d, distancia: %d, tiempo: %d\n", ori, dest, dist, t);
    }

    fclose(f);
}
// Se mantiene el seguimiento de los nodos a través de los segmentos de la ruta.
int route_nodes[MAX_ROUTES][MAX_SEGMENTS + 1];  // Guardar nodos procesados por ruta (de inicio -> parada_1 -> parada_2 -> meta)

void call_boastar_and_log(FILE* output_file, unsigned current_start, unsigned current_goal, int instance) {
    printf("Ejecutando BOA* entre %u y %u\n", current_start, current_goal);
    
    // Declarar las variables para medir el tiempo de ejecución
    struct timeval tstart, tend;
    float runtime;

    // Configurar los nodos de inicio y meta
    start = current_start;
    goal = current_goal;

    // Imprimir las conexiones de los nodos de inicio y meta
    printf("Nodo de inicio %u tiene %u conexiones\n", start, adjacent_table[start][0]);
    unsigned i; 
    for (i = 1; i <= adjacent_table[start][0]; i++) {
        printf("  Conexión %u: Nodo %u con costo %u, tiempo %u\n", i, adjacent_table[start][i * 3 - 2], 
               adjacent_table[start][i * 3 - 1], adjacent_table[start][i * 3]);
    }

    printf("Nodo meta %u tiene %u conexiones\n", goal, adjacent_table[goal][0]);
    for (i = 1; i <= adjacent_table[goal][0]; i++) {
        printf("  Conexión %u: Nodo %u con costo %u, tiempo %u\n", i, adjacent_table[goal][i * 3 - 2], 
               adjacent_table[goal][i * 3 - 1], adjacent_table[goal][i * 3]);
    }

    // Ejecutar BOA* y registrar el resultado
    initialize_parameters();
    printf("Ejecutando BOA* para el segmento %u -> %u\n", current_start, current_goal);
    gettimeofday(&tstart, NULL);  // Capturar el tiempo de inicio
    boastar();
    gettimeofday(&tend, NULL);    // Capturar el tiempo de finalización

    // Calcular el tiempo de ejecución
    runtime = 1.0 * (tend.tv_sec - tstart.tv_sec) + 1.0 * (tend.tv_usec - tstart.tv_usec) / 1000000.0;

    // Guardar los resultados en el archivo de salida
    fprintf(output_file, "#instancia;%d;nsoluciones;%d;runtime;%f;nodos_expandidos;%llu;nodos_generados;%llu\n",
            instance, nsolutions, runtime * 1000, stat_expansions, stat_generated);
    printf("#instancia;%d;nsoluciones;%d;runtime;%f;nodos_expandidos;%llu;nodos_generados;%llu\n",
           instance, nsolutions, runtime * 1000, stat_expansions, stat_generated);
}


// antiguo MAIN
/*
int main(int argc, char* argv[]) {
    if (argc != 3) {
        printf("Uso: %s <NY-road-d.txt> <salida344.csv>\n", argv[0]);
        return 1;
    }

    read_adjacent_table(argv[1]);
    new_graph();

    unsigned start_points[] = {2995, 2515, 1443}; // Ejemplo de múltiples inicios
    unsigned goal_points[] = {2515, 1443, 20386};   // Ejemplo de múltiples metas
    unsigned i, j; // Declarar fuera del bucle

    for (i = 0; i < sizeof(start_points) / sizeof(start_points[0]); i++) {
        for (j = 0; j < sizeof(goal_points) / sizeof(goal_points[0]); j++) {
            start = start_points[i];
            goal = goal_points[j];
            printf("Resolviendo de %u a %u\n", start, goal);

            call_boastar(argv[2]);
            start_points[i] = 0; // Marcar el inicio como "usado"
        	goal_points[i] = 0;  // Marcar el objetivo como "usado"
        }
    }

    return 0;
}
*/


int main(int argc, char* argv[]) {
    if (argc != 4) {
        printf("Uso: %s <NY-road-d.txt> <NY-queries-2p.txt> <output.txt>\n", argv[0]);
        return 1;
    }

    // Leer el grafo desde NY-road-d.txt
    printf("Cargando grafo desde %s...\n", argv[1]);
    read_adjacent_table(argv[1]);
    new_graph();

    // Leer rutas desde NY-queries-2p.txt
    printf("Cargando rutas desde %s...\n", argv[2]);
    unsigned start[MAX_ROUTES], stop1[MAX_ROUTES], stop2[MAX_ROUTES], goal[MAX_ROUTES];
    unsigned num_routes = 0;

    // Llamada correcta a la función con los argumentos adecuados
    read_queries(argv[2], start, stop1, stop2, goal, &num_routes);

    // Abrir archivo de salida
    FILE* output_file = fopen(argv[3], "w");
    if (output_file == NULL) {
        printf("Error al abrir el archivo de salida %s.\n", argv[3]);
        return 1;
    }

    // Procesar cada ruta
    unsigned i;
    for (i = 0; i < num_routes; i++) {
        unsigned instance = 1 + i * 3; // Contador de instancias por segmento

        // Guardar la ruta completa en el archivo de salida
        fprintf(output_file, "Ruta #%u: %u -> %u -> %u -> %u\n", i + 1, start[i], stop1[i], stop2[i], goal[i]);
        printf("Procesando ruta #%u: %u -> %u -> %u -> %u\n", i + 1, start[i], stop1[i], stop2[i], goal[i]);

        // Procesar y mostrar cada segmento
        unsigned current_start = start[i];
        unsigned current_goal = stop1[i];
        route_nodes[i][0] = current_start;  // Guardar nodo de inicio
        fprintf(output_file, "  Segmento 1: %u -> %u\n", current_start, current_goal);
        printf("  Segmento 1: %u -> %u\n", current_start, current_goal);

        // Verificar si BOA* se ejecuta correctamente
        printf("Ejecutando BOA* para el segmento 1: %u -> %u\n", current_start, current_goal);
        call_boastar_and_log(output_file, current_start, current_goal, instance++);

        current_start = stop1[i];
        current_goal = stop2[i];
        route_nodes[i][1] = current_start;  // Guardar primera parada
        fprintf(output_file, "  Segmento 2: %u -> %u\n", current_start, current_goal);
        printf("  Segmento 2: %u -> %u\n", current_start, current_goal);
        
        // Verificar si BOA* se ejecuta correctamente
        printf("Ejecutando BOA* para el segmento 2: %u -> %u\n", current_start, current_goal);
        call_boastar_and_log(output_file, current_start, current_goal, instance++);

        current_start = stop2[i];
        current_goal = goal[i];
        route_nodes[i][2] = current_start;  // Guardar segunda parada
        route_nodes[i][3] = current_goal;  // Guardar meta
        fprintf(output_file, "  Segmento 3: %u -> %u\n", current_start, current_goal);
        printf("  Segmento 3: %u -> %u\n", current_start, current_goal);
        
        // Verificar si BOA* se ejecuta correctamente
        printf("Ejecutando BOA* para el segmento 3: %u -> %u\n", current_start, current_goal);
        call_boastar_and_log(output_file, current_start, current_goal, instance++);

        // Mostrar resultados y proceso del BOA para la ruta completa
        fprintf(output_file, "Resultados del BOA para Ruta #%u:\n", i + 1);
        unsigned j;
        for (j = 0; j < 3; j++) {
            fprintf(output_file, "  Nodo %u -> Nodo %u: %d soluciones, %llu nodos expandidos, %llu nodos generados.\n",
                    route_nodes[i][j], route_nodes[i][j + 1], nsolutions, stat_expansions, stat_generated);
            printf("  Nodo %u -> Nodo %u: %d soluciones, %llu nodos expandidos, %llu nodos generados.\n",
                   route_nodes[i][j], route_nodes[i][j + 1], nsolutions, stat_expansions, stat_generated);
        }

        // Imprimir el orden de las paradas al final de la ruta
        fprintf(output_file, "  Orden recorrido: ");
        for (j = 0; j <= 3; j++) {
            fprintf(output_file, "%u", route_nodes[i][j]);
            if (j < 3) fprintf(output_file, " -> ");
        }
        fprintf(output_file, "\n\n");
        printf("  Orden recorrido: ");
        for (j = 0; j <= 3; j++) {
            printf("%u", route_nodes[i][j]);
            if (j < 3) printf(" -> ");
        }
        printf("\n\n");
    }

    printf("Todas las rutas han sido procesadas y guardadas en %s.\n", argv[3]);
    fclose(output_file);

    return 0;
}







